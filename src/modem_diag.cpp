/*
 * This file is part of the MAVLink Router project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "modem_diag.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>

#include <common/log.h>

#include "mainloop.h"

// Packet layout: size(1) MAC(6) magic(2) control(1) payload(n) crc(2).
// "size" counts everything except itself and the CRC.
static constexpr size_t HDR_LEN = 1 + 6 + 2 + 1;  // up to and including control
static constexpr uint8_t MIN_SIZE = 6 + 2 + 1 + 1; // MAC + magic + control + cmd ID
static constexpr uint8_t MAX_SIZE = 254;
static constexpr size_t RX_LIMIT = 1024;           // drop garbage beyond this

static constexpr uint8_t CTRL_RESPONSE_NEEDED = 0x02;
static constexpr uint8_t CTRL_ERROR = 0x08;
static constexpr uint8_t CTRL_ACCESS_DENIED = 0x10;

static constexpr uint8_t CMD_READ = 0x00;
static constexpr uint8_t CMD_WRITE = 0x04;

// Parameter IDs (Appendix A)
static constexpr uint8_t PAR_POWER_AFTER_RESET = 3; // proxy, 1 byte, dBm
static constexpr uint8_t PAR_VERSION = 5;           // read only, null-terminated string
static constexpr uint8_t PAR_POWER_IMMEDIATE = 29;  // immediate, 1 byte, dBm
static constexpr uint8_t PAR_TEMPERATURE = 55;      // read only, 1 byte, degrees C + 55
static constexpr uint8_t PAR_PRODUCT = 104;         // read only, null-terminated string
static constexpr uint8_t PAR_HW_REVISION = 111;     // read only, 1 byte

static constexpr int TEMPERATURE_OFFSET = 55;

static constexpr uint32_t RESPONSE_TIMEOUT_MS = 500;
static constexpr uint32_t REQUEST_GAP_MS = 25; // protocol requires >= 20 ms between requests
static constexpr int MAX_POWER_ATTEMPTS = 3;

namespace modem_diag {

uint16_t crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
        }
    }
    return crc;
}

std::vector<uint8_t> build_request(uint16_t magic, const std::vector<uint8_t> &payload)
{
    std::vector<uint8_t> pkt;
    pkt.reserve(HDR_LEN + payload.size() + 2);

    pkt.push_back(static_cast<uint8_t>(MIN_SIZE - 1 + payload.size())); // size
    pkt.insert(pkt.end(), 6, 0x00);                                    // local MAC
    pkt.push_back(magic >> 8);                                         // magic, high byte first
    pkt.push_back(magic & 0xff);
    pkt.push_back(CTRL_RESPONSE_NEEDED);
    pkt.insert(pkt.end(), payload.begin(), payload.end());

    const uint16_t crc = crc16(pkt.data(), pkt.size());
    pkt.push_back(crc >> 8); // CRC, high byte first
    pkt.push_back(crc & 0xff);

    return pkt;
}

} // namespace modem_diag

ModemDiag::ModemDiag(std::shared_ptr<Endpoint> endpoint, const ModemPowerConfig &power_conf)
    : _endpoint(std::move(endpoint))
    , _normal_dbm(static_cast<int>(power_conf.normal_power))
    , _boost_dbm(static_cast<int>(power_conf.boost_power))
    , _allow_boost(power_conf.allow_boost)
{
    // "port_boost" -> "Port". A name without a prefix is used as is.
    const std::string &name = _endpoint->get_name();
    const size_t underscore = name.find('_');
    _label = (underscore == 0 || underscore == std::string::npos) ? name : name.substr(0, underscore);
    _label[0] = static_cast<char>(toupper(static_cast<unsigned char>(_label[0])));

    // Share the endpoint's fd for polling. The endpoint owns and closes it.
    fd = _endpoint->fd;
}

ModemDiag::~ModemDiag()
{
    // Stop ~Pollable() closing the endpoint's fd.
    fd = -1;
}

void ModemDiag::set_power(uint8_t dbm)
{
    if (!_writes_enabled) {
        return; // Mainloop doesn't call this then, but never write regardless
    }

    // Already writing and confirming this power.
    if (_target_dbm == dbm) {
        return;
    }

    // A newer power command replaces any power command not yet sent.
    _requests.erase(std::remove_if(_requests.begin(),
                                   _requests.end(),
                                   [](const Request &r) { return r.power_cmd; }),
                    _requests.end());

    _target_dbm = dbm;
    _attempts = 1;

    _queue({CMD_WRITE, PAR_POWER_IMMEDIATE, dbm}, true);
    _queue({CMD_READ, PAR_POWER_IMMEDIATE}, true);
}

void ModemDiag::read_power()
{
    _queue({CMD_READ, PAR_POWER_IMMEDIATE, PAR_POWER_AFTER_RESET, PAR_TEMPERATURE}, false);
}

void ModemDiag::read_info()
{
    // Variable-size parameters are read on their own, as the protocol recommends.
    _queue({CMD_READ, PAR_VERSION}, false);
    _queue({CMD_READ, PAR_PRODUCT}, false);
    _queue({CMD_READ, PAR_HW_REVISION}, false);
}

void ModemDiag::start_periodic_read(uint32_t interval_ms)
{
    Mainloop::get_instance().add_timeout(
        interval_ms,
        std::bind(&ModemDiag::_periodic_read_cb, this, std::placeholders::_1),
        this);
}

bool ModemDiag::_periodic_read_cb(void *data)
{
    // Skip this round if anything is in flight; that will read the power anyway.
    if (!_awaiting && _requests.empty()) {
        _queue({CMD_READ, PAR_POWER_IMMEDIATE, PAR_TEMPERATURE}, false);
    }
    return true; // keep repeating
}

bool ModemDiag::_current_reads(uint8_t par) const
{
    const auto &p = _current.payload;
    return p[0] == CMD_READ && std::find(p.begin() + 1, p.end(), par) != p.end();
}

void ModemDiag::_read_failed()
{
    // Values this request was refreshing are no longer known to be current.
    if (_current_reads(PAR_TEMPERATURE)) {
        _temperature_c = TEMPERATURE_UNKNOWN;
    }
    if (_current_reads(PAR_POWER_IMMEDIATE)) {
        _set_power_dbm(POWER_UNKNOWN);
        _check_target();
    }
}

void ModemDiag::_queue(std::vector<uint8_t> payload, bool power_cmd)
{
    _requests.push_back(Request{std::move(payload), power_cmd});
    _send_next();
}

void ModemDiag::_send_next()
{
    if (_awaiting || _gap_timeout != nullptr || _requests.empty()) {
        return;
    }

    _current = _requests.front();
    _requests.pop_front();
    _current_magic = _next_magic++;

    std::vector<uint8_t> pkt = modem_diag::build_request(_current_magic, _current.payload);

    struct buffer buf = {};
    buf.data = pkt.data();
    buf.len = pkt.size();

    const int r = _endpoint->write_msg(&buf);
    if (r < 0) {
        log_error("Modem %s: diag write failed (%d)", label().c_str(), r);
    }
    log_trace("Modem %s: sent diag cmd 0x%02x, magic 0x%04x",
              label().c_str(),
              _current.payload[0],
              _current_magic);

    _awaiting = true;
    _response_timeout = Mainloop::get_instance().add_timeout(
        RESPONSE_TIMEOUT_MS,
        std::bind(&ModemDiag::_response_timeout_cb, this, std::placeholders::_1),
        this);
}

void ModemDiag::_complete_request()
{
    _awaiting = false;

    if (_response_timeout != nullptr) {
        Mainloop::get_instance().del_timeout(_response_timeout);
        _response_timeout = nullptr;
    }

    // Respect the minimum gap between requests before sending the next one.
    if (!_requests.empty()) {
        _gap_timeout = Mainloop::get_instance().add_timeout(
            REQUEST_GAP_MS,
            std::bind(&ModemDiag::_gap_timeout_cb, this, std::placeholders::_1),
            this);
    }
}

bool ModemDiag::_response_timeout_cb(void *data)
{
    _response_timeout = nullptr; // removed by returning false

    const uint8_t cmd = _current.payload[0];
    log_warning("Modem %s: no reply to diag %s (magic 0x%04x)",
                label().c_str(),
                cmd == CMD_WRITE ? "write" : "read",
                _current_magic);

    _read_failed();
    _complete_request();
    return false;
}

bool ModemDiag::_gap_timeout_cb(void *data)
{
    _gap_timeout = nullptr; // removed by returning false
    _send_next();
    return false;
}

int ModemDiag::handle_read()
{
    uint8_t tmp[256];
    const ssize_t r = ::read(fd, tmp, sizeof(tmp));

    if (r < 0) {
        if (errno == EAGAIN) {
            return 0;
        }
        log_error("Modem %s: diag read failed (%m)", label().c_str());
        return -errno;
    }

    _rx.insert(_rx.end(), tmp, tmp + r);
    if (_rx.size() > RX_LIMIT) {
        log_warning("Modem %s: discarding %zu bytes of unparsed diag data",
                    label().c_str(),
                    _rx.size());
        _rx.clear();
        return r;
    }

    _parse_rx();
    return r;
}

void ModemDiag::_parse_rx()
{
    while (!_rx.empty()) {
        const uint8_t size = _rx[0];

        // Not a plausible size byte: drop it and resync on the next byte.
        if (size < MIN_SIZE || size > MAX_SIZE) {
            _rx.erase(_rx.begin());
            continue;
        }

        const size_t total = size + 3u;
        if (_rx.size() < total) {
            return; // wait for the rest of the packet
        }

        const uint16_t crc_rx = (_rx[total - 2] << 8) | _rx[total - 1];
        if (modem_diag::crc16(_rx.data(), total - 2) != crc_rx) {
            log_debug("Modem %s: diag CRC mismatch, resyncing", label().c_str());
            _rx.erase(_rx.begin());
            continue;
        }

        _handle_packet(_rx.data(), total);
        _rx.erase(_rx.begin(), _rx.begin() + total);
    }
}

void ModemDiag::_handle_packet(const uint8_t *pkt, size_t len)
{
    const uint16_t magic = (pkt[7] << 8) | pkt[8];
    const uint8_t control = pkt[9];
    const uint8_t cmd = pkt[10];
    const uint8_t *data = pkt + HDR_LEN + 1; // after the command ID
    const size_t data_len = len - HDR_LEN - 1 - 2;

    if (!_awaiting || magic != _current_magic || cmd != _current.payload[0]) {
        log_debug("Modem %s: ignoring unexpected diag reply (cmd 0x%02x, magic 0x%04x)",
                  label().c_str(),
                  cmd,
                  magic);
        return;
    }

    // Replies always carry the modem's real MAC, even to local (MAC 0) requests.
    char mac[18];
    snprintf(mac, sizeof(mac), "%02X:%02X:%02X:%02X:%02X:%02X",
             pkt[1], pkt[2], pkt[3], pkt[4], pkt[5], pkt[6]);
    if (_mac_address != mac) {
        _mac_address = mac;
        log_info("Modem %s: MAC %s", label().c_str(), mac);
    }

    if (control & CTRL_ACCESS_DENIED) {
        log_error("Modem %s: diag %s denied, modem requires a diag login",
                  label().c_str(),
                  cmd == CMD_WRITE ? "write" : "read");
        // Nothing was written or read: the power is not confirmed.
        _read_failed();
    } else if (cmd == CMD_WRITE) {
        _handle_write_reply(control, data, data_len);
    } else {
        _handle_read_reply(control, data, data_len);
    }

    _complete_request();
}

void ModemDiag::_handle_write_reply(uint8_t control, const uint8_t *data, size_t len)
{
    if (!(control & CTRL_ERROR)) {
        log_debug("Modem %s: power write accepted", label().c_str());
        return;
    }

    // Error replies list (parameter ID, error code) pairs.
    // 1 = unknown parameter, 2 = out of range, 3 = wrong size.
    if (len < 2) {
        log_error("Modem %s: power write rejected", label().c_str());
    }
    for (size_t i = 0; i + 1 < len; i += 2) {
        log_error("Modem %s: power write rejected, par %u error %u",
                  label().c_str(),
                  data[i],
                  data[i + 1]);
    }
}

void ModemDiag::_handle_read_reply(uint8_t control, const uint8_t *data, size_t len)
{
    if (control & CTRL_ERROR) {
        log_warning("Modem %s: diag read reported an error", label().c_str());
    }

    // Reply is a list of parameter IDs, each followed by its value. Parameter
    // sizes are not sent, so only parameters we know the size of can be parsed.
    int power = POWER_UNKNOWN;
    size_t i = 0;
    while (i < len) {
        const uint8_t par = data[i++];

        if (par == PAR_VERSION || par == PAR_PRODUCT) {
            const void *nul = memchr(data + i, '\0', len - i);
            if (nul == nullptr) {
                log_warning("Modem %s: truncated string par %u in diag reply", label().c_str(), par);
                break;
            }
            const size_t n = static_cast<const uint8_t *>(nul) - (data + i);
            std::string &dst = (par == PAR_VERSION) ? _firmware_version : _product;
            dst.assign(reinterpret_cast<const char *>(data + i), n);
            log_info("Modem %s: %s %s",
                     label().c_str(),
                     par == PAR_VERSION ? "Firmware Version" : "Product",
                     dst.c_str());
            i += n + 1;
        } else if (par == PAR_POWER_IMMEDIATE || par == PAR_POWER_AFTER_RESET
                   || par == PAR_TEMPERATURE || par == PAR_HW_REVISION) {
            if (i >= len) {
                break;
            }
            const uint8_t value = data[i++];
            if (par == PAR_POWER_IMMEDIATE) {
                power = value;
            } else if (par == PAR_POWER_AFTER_RESET) {
                log_info("Modem %s: Power After Reset %u dBm", label().c_str(), value);
            } else if (par == PAR_TEMPERATURE) {
                const int temperature = value - TEMPERATURE_OFFSET;
                // Info only on change, so the periodic read doesn't log every time
                if (temperature != _temperature_c) {
                    log_info("Modem %s: Temperature %d C", label().c_str(), temperature);
                } else {
                    log_debug("Modem %s: Temperature %d C", label().c_str(), temperature);
                }
                _temperature_c = temperature;
            } else {
                _hw_revision = value;
                log_info("Modem %s: Hardware Revision %d", label().c_str(), _hw_revision);
            }
        } else {
            log_warning("Modem %s: unexpected par %u in diag reply", label().c_str(), par);
            break; // unknown size, can't parse further
        }
    }

    // Only reads that asked for the output power update it.
    if (!_current_reads(PAR_POWER_IMMEDIATE)) {
        return;
    }

    if (power == POWER_UNKNOWN) {
        log_warning("Modem %s: diag reply did not include output power", label().c_str());
    } else {
        log_debug("Modem %s: Power Output %d dBm", label().c_str(), power);
    }

    _set_power_dbm(power);
    _check_target();
}

void ModemDiag::_set_power_dbm(int dbm)
{
    if (dbm == _power_dbm) {
        return;
    }

    if (dbm != POWER_UNKNOWN) {
        log_info("Modem %s: Power Output %d dBm", label().c_str(), dbm);

        if (!_writes_enabled && dbm != _normal_dbm) {
            log_warning("Modem %s: output power %d dBm differs from NormalPower %d dBm, "
                        "not corrected (Power writes Disabled)",
                        label().c_str(),
                        dbm,
                        _normal_dbm);
        }
    }

    _power_dbm = dbm;
    if (on_power_change) {
        on_power_change();
    }
}

void ModemDiag::_check_target()
{
    // Only relevant to the read-back that ends a set_power() sequence.
    if (_target_dbm == POWER_UNKNOWN || !_current.power_cmd) {
        return;
    }

    if (_power_dbm == _target_dbm) {
        log_info("Modem %s: Power Output %d dBm confirmed", label().c_str(), _target_dbm);
        _target_dbm = POWER_UNKNOWN;
        return;
    }

    if (_attempts >= MAX_POWER_ATTEMPTS) {
        log_error("Modem %s: output power not confirmed at %d dBm after %d attempts",
                  label().c_str(),
                  _target_dbm,
                  _attempts);
        _target_dbm = POWER_UNKNOWN;
        return;
    }

    _attempts++;
    log_warning("Modem %s: output power not confirmed at %d dBm, retrying (%d/%d)",
                label().c_str(),
                _target_dbm,
                _attempts,
                MAX_POWER_ATTEMPTS);
    _queue({CMD_WRITE, PAR_POWER_IMMEDIATE, static_cast<uint8_t>(_target_dbm)}, true);
    _queue({CMD_READ, PAR_POWER_IMMEDIATE}, true);
}
