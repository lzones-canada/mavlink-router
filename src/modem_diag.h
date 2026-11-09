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
#pragma once

#include <climits>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "endpoint.h"
#include "pollable.h"
#include "timeout.h"

/*
 * Microhard P900 diagnostics channel ("The Diagnostics Channel Protocol"
 * application note). Sets the modem output power and reads it back, so the
 * boost state reported to the GCS is confirmed by the modem itself.
 *
 * The transport is whatever endpoint main.conf defines for the *_boost
 * section (UART on the bench, UDP through the serial converter in
 * production). This class writes raw diag packets to that endpoint and reads
 * the raw replies from its fd; the endpoint is never routed MAVLink.
 */
namespace modem_diag {

// CRC-16 ANSI (poly 0x8005 reflected, init 0xFFFF), as in Appendix B.
// Covers the size byte, MAC, magic number, control byte and payload.
uint16_t crc16(const uint8_t *data, size_t len);

// Build a complete request to the local modem (MAC 0) with "response needed"
// set. @payload is the command ID followed by its data.
std::vector<uint8_t> build_request(uint16_t magic, const std::vector<uint8_t> &payload);

} // namespace modem_diag

class ModemDiag : public Pollable {
public:
    static constexpr uint8_t POWER_NORMAL_DBM = 26;
    static constexpr uint8_t POWER_BOOST_DBM = 30;
    static constexpr int POWER_UNKNOWN = -1;
    static constexpr int HW_REVISION_UNKNOWN = -1;
    static constexpr int TEMPERATURE_UNKNOWN = INT_MIN;

    explicit ModemDiag(std::shared_ptr<Endpoint> endpoint);
    ~ModemDiag() override;

    std::string get_name() const { return _endpoint->get_name(); }

    // Write the immediate output power (par 29), then read it back to confirm.
    // Retries a few times if the read-back value doesn't match.
    void set_power(uint8_t dbm);

    // Read the immediate output power (par 29), the power the modem uses
    // after a reset (par 3) and the temperature (par 55).
    void read_power();

    // Read the modem's fixed identity: firmware version (par 5), product
    // string (par 104) and hardware revision (par 111).
    void read_info();

    // Re-read the output power and temperature every @interval_ms, so a modem
    // that resets (and falls back to its after-reset power) is noticed.
    void start_periodic_read(uint32_t interval_ms);

    // Output power last reported by the modem, POWER_UNKNOWN until confirmed
    // or after a failed read.
    int power_dbm() const { return _power_dbm; }

    // Identity as reported by the modem; strings are empty until known.
    const std::string &firmware_version() const { return _firmware_version; } // e.g. "v1.09"
    const std::string &product() const { return _product; }
    const std::string &mac_address() const { return _mac_address; } // "12:34:56:78:AB:CD"
    int hardware_revision() const { return _hw_revision; }          // HW_REVISION_UNKNOWN until read

    // Modem temperature in degrees C, TEMPERATURE_UNKNOWN until read.
    // Refreshed with each periodic read.
    int temperature_c() const { return _temperature_c; }

    // Called whenever power_dbm() changes.
    std::function<void()> on_power_change;

    int handle_read() override;
    bool handle_canwrite() override { return false; }
    bool is_critical() override { return false; }

private:
    struct Request {
        std::vector<uint8_t> payload; // command ID followed by its data
        bool power_cmd;               // part of a set_power() sequence
    };

    void _queue(std::vector<uint8_t> payload, bool power_cmd);
    void _send_next();
    void _complete_request();
    bool _response_timeout_cb(void *data);
    bool _gap_timeout_cb(void *data);
    bool _periodic_read_cb(void *data);
    bool _current_reads(uint8_t par) const;
    void _read_failed();

    void _parse_rx();
    void _handle_packet(const uint8_t *pkt, size_t len);
    void _handle_write_reply(uint8_t control, const uint8_t *data, size_t len);
    void _handle_read_reply(uint8_t control, const uint8_t *data, size_t len);

    void _set_power_dbm(int dbm);
    void _check_target();

    std::shared_ptr<Endpoint> _endpoint;

    std::deque<Request> _requests;
    bool _awaiting = false;      // a request is on the wire, waiting for its reply
    Request _current{};          // the request on the wire
    uint16_t _current_magic = 0;
    uint16_t _next_magic = 0x0123;
    Timeout *_response_timeout = nullptr;
    Timeout *_gap_timeout = nullptr;

    std::vector<uint8_t> _rx;

    int _power_dbm = POWER_UNKNOWN;
    int _target_dbm = POWER_UNKNOWN; // set_power() value still being confirmed
    int _attempts = 0;

    std::string _firmware_version;
    std::string _product;
    std::string _mac_address;
    int _hw_revision = HW_REVISION_UNKNOWN;
    int _temperature_c = TEMPERATURE_UNKNOWN;
};
