/*
 * This file is part of the MAVLink Router project
 *
 * Copyright (C) 2016  Intel Corporation. All rights reserved.
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
#include "mainloop.h"

#include <assert.h>
#include <signal.h>
#include <sys/epoll.h>
#include <sys/timerfd.h>
#include <unistd.h>

#include <atomic>
#include <memory>

#include <common/log.h>
#include <common/util.h>

#include "autolog.h"
#include "tlog.h"

static std::atomic<bool> should_exit{false};


Mainloop Mainloop::_instance{};
bool Mainloop::_initialized = false;

static void exit_signal_handler(int signum)
{
    Mainloop::instance().request_exit(0);
}

static void setup_signal_handlers()
{
    struct sigaction sa = {};

    sa.sa_flags = SA_NOCLDSTOP;
    sa.sa_handler = exit_signal_handler;
    sigaction(SIGTERM, &sa, nullptr);
    sigaction(SIGINT, &sa, nullptr);

    sa.sa_handler = SIG_IGN;
    sigaction(SIGPIPE, &sa, nullptr);
}

Mainloop &Mainloop::init()
{
    assert(_initialized == false);

    _initialized = true;

    return _instance;
}

void Mainloop::teardown()
{
    _initialized = false;
}

Mainloop &Mainloop::instance()
{
    return _instance;
}

void Mainloop::request_exit(int retcode)
{
    _retcode = retcode;
    should_exit.store(true, std::memory_order_relaxed);
}

int Mainloop::open()
{
    _retcode = -1;

    if (epollfd != -1) {
        return -EBUSY;
    }

    epollfd = epoll_create1(EPOLL_CLOEXEC);

    if (epollfd == -1) {
        log_error("%m");
        return -1;
    }

    _retcode = 0;

    return 0;
}

int Mainloop::mod_fd(int fd, void *data, int events) const
{
    struct epoll_event epev = {};

    epev.events = events;
    epev.data.ptr = data;

    if (epoll_ctl(epollfd, EPOLL_CTL_MOD, fd, &epev) < 0) {
        log_error("Could not mod fd (%m)");
        return -1;
    }

    return 0;
}

int Mainloop::add_fd(int fd, void *data, int events) const
{
    struct epoll_event epev = {};

    epev.events = events;
    epev.data.ptr = data;

    if (epoll_ctl(epollfd, EPOLL_CTL_ADD, fd, &epev) < 0) {
        log_error("Could not add fd to epoll (%m)");
        return -1;
    }

    return 0;
}

int Mainloop::remove_fd(int fd) const
{
    if (epoll_ctl(epollfd, EPOLL_CTL_DEL, fd, nullptr) < 0) {
        log_error("Could not remove fd from epoll (%m)");
        return -1;
    }

    return 0;
}

int Mainloop::handle_modem_tx(const std::shared_ptr<UdpEndpoint> &udpEndpoint, const struct buffer *buf)
{
    // reader result.
    int r = 0;

    // Determine the current write mode based on our current endpoint.
    writeToPort = udpEndpoint->get_name() == "port_modem";
    writeToStbd = udpEndpoint->get_name() == "stbd_modem";

    // Handle actions based on current state
    switch (modemState) {
        case PORT_TX:
            // Confirm we are on our endpoint for message sending
            if(writeToPort) {
                r = udpEndpoint->write_msg(buf);
                log_debug("PORT_TX: msg_id:%u, seq:%u, sysid:%u", buf->curr.msg_id, buf->curr.seq_id, buf->curr.src_sysid);
                prev_port_modem = true;
                prev_stbd_modem = false;
            }
            break;
        case STBD_TX:
            if(writeToStbd) {
                r = udpEndpoint->write_msg(buf);
                log_debug("STBD_TX: msg_id:%u, seq:%u, sysid:%u", buf->curr.msg_id, buf->curr.seq_id, buf->curr.src_sysid);
                prev_port_modem = false;
                prev_stbd_modem = true;
            }
            break;
        case BOTH_TX:
        // Transmit the message from both modems
        if (writeToPort && prev_port_modem) {
            if (buf->curr.seq_id != lastSeqIdStbd) {
                r = udpEndpoint->write_msg(buf);
                log_debug("PORT_TX: msg_id:%u, seq:%u, sysid:%u", buf->curr.msg_id, buf->curr.seq_id, buf->curr.src_sysid);
                lastSeqIdPort = buf->curr.seq_id;
                messageCounter++;
                // Switch to the Stbd modem
                if (messageCounter % 2 == 0) {
                    prev_port_modem = false;
                    prev_stbd_modem = true;
                }
            } else {
                log_debug("PORT_TX: Skipping duplicate message with seq_id: %u", buf->curr.seq_id);
            }
        } else if (writeToStbd && prev_stbd_modem) {
            if (buf->curr.seq_id != lastSeqIdPort) {
                r = udpEndpoint->write_msg(buf);
                log_debug("STBD_TX: msg_id:%u, seq:%u, sysid:%u", buf->curr.msg_id, buf->curr.seq_id, buf->curr.src_sysid);
                lastSeqIdStbd = buf->curr.seq_id;
                messageCounter++;
                // Switch to the Port modem
                if (messageCounter % 2 == 0) {
                    prev_port_modem = true;
                    prev_stbd_modem = false;
                }
            } else {
                log_debug("STBD_TX: Skipping duplicate message with seq_id: %u", buf->curr.seq_id);
            }
        }
        break;
        case BOTH_OFF:
            // Transmit a message just once to indicate the modems are turning off
            // and then continue without further transmitting messages
            if (writeToPort && prev_port_modem) {
                r = udpEndpoint->write_msg(buf);
                // Update the state to indicate that both modems are off
                prev_port_modem = false;
                log_debug("BOTH_OFF[%s]: msg_id:%u, seq:%u, sysid:%u", udpEndpoint->get_name().c_str(),buf->curr.msg_id, buf->curr.seq_id, buf->curr.src_sysid);
            } else if (writeToStbd && prev_stbd_modem) {
                r = udpEndpoint->write_msg(buf);
                // Update the state to indicate that both modems are off
                prev_stbd_modem = false;
                log_debug("BOTH_OFF[%s]: msg_id:%u, seq:%u, sysid:%u", udpEndpoint->get_name().c_str(),buf->curr.msg_id, buf->curr.seq_id, buf->curr.src_sysid);
            }
            break;
    }

    return r;
}

int Mainloop::convert_gps_to_mavlink1(const std::shared_ptr<UdpEndpoint>& udpEndpoint, const struct buffer* buf)
{
    // Initialize temp gps placeholder message
    mavlink_message_t tmp_gps{};

    // Update the appropriate GPS cache based on GPS1 or GPS2
    if (buf->curr.msg_id == MAVLINK_MSG_ID_GPS_RAW_INT) {
        // Copy over payload and msg length
        memmove(tmp_gps.payload64, buf->curr.payload, buf->curr.payload_len);
        tmp_gps.len = buf->curr.payload_len;
        // Decode GPS_RAW_INT message into GPS_RAW_INT struct for MAV1
        mavlink_msg_gps_raw_int_decode(&tmp_gps, &_gps1_cache);
        _gps1_valid = true;
    } else if (buf->curr.msg_id == MAVLINK_MSG_ID_GPS2_RAW) {
        // Initialize GPS2 message for MAV1
        mavlink_gps2_raw_t gps2 = {};
        // Copy over payload and msg length
        memmove(tmp_gps.payload64, buf->curr.payload, buf->curr.payload_len);
        tmp_gps.len = buf->curr.payload_len;
        // GPS2_RAW has extra fields (dgps_age, dgps_numch) — must decode separately
        mavlink_msg_gps2_raw_decode(&tmp_gps, &gps2);
        // Transcribe common fields into GPS_RAW_INT layout
        _gps2_cache.time_usec          = gps2.time_usec;
        _gps2_cache.lat                = gps2.lat;
        _gps2_cache.lon                = gps2.lon;
        _gps2_cache.alt                = gps2.alt;
        _gps2_cache.eph                = gps2.eph;
        _gps2_cache.epv                = gps2.epv;
        _gps2_cache.vel                = gps2.vel;
        _gps2_cache.cog                = gps2.cog;
        _gps2_cache.fix_type           = gps2.fix_type;
        _gps2_cache.satellites_visible = gps2.satellites_visible;
        _gps2_cache.alt_ellipsoid      = gps2.alt_ellipsoid;
        _gps2_cache.h_acc              = gps2.h_acc;
        _gps2_cache.v_acc              = gps2.v_acc;
        _gps2_cache.vel_acc            = gps2.vel_acc;
        _gps2_cache.hdg_acc            = gps2.hdg_acc;
        _gps2_cache.yaw                = gps2.yaw;
        _gps2_valid = true;
    }

    // GPS2 wins only on better fix_type, OR on tied fix_type with more satellites; GPS1 takes all remaining ties
    const mavlink_gps_raw_int_t *best_gps = &_gps1_cache;
    uint8_t gps_source = 1;
    if (_gps2_valid && (!_gps1_valid || _gps2_cache.fix_type > _gps1_cache.fix_type ||
        (_gps2_cache.fix_type == _gps1_cache.fix_type && _gps2_cache.satellites_visible > _gps1_cache.satellites_visible))) {
        best_gps = &_gps2_cache;
        gps_source = 2;
    }

    // Only send when GPS message arrived is feeding the selected source.
    const bool is_fresh = (best_gps == &_gps1_cache && buf->curr.msg_id == MAVLINK_MSG_ID_GPS_RAW_INT)
                        || (best_gps == &_gps2_cache && buf->curr.msg_id == MAVLINK_MSG_ID_GPS2_RAW);
    if (!is_fresh) {
        return 0;
    }

    // GPS_FIX_TYPE: 0=No GPS, 1=No Fix, 2=2D Fix, 3=3D Fix, 4=DGPS, 5=RTK Float, 6=RTK Fixed
    if (best_gps->fix_type < GPS_FIX_TYPE_2D_FIX) {
        log_trace(" <> Tracker GPS: no lock (GPS[1] fix=%u sats=%u, GPS[2] fix=%u sats=%u)",
                _gps1_cache.fix_type, _gps1_cache.satellites_visible,
                _gps2_cache.fix_type, _gps2_cache.satellites_visible);
        return 0;
    }

    // Log once on initial source selection and on every source switch
    if (gps_source != _active_gps_source) {
        log_trace(" <> Tracker GPS[%u] -> GPS[%u] fix_type=%u, satellites=%u",
                _active_gps_source, gps_source, best_gps->fix_type, best_gps->satellites_visible);
        _active_gps_source = gps_source;
    }

    // Temporary set to MAVlink1
    mavlink_set_proto_version(MAVLINK_COMM_0, 1);

    // Initialize MAV1 message
    mavlink_message_t mav1_msg = {};
    // Buffer to store outgoing MAVLink1 message
    uint8_t data[MAVLINK_MAX_PACKET_LEN] = {};
    struct buffer out = {};

    // Encode the MAVLink 1 GPS_RAW_INT message
    mavlink_msg_gps_raw_int_encode(buf->curr.src_sysid, buf->curr.src_compid, &mav1_msg, best_gps);

    // Serialize the MAVLink 1 message into the send buffer
    out.len = mavlink_msg_to_send_buffer(data, &mav1_msg);
    out.data = data;

    // Write the message to the UDP endpoint
    const int r = udpEndpoint->write_msg(&out);

    // Set back to MAVlink2
    mavlink_set_proto_version(MAVLINK_COMM_0, 2);

    return r;
}

int Mainloop::write_msg(const std::shared_ptr<Endpoint> &e, const struct buffer *buf)
{
    // reader result.
    int r = 0;

    // Confirm endpoint is UdpEndpoint
    auto udpEndpoint = std::dynamic_pointer_cast<UdpEndpoint>(e);

    // Custom action for GCS Modems
    if (udpEndpoint && (udpEndpoint->get_name() == "port_modem" || udpEndpoint->get_name() == "stbd_modem")) {
        r = handle_modem_tx(udpEndpoint, buf);
    } else if (udpEndpoint && udpEndpoint->get_name() == "tracker" && (buf->curr.msg_id == MAVLINK_MSG_ID_GPS_RAW_INT || buf->curr.msg_id == MAVLINK_MSG_ID_GPS2_RAW)) {
        // Intercept GPS_RAW_INT for Tracker Endpoint (Convert MAV2 -> MAV1)
        r = convert_gps_to_mavlink1(udpEndpoint, buf);
    } else {
        // Proceed as before.
        r = e->write_msg(buf);
    }

    /*
     * If endpoint would block, add EPOLLOUT event to get notified when it's
     * possible to write again
     */
    if (r == -EAGAIN) {
        mod_fd(e->fd, e.get(), EPOLLIN | EPOLLOUT);
    }

    return r;
}

void Mainloop::handle_modem_boost(const struct buffer *buf, const bool boost_modem, const std::shared_ptr<UdpEndpoint> &modem_diag)
{
    // log out if modem UART is not available
    if (modem_diag == nullptr) {
        log_error("MODEM ENDPOINT NOT FOUND");
        return;
    }

    // Create and initialize the MODEM_CMD structure
    MODEM_CMD modem_cmd = {};
    modem_cmd.size = 0x0C;                      // Size of the packet (excluding size byte and CRC)
    memset(modem_cmd.mac_address, 0x00, sizeof(modem_cmd.mac_address)); // MAC address
    modem_cmd.magic_number = htons(0x0123);     // Converts 0x1234 to big-endian format

    // Populate payload
    modem_cmd.control_byte = 0x02;              // Control byte: response needed
    modem_cmd.cmd_id = 0x04;                    // Command ID for Write
    modem_cmd.param_id = 0x1D;                  // Output Power - Immediate (Decimal 29)

    // Trigger boost command
    if (boost_modem) {
        modem_cmd.param_value = 0x1E;           // Power value for enabled state
        modem_cmd.crc1 = 0x10;                  // Hardcoded CRC1
        modem_cmd.crc2 = 0x1F;                  // Hardcoded CRC2
        log_info(" > %s [%d]%s: Enable  - %d dBm", modem_diag->get_type().c_str(), modem_diag->fd, modem_diag->get_name().c_str(),modem_cmd.param_value);
    // Disable boost command
    } else {
        modem_cmd.param_value = 0x1A;           // Power value for disabled state
        modem_cmd.crc1 = 0xD3;                  // Hardcoded CRC1
        modem_cmd.crc2 = 0x1E;                  // Hardcoded CRC2
        log_info(" > %s [%d]%s: Disable - %d dBm", modem_diag->get_type().c_str(), modem_diag->fd, modem_diag->get_name().c_str(),modem_cmd.param_value);
    }

    // Allocate memory for the buffer and copy the structure into it
    struct buffer modem_buf;
    modem_buf.len = sizeof(modem_cmd);
    modem_buf.data = modem_cmd.data; // Use union's raw data;

    // Send the message
    modem_diag->write_msg(&modem_buf);

    return;
}

void Mainloop::handle_station_ctrl_msg(const struct buffer *buf)
{
    // Extract a pointer to mavlink_station_ctrl_t from the payload data in buf
    const mavlink_station_ctrl_t *station_ctrl = (mavlink_station_ctrl_t *)buf->curr.payload;

    // Set the port_modem and stbd_modem flags based on the station control flags
    port_modem  = station_ctrl->flags & STATION_CTRL_FLAGS::TX_PORT_MODEM;
    stbd_modem  = station_ctrl->flags & STATION_CTRL_FLAGS::TX_STBD_MODEM;
    modem_boost = station_ctrl->flags & STATION_CTRL_FLAGS::MODEM_BOOST;

    // Check if the modem boost flag is set and if the modem endpoint is available
    if(modem_boost != prev_modem_boost) {
        for (const auto &modem_diag : this->gcs_modems) {
            handle_modem_boost(buf, modem_boost, modem_diag);
        }
        // track state of the modem boost.
        prev_modem_boost = modem_boost;
    }

    // Update the state machine based on the current state of the modems
    if (port_modem && !stbd_modem) {
        modemState = PORT_TX;
    } else if (!port_modem && stbd_modem) {
        modemState = STBD_TX;
    } else if (port_modem && stbd_modem) {
        modemState = BOTH_TX;
        messageCounter = 0;
    } else {
        modemState = BOTH_OFF;
    }

    return;
}

void Mainloop::send_station_status_msg(const struct buffer *buf)
{
    // Initialize message
    mavlink_message_t mav_msg = {};
    mavlink_station_status_t station_status = {};

    // Buffer to store outgoing message
    uint8_t data[MAVLINK_MAX_PACKET_LEN] = {};
    struct buffer out = {};

    // Build STATION_STATUS based on current modem states
    uint8_t status_flags = 0;
    // Tx Modem Port modem status - GCS
    if (port_modem) {
        status_flags |= STATION_STATUS_FLAGS_TX_PORT;
    }
    // Tx Modem Stbd modem status - GCS
    if (stbd_modem) {
        status_flags |= STATION_STATUS_FLAGS_TX_STBD;
    }
    // Tx Modem Boost status - GCS
    if (modem_boost) {
        status_flags |= STATION_STATUS_FLAGS_MODEM_BOOST;
    }
    // Set the flags
    station_status.flags = status_flags;

    // Encode the STATION_STATUS message
    mavlink_msg_station_status_encode(MAV_COMP_ID_AUTOPILOT1, MAV_COMP_ID_ONBOARD_COMPUTER, &mav_msg, &station_status);
    
    // Serialize the message
    out.len = mavlink_msg_to_send_buffer(data, &mav_msg);
    out.data = data;

    // Populate buffer metadata for routing
    out.curr.msg_id = MAVLINK_MSG_ID_STATION_STATUS;
    out.curr.src_sysid = MAV_COMP_ID_AUTOPILOT1;
    out.curr.src_compid = MAV_COMP_ID_ONBOARD_COMPUTER;
    out.curr.target_sysid = -1;  // broadcast
    out.curr.target_compid = -1; // broadcast
    out.curr.seq_id = buf->curr.seq_id;
    out.curr.payload_len = mav_msg.len;
    out.curr.payload = reinterpret_cast<uint8_t*>(_MAV_PAYLOAD_NON_CONST(&mav_msg));

    // Inject the status message back into routing
    route_msg(&out);
}

void Mainloop::route_msg(struct buffer *buf)
{
    bool unknown = true;

    // Special case for intercepting and handling GCS Station Modem Control Messages
    if (buf->curr.msg_id == MAVLINK_MSG_ID_STATION_CTRL) {
        // Intercept and handle the station control message
        handle_station_ctrl_msg(buf);
        send_station_status = true;
    }

    for (const auto &e : this->g_endpoints) {
        auto acceptState = e->accept_msg(buf);

        switch (acceptState) {
        case Endpoint::AcceptState::Accepted:
            log_trace("Endpoint [%d] accepted message %u to %d/%d from %u/%u",
                      e->fd,
                      buf->curr.msg_id,
                      buf->curr.target_sysid,
                      buf->curr.target_compid,
                      buf->curr.src_sysid,
                      buf->curr.src_compid);
            if (write_msg(e, buf) == -EPIPE) { // only TCP endpoints should return -EPIPE
                should_process_tcp_hangups = true;
            }
            unknown = false;
            break;
        case Endpoint::AcceptState::Filtered:
            log_trace("Endpoint [%d] filtered out message %u to %d/%d from %u/%u",
                      e->fd,
                      buf->curr.msg_id,
                      buf->curr.target_sysid,
                      buf->curr.target_compid,
                      buf->curr.src_sysid,
                      buf->curr.src_compid);
            unknown = false;
            break;
        case Endpoint::AcceptState::Rejected:
            // fall through
        default:
            break; // do nothing (will count as unknown)
        }
    }

    if (unknown) {
        _errors_aggregate.msg_to_unknown++;
        log_trace("Message %u to unknown sysid/compid: %d/%d",
                  buf->curr.msg_id,
                  buf->curr.target_sysid,
                  buf->curr.target_compid);
    }

     // Special case for sending GCS Station Modem Status Messages in response to control messages
    if (send_station_status) {
        // Must flip flag first
        send_station_status = false;
        send_station_status_msg(buf);
    }
}

void Mainloop::process_tcp_hangups()
{
    // Remove endpoints, which are invalid
    for (auto it = g_endpoints.begin(); it != g_endpoints.end();) {
        if (it->get()->get_type() == ENDPOINT_TYPE_TCP) {
            auto *tcp_endpoint = static_cast<TcpEndpoint *>(it->get());
            if (!tcp_endpoint->is_valid()) {
                it = g_endpoints.erase(it);
            } else {
                ++it;
            }
        } else {
            ++it;
        }
    }

    should_process_tcp_hangups = false;
}

void Mainloop::handle_tcp_connection()
{
    log_debug("TCP Server: New client");

    auto *tcp = new TcpEndpoint{"dynamic"};

    int fd = tcp->accept(g_tcp_fd);
    if (fd == -1) {
        goto accept_error;
    }

    g_endpoints.emplace_back(tcp);
    this->add_fd(g_endpoints.back()->fd, g_endpoints.back().get(), EPOLLIN);

    return;

accept_error:
    log_error("TCP Server: Could not accept TCP connection (%m)");
    delete tcp;
}

int Mainloop::loop()
{
    const int max_events = 8;
    struct epoll_event events[max_events];
    int r;

    if (epollfd < 0) {
        return -EINVAL;
    }

    setup_signal_handlers();

    add_timeout(LOG_AGGREGATE_INTERVAL_SEC * MSEC_PER_SEC,
                std::bind(&Mainloop::_log_aggregate_timeout, this, std::placeholders::_1),
                this);

    while (!should_exit.load(std::memory_order_relaxed)) {
        int i;

        r = epoll_wait(epollfd, events, max_events, -1);
        if (r < 0 && errno == EINTR) {
            continue;
        }

        for (i = 0; i < r; i++) {
            if (events[i].data.ptr == &g_tcp_fd) {
                handle_tcp_connection();
                continue;
            }

            auto *p = static_cast<Pollable *>(events[i].data.ptr);

            if (events[i].events & EPOLLIN) {
                int rd = p->handle_read();
                if (rd < 0 && !p->is_valid()) {
                    // Only TcpEndpoint may become invalid after a read
                    should_process_tcp_hangups = true;
                }
            }

            if (events[i].events & EPOLLOUT) {
                if (!p->handle_canwrite()) {
                    mod_fd(p->fd, p, EPOLLIN);
                }
            }

            if (events[i].events & EPOLLERR) {
                log_error("poll error for fd %i", p->fd);

                if (p->is_critical()) {
                    log_error("Critical fd %i got error, exiting", p->fd);
                    request_exit(EXIT_FAILURE);
                } else {
                    log_debug("Non-critical fd %i, error is okay.", p->fd);
                }
            }
        }

        if (should_process_tcp_hangups) {
            process_tcp_hangups();
        }

        _del_timeouts();
    }

    if (_log_endpoint != nullptr) {
        _log_endpoint->stop();
    }

    clear_endpoints();

    // free all remaning Timeouts
    while (_timeouts != nullptr) {
        Timeout *current = _timeouts;
        _timeouts = current->next;
        remove_fd(current->fd);
        delete current;
    }

    return _retcode;
}

bool Mainloop::_log_aggregate_timeout(void *data)
{
    if (_errors_aggregate.msg_to_unknown > 0) {
        log_warning("%u messages to unknown endpoints in the last %d seconds",
                    _errors_aggregate.msg_to_unknown,
                    LOG_AGGREGATE_INTERVAL_SEC);
        _errors_aggregate.msg_to_unknown = 0;
    }

    for (const auto &e : g_endpoints) {
        e->log_aggregate(LOG_AGGREGATE_INTERVAL_SEC);
    }
    return true;
}

void Mainloop::print_statistics()
{
    for (const auto &e : g_endpoints) {
        e->print_statistics();
    }
}

static bool _print_statistics_timeout_cb(void *data)
{
    auto *mainloop = static_cast<Mainloop *>(data);
    mainloop->print_statistics();
    return true;
}

bool Mainloop::dedup_check_msg(const buffer *buf)
{
    return _msg_dedup.check_packet(buf->data, buf->len)
        == Dedup::PacketStatus::NEW_PACKET_OR_TIMED_OUT;
}

bool Mainloop::add_endpoints(const Configuration &config)
{
    // Create UART and UDP endpoints
    if (config.sniffer_sysid != 0) {
        Endpoint::sniffer_sysid = config.sniffer_sysid;
        log_info("An endpoint with sysid %u on it will sniff all messages",
                 Endpoint::sniffer_sysid);
    }
    // Create UART endpoints
    for (const auto &conf : config.uart_configs) {
        auto uart = std::make_shared<UartEndpoint>(conf.name);

        if (!uart->setup(conf)) {
            return false;
        }

        g_endpoints.push_back(uart);
        auto endpoint = g_endpoints.back();
        this->add_fd(endpoint->fd, endpoint.get(), EPOLLIN);
    }
    // Create UDP endpoints
    for (const auto &conf : config.udp_configs) {
        auto udp = std::make_shared<UdpEndpoint>(conf.name);

        if (!udp->setup(conf)) {
            return false;
        }

        // Check if the UART is a GCS Modem tagged with "boost"
        if (conf.name.find("boost") != std::string::npos) {
            gcs_modems.emplace_back(udp);
        }
        // Proceed as before - Don't add endpoint for our modem UARTs
        else {
            g_endpoints.emplace_back(udp);
            auto endpoint = g_endpoints.back();
            this->add_fd(endpoint->fd, endpoint.get(), EPOLLIN);
        }
    }

    // Create TCP endpoints
    for (const auto &conf : config.tcp_configs) {
        auto tcp = std::make_shared<TcpEndpoint>(conf.name);

        if (!tcp->setup(conf)) { // handles reconnect and add_fd
            return false;        // only on fatal errors
        }

        g_endpoints.emplace_back(tcp);
    }

    // Link grouped endpoints together
    for (auto e : g_endpoints) {
        if (e->get_group_name().empty()) {
            continue;
        }

        for (auto other : g_endpoints) { // find other endpoints in group
            if (other != e && e->get_group_name() == e->get_group_name()) {
                e->link_group_member(other);
            }
        }
    }

    // Create TCP server
    if (config.tcp_port != 0u) {
        g_tcp_fd = tcp_open(config.tcp_port);
    }

    // Create Log endpoint
    auto conf = config.log_config;
    if (!conf.logs_dir.empty()) {
        switch (conf.mavlink_dialect) {
        case LogOptions::MavDialect::Ardupilotmega:
            this->_log_endpoint = std::make_shared<BinLog>(conf);
            break;

        case LogOptions::MavDialect::Common:
            this->_log_endpoint = std::make_shared<ULog>(conf);
            break;

        case LogOptions::MavDialect::Auto:
            this->_log_endpoint = std::make_shared<AutoLog>(conf);
            break;

            // no default case on purpose
        }
        this->_log_endpoint->mark_unfinished_logs();
        g_endpoints.emplace_back(this->_log_endpoint);

        if (conf.log_telemetry) {
            auto tlog_endpoint = std::make_shared<TLog>(conf);
            tlog_endpoint->mark_unfinished_logs();
            g_endpoints.emplace_back(tlog_endpoint);
        }
    }

    // Apply other options
    if (config.report_msg_statistics) {
        add_timeout(MSEC_PER_SEC, _print_statistics_timeout_cb, this);
    }

    if (config.dedup_period_ms > 0) {
        log_info("Message de-duplication enabled: %ld ms period", config.dedup_period_ms);
        _msg_dedup.set_dedup_period(config.dedup_period_ms);
    }

    return true;
}

void Mainloop::clear_endpoints()
{
    g_endpoints.clear();
}

int Mainloop::tcp_open(unsigned long tcp_port)
{
    int fd;
    struct sockaddr_in6 sockaddr = {};
    int val = 1;

    fd = socket(AF_INET6, SOCK_STREAM | SOCK_NONBLOCK, 0);
    if (fd == -1) {
        log_error("TCP Server: Could not create tcp socket (%m)");
        return -1;
    }

    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &val, sizeof(val));

    sockaddr.sin6_family = AF_INET6;
    sockaddr.sin6_port = htons(tcp_port);
    sockaddr.sin6_addr = in6addr_any;

    if (bind(fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) < 0) {
        log_error("TCP Server: Could not bind to tcp socket (%m)");
        close(fd);
        return -1;
    }

    if (listen(fd, SOMAXCONN) < 0) {
        log_error("TCP Server: Could not listen on tcp socket (%m)");
        close(fd);
        return -1;
    }

    add_fd(fd, &g_tcp_fd, EPOLLIN);

    log_info("Opened TCP Server [%d] [::]:%lu", fd, tcp_port);

    return fd;
}

Timeout *Mainloop::add_timeout(uint32_t timeout_msec, std::function<bool(void *)> cb,
                               const void *data)
{
    auto *t = new Timeout(cb, data);

    assert_or_return(t, nullptr);

    t->fd = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK);
    if (t->fd < 0) {
        log_error("Unable to create timerfd: %m");
        goto error;
    }

    mod_timeout(t, timeout_msec);

    if (add_fd(t->fd, t, EPOLLIN) < 0) {
        goto error;
    }

    t->next = _timeouts;
    _timeouts = t;

    return t;

error:
    delete t;
    return nullptr;
}

void Mainloop::del_timeout(Timeout *t)
{
    t->remove_me = true;
}

void Mainloop::mod_timeout(Timeout *t, uint32_t timeout_msec)
{
    struct itimerspec ts;

    ts.it_interval.tv_sec = timeout_msec / MSEC_PER_SEC;
    ts.it_interval.tv_nsec = (timeout_msec % MSEC_PER_SEC) * NSEC_PER_MSEC;
    ts.it_value.tv_sec = ts.it_interval.tv_sec;
    ts.it_value.tv_nsec = ts.it_interval.tv_nsec;

    timerfd_settime(t->fd, 0, &ts, nullptr);
}

void Mainloop::_del_timeouts()
{
    // Guarantee one valid Timeout on the beginning of the list
    while ((_timeouts != nullptr) && _timeouts->remove_me) {
        Timeout *next = _timeouts->next;
        remove_fd(_timeouts->fd);
        delete _timeouts;
        _timeouts = next;
    }

    // Remove all other Timeouts
    if (_timeouts != nullptr) {
        Timeout *prev = _timeouts;
        Timeout *current = _timeouts->next;
        while (current != nullptr) {
            if (current->remove_me) {
                prev->next = current->next;
                remove_fd(current->fd);
                delete current;
                current = prev->next;
            } else {
                prev = current;
                current = current->next;
            }
        }
    }
}
