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

int Mainloop::handle_modem_tx(const std::shared_ptr<Endpoint> &endpoint, const struct buffer *buf)
{
    // reader result.
    int r = 0;

    // Determine the current write mode based on our current endpoint.
    writeToPort = endpoint->get_name() == "port_modem";
    writeToStbd = endpoint->get_name() == "stbd_modem";

    // Handle actions based on current state
    switch (modemState) {
        case PORT_TX:
            // Confirm we are on our endpoint for message sending
            if(writeToPort) {
                r = endpoint->write_msg(buf);
                log_debug("PORT_TX: msg_id:%u, seq:%u, sysid:%u", buf->curr.msg_id, buf->curr.seq_id, buf->curr.src_sysid);
                prev_port_modem = true;
                prev_stbd_modem = false;
            }
            break;
        case STBD_TX:
            if(writeToStbd) {
                r = endpoint->write_msg(buf);
                log_debug("STBD_TX: msg_id:%u, seq:%u, sysid:%u", buf->curr.msg_id, buf->curr.seq_id, buf->curr.src_sysid);
                prev_port_modem = false;
                prev_stbd_modem = true;
            }
            break;
        case BOTH_TX:
        // Transmit the message from both modems
        if (writeToPort && prev_port_modem) {
            if (buf->curr.seq_id != lastSeqIdStbd) {
                r = endpoint->write_msg(buf);
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
                r = endpoint->write_msg(buf);
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
                r = endpoint->write_msg(buf);
                // Update the state to indicate that both modems are off
                prev_port_modem = false;
                log_debug("BOTH_OFF[%s]: msg_id:%u, seq:%u, sysid:%u", endpoint->get_name().c_str(),buf->curr.msg_id, buf->curr.seq_id, buf->curr.src_sysid);
            } else if (writeToStbd && prev_stbd_modem) {
                r = endpoint->write_msg(buf);
                // Update the state to indicate that both modems are off
                prev_stbd_modem = false;
                log_debug("BOTH_OFF[%s]: msg_id:%u, seq:%u, sysid:%u", endpoint->get_name().c_str(),buf->curr.msg_id, buf->curr.seq_id, buf->curr.src_sysid);
            }
            break;
    }

    return r;
}

int Mainloop::convert_gps_to_mavlink1(const std::shared_ptr<Endpoint>& endpoint, const struct buffer* buf)
{
    // Decode the incoming GPS_RAW_INT
    mavlink_message_t tmp_gps{};
    memmove(tmp_gps.payload64, buf->curr.payload, buf->curr.payload_len);
    tmp_gps.len = buf->curr.payload_len;

    mavlink_gps_raw_int_t gps1{};
    mavlink_msg_gps_raw_int_decode(&tmp_gps, &gps1);

    // GPS_FIX_TYPE: 0=No GPS, 1=No Fix, 2=2D Fix, 3=3D Fix, 4=DGPS, 5=RTK Float, 6=RTK Fixed
    if (gps1.fix_type < GPS_FIX_TYPE_2D_FIX) {
        log_trace(" <> Tracker GPS: no lock (GPS1 fix=%u sats=%u)",
                  gps1.fix_type, gps1.satellites_visible);
        return 0;
    }

    // Temporary set to MAVlink1
    mavlink_set_proto_version(MAVLINK_COMM_0, 1);

    // Initialize MAV1 message
    mavlink_message_t mav1_msg = {};
    // Buffer to store outgoing MAVLink1 message
    uint8_t data[MAVLINK_MAX_PACKET_LEN] = {};
    struct buffer out = {};

    // Encode the MAVLink 1 GPS_RAW_INT message
    mavlink_msg_gps_raw_int_encode(buf->curr.src_sysid, buf->curr.src_compid, &mav1_msg, &gps1);

    // Serialize the MAVLink 1 message into the send buffer
    out.len = mavlink_msg_to_send_buffer(data, &mav1_msg);
    out.data = data;

    // Write the message to the tracker endpoint
    const int r = endpoint->write_msg(&out);

    // Set back to MAVlink2
    mavlink_set_proto_version(MAVLINK_COMM_0, 2);

    return r;
}

int Mainloop::write_msg(const std::shared_ptr<Endpoint> &e, const struct buffer *buf)
{
    // reader result.
    int r = 0;

    // Confirm endpoint
    const auto &name = e->get_name();

    // Custom action for GCS Modems
    if (name == "port_modem" || name == "stbd_modem") {
        r = handle_modem_tx(e, buf);
    } else if (name == "tracker" && buf->curr.msg_id == MAVLINK_MSG_ID_GPS_RAW_INT) {
        // Intercept GPS_RAW_INT for Tracker Endpoint (Convert MAV2 -> MAV1)
        r = convert_gps_to_mavlink1(e, buf);
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

void Mainloop::set_modem_power(bool boost, bool only_mismatched)
{
    // AllowBoost = false: the modems keep the power they were configured with.
    // Diag reads (power, temperature, firmware, ...) still run.
    if (!_modem_power_writes) {
        // Info only for an actual request; the re-check on every STATION_CTRL is quiet.
        if (!only_mismatched) {
            log_info(" > Modem power writes disabled (AllowBoost = false): ignoring %s",
                     boost ? "BOOST" : "UNBOOST");
        }
        return;
    }

    // Boost always applies to every modem, never just one of them.
    for (const auto &modem : gcs_modems) {
        const int dbm = boost ? modem->boost_power_dbm() : modem->normal_power_dbm();

        if (only_mismatched
            && (modem->power_dbm() == ModemDiag::POWER_UNKNOWN || modem->power_dbm() == dbm)) {
            continue;
        }
        log_info(" > Modem %s: %s - %d dBm", modem->label().c_str(), boost ? "Enable " : "Disable", dbm);
        modem->set_power(static_cast<uint8_t>(dbm));
    }
}

bool Mainloop::modem_boost_confirmed() const
{
    // No diag endpoints configured: nothing to confirm against, report what was commanded.
    if (gcs_modems.empty()) {
        return modem_boost;
    }

    // Boosted only when every modem reports its boost power.
    for (const auto &modem : gcs_modems) {
        if (modem->power_dbm() != modem->boost_power_dbm()) {
            return false;
        }
    }
    return true;
}

void Mainloop::_on_modem_power_change()
{
    if (modem_boost_confirmed() == _reported_modem_boost) {
        return;
    }

    // The confirmed boost state changed since the GCS last heard it. Push an
    // unsolicited STATION_STATUS to the endpoint that last sent STATION_CTRL,
    // if it is still connected.
    for (const auto &e : g_endpoints) {
        if (e.get() == _station_ctrl_src) {
            send_station_status_msg(e.get());
            return;
        }
    }
}

void Mainloop::handle_station_ctrl_msg(const struct buffer *buf)
{
    // Extract a pointer to mavlink_station_ctrl_t from the payload data in buf
    const mavlink_station_ctrl_t *station_ctrl = (mavlink_station_ctrl_t *)buf->curr.payload;

    // Set the port_modem and stbd_modem flags based on the station control flags
    port_modem  = station_ctrl->flags & STATION_CTRL_FLAGS::TX_PORT_MODEM;
    stbd_modem  = station_ctrl->flags & STATION_CTRL_FLAGS::TX_STBD_MODEM;
    modem_boost = station_ctrl->flags & STATION_CTRL_FLAGS::MODEM_BOOST;

    // Command the modems when the requested boost state changes
    if(modem_boost != prev_modem_boost) {
        set_modem_power(modem_boost);
        // track state of the modem boost.
        prev_modem_boost = modem_boost;
    } else {
        // Same request as before, but a modem may report a different power, e.g.
        // it was configured to start at boost power. Re-command just those modems,
        // so the operator selecting UNBOOST (or BOOST) always takes effect. Modems
        // with unknown power (not replying) are left alone to avoid hammering them.
        set_modem_power(modem_boost, true);
    }

    // Update the state machine based on the current state of the modems
    if (port_modem && !stbd_modem) {
        modemState = PORT_TX;
    } else if (!port_modem && stbd_modem) {
        modemState = STBD_TX;
    } else if (port_modem && stbd_modem) {
        if (modemState != BOTH_TX) {
            messageCounter = 0;
            lastSeqIdPort = 0;
            lastSeqIdStbd = 0;
            // BOTH_TX only transmits while one of the prev_* flags is set. Coming
            // from BOTH_OFF both are cleared, which would stall transmission, so
            // seed the alternation starting on the port modem.
            prev_port_modem = true;
            prev_stbd_modem = false;
        }
        modemState = BOTH_TX;
    } else {
        modemState = BOTH_OFF;
    }

    return;
}

void Mainloop::send_station_status_msg(Endpoint *dst)
{
    // Confirm pointer
    if (dst == nullptr) {
        log_error("STATION_STATUS: no source endpoint to reply to, dropping");
        return;
    }

    // Identity we report STATION_STATUS from (sysid 1, comp 191). Named separately
    // because the old code passed a component-ID constant as the sysid.
    constexpr uint8_t STATION_SYSID  = 1;
    constexpr uint8_t STATION_COMPID = MAV_COMP_ID_ONBOARD_COMPUTER;

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
    // Tx Modem Boost status - GCS. Set only once every modem has confirmed the
    // boost power by read-back, not merely because it was commanded.
    _reported_modem_boost = modem_boost_confirmed();
    if (_reported_modem_boost) {
        status_flags |= STATION_STATUS_FLAGS_MODEM_BOOST;
    }
    // Set the flags
    station_status.flags = status_flags;

    // Encode the STATION_STATUS message
    mavlink_msg_station_status_encode(STATION_SYSID, STATION_COMPID, &mav_msg, &station_status);
    
    // Serialize the message
    out.len = mavlink_msg_to_send_buffer(data, &mav_msg);
    out.data = data;

    // Populate buffer metadata for routing
    out.curr.msg_id = MAVLINK_MSG_ID_STATION_STATUS;
    out.curr.src_sysid = STATION_SYSID;
    out.curr.src_compid = STATION_COMPID;
    out.curr.target_sysid = _station_ctrl_sysid;
    out.curr.target_compid = _station_ctrl_compid;
    out.curr.payload_len = mav_msg.len;
    out.curr.payload = reinterpret_cast<uint8_t*>(_MAV_PAYLOAD_NON_CONST(&mav_msg));

    // Write directly to the GCS endpoint. Bypasses g_endpoints,
    // write_msg's name-based dispatch, and handle_modem_tx entirely.
    dst->write_msg(&out);
}

void Mainloop::route_msg(struct buffer *buf, Endpoint *src_ep)
{
    bool unknown = true;

    // Special case for intercepting and handling GCS Station Modem Control Messages
    if (buf->curr.msg_id == MAVLINK_MSG_ID_STATION_CTRL) {
        // Remember who to send STATION_STATUS to, including unsolicited
        // updates once the modems confirm a power change.
        _station_ctrl_src = src_ep;
        _station_ctrl_sysid = buf->curr.src_sysid;
        _station_ctrl_compid = buf->curr.src_compid;

        // Intercept and handle the station control message
        handle_station_ctrl_msg(buf);
        send_station_status_msg(src_ep);
        return;
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
    const int max_events = 16;
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

        // Check if the UART is a GCS Modem tagged with "boost"
        if (conf.name.find("boost") != std::string::npos) {
            _add_modem_diag(uart, conf.modem_power);
        }
        // Proceed as before - Don't add endpoint for our modem UARTs
        else {
            g_endpoints.push_back(uart);
            auto endpoint = g_endpoints.back();
            this->add_fd(endpoint->fd, endpoint.get(), EPOLLIN);
        }
    }
    // Create UDP endpoints
    for (const auto &conf : config.udp_configs) {
        auto udp = std::make_shared<UdpEndpoint>(conf.name);

        if (!udp->setup(conf)) {
            return false;
        }

        // Check if the UDP is a GCS Modem tagged with "boost"
        if (conf.name.find("boost") != std::string::npos) {
            _add_modem_diag(udp, conf.modem_power);
        }
        // Proceed as before - Don't add endpoint for our modem UDPs
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
            // Compare against 'other': the old self-vs-self test was always true
            if (other != e && e->get_group_name() == other->get_group_name()) {
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

    _setup_modem_power();

    // Read each modem's current and after-reset output power, so the boost
    // state reported to the GCS starts from what the modems actually report,
    // and its identity (firmware, product, hardware revision). Then keep
    // re-reading the power and temperature so a modem that resets (falling back
    // to its after-reset power) is noticed.
    for (const auto &modem : gcs_modems) {
        modem->read_power();
        modem->read_info();
        modem->start_periodic_read(MODEM_POWER_RECHECK_SEC * MSEC_PER_SEC);
    }

    // Send initial modem boost state at startup
    if (modem_boost) {
        set_modem_power(true);
        prev_modem_boost = modem_boost;
    }

    return true;
}

void Mainloop::_setup_modem_power()
{
    // Boost applies to all modems or none, so power is only written when every
    // diag endpoint allows it.
    size_t allowed = 0;
    for (const auto &modem : gcs_modems) {
        if (modem->allow_boost()) {
            allowed++;
        }
    }
    _modem_power_writes = (allowed == gcs_modems.size());

    if (allowed != 0 && allowed != gcs_modems.size()) {
        log_warning("AllowBoost differs between modem diag endpoints; "
                    "boost applies to all modems or none, so power writes are disabled");
    }

    for (const auto &modem : gcs_modems) {
        modem->set_writes_enabled(_modem_power_writes);
        log_info("Modem %s: NormalPower %d dBm, BoostPower %d dBm, AllowBoost %s",
                 modem->label().c_str(),
                 modem->normal_power_dbm(),
                 modem->boost_power_dbm(),
                 modem->allow_boost() ? "true" : "false");
    }

    if (!gcs_modems.empty() && !_modem_power_writes) {
        log_info("Modem power writes disabled: modems keep their configured power");
    }
}

void Mainloop::_add_modem_diag(const std::shared_ptr<Endpoint> &endpoint,
                               const ModemPowerConfig &power_conf)
{
    // Diag endpoints only carry modem power commands: they are polled for the
    // modem's replies but never routed MAVLink (not added to g_endpoints).
    auto modem = std::make_shared<ModemDiag>(endpoint, power_conf);
    modem->on_power_change = [this]() { _on_modem_power_change(); };

    add_fd(modem->fd, modem.get(), EPOLLIN);
    gcs_modems.emplace_back(modem);
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
