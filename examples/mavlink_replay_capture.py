#!/usr/bin/python

# This file is part of the MAVLink Router project
#
# Copyright (C) 2021  Lucas De Marchi <lucas.de.marchi@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time
import json
from pymavlink import mavutil
from lzc import *

log_file_path = "mavlink_messages_tracker.log"
master = mavutil.mavlink_connection('/dev/ttyUSB1', baud=57600)

def replay_mavlink_messages():
    try:
        with open(log_file_path, 'r') as log_file:
            for line in log_file:
                message = json.loads(line.strip())
                msg_type = message["mavpackettype"]

                if msg_type == "HEARTBEAT":
                    mav_msg = MAVLink_heartbeat_message(
                        message["type"],
                        message["autopilot"],
                        message["base_mode"],
                        message["custom_mode"],
                        message["system_status"],
                        message["mavlink_version"]
                    )
                elif msg_type == "GLOBAL_POSITION_INT":
                    mav_msg = MAVLink_global_position_int_message(
                        message["time_boot_ms"],
                        message["lat"],
                        message["lon"],
                        message["alt"],
                        message["relative_alt"],
                        message["vx"],
                        message["vy"],
                        message["vz"],
                        message["hdg"]
                    )
                elif msg_type == "GPS_RAW_INT":
                    mav_msg = MAVLink_gps_raw_int_message(
                        message["time_usec"],
                        message["fix_type"],
                        message["lat"],
                        message["lon"],
                        message["alt"],
                        message["eph"],
                        message["epv"],
                        message["vel"],
                        message["cog"],
                        message["satellites_visible"],
                        message["alt_ellipsoid"],
                        message["h_acc"],
                        message["v_acc"],
                        message["vel_acc"],
                        message["hdg_acc"],
                        message["yaw"]
                    )
                else:
                    continue  # Skip unknown message types

                master.mav.send(mav_msg)
                print(f"Sent: {msg_type}")
                sys.stdout.flush()
                time.sleep(0.1)  # Simulate the delay between messages

    except KeyboardInterrupt:
        print("Keyboard interrupt")
    except Exception as e:
        print(e)
    finally:
        master.close()
        print("Connection closed")

if __name__ == "__main__":
    replay_mavlink_messages()
