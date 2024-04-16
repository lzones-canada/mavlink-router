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

import sys
import time
from pymavlink import mavutil
from lzc import *

# Define the connection parameters
device = 'udpout:' + sys.argv[1]
source_system = 1
source_component = 1

# Heartbeat message parameters
vehicle_type = mavutil.mavlink.MAV_TYPE_FIXED_WING  # Example vehicle type (MAV_TYPE)
autopilot_type = mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA  # Example autopilot type (MAV_AUTOPILOT)
base_mode = 209  # Example base mode
custom_mode = 0  # Example custom mode
system_status = mavutil.mavlink.MAV_STATE_STANDBY  # Example system status
mavlink_version = 3  # Example MAVLink version

#uavionix_adsb_out_status message parameters
state = 3  # Example state value
squawk = 1200  # Example squawk value
NIC_NACp = 4  # Example NIC_NACp value
boardTemp = 25  # Example board temperature value
fault = 0  # Example fault value
flight_id = b'ABC12345'  # Example flight ID value

# Establish the MAVLink connection
master = mavutil.mavlink_connection(device, source_system=source_system, source_component=source_component)

while True:
    try:
        hbeatMsg = MAVLink_heartbeat_message(
            type=vehicle_type,
            autopilot=autopilot_type,
            base_mode=base_mode,
            custom_mode=custom_mode,
            system_status=system_status,
            mavlink_version=mavlink_version
        )
            
        adsbStatusMsg = MAVLink_uavionix_adsb_out_status_message(
                    state= 0,
                    squawk= 0,
                    NIC_NACp=NIC_NACp,
                    boardTemp=0,
                    fault=0,
                    flight_id=b"0",
                )

        master.mav.send(hbeatMsg)
        time.sleep(0.5)
        master.mav.send(adsbStatusMsg)
        print(".", end="")
        sys.stdout.flush()
        time.sleep(0.5)

    except KeyboardInterrupt:
            print("Exiting..")
            sys.exit(0)
