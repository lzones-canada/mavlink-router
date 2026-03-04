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
#from lzc import *
#from datetime import datetime

# Serial connection.
#master = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
master = mavutil.mavlink_connection('udp:0.0.0.0:14550')
#master = mavutil.mavlink_connection('udpout:192.168.2.80:8001', dialect='common')

def main():
    
    try:
        while True:
            msg = master.recv_match(blocking=True)
            if not msg:
                continue
            print(msg.get_type())
            # Get the current timestamp with milliseconds
            #now = datetime.now()
            #timestamp = now.strftime('%H:%M:%S.%f')[:-3]  # Truncate to milliseconds
            # Print the message type with the timestamp
            #print(f"{timestamp} - {msg.get_type()}")

    except KeyboardInterrupt:
        print("Keyboard interrupt")
    except Exception as e:
        print(e)
    finally:
        master.close()
        print("Connection closed")

if __name__ == "__main__":
    main()















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

# import sys
# import time
# from pymavlink import mavutil
# from lzc import *
# #from datetime import datetime

# # Serial connection.
# master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)

# def main():
    
#     try:
#         while True:
#             #msg = master.recv_match(blocking=True)
#             # Wait for the GPS_RAW_INT or GLOBAL_POSITION_INT message
#             msg = master.recv_match(type=['GPS_RAW_INT', 'GLOBAL_POSITION_INT'], blocking=True)
#             if not msg:
#                 continue
#             if msg:
#                 if msg.get_type() == 'GPS_RAW_INT':
#                     # GPS_RAW_INT has lat, lon in 1E7 (integer degrees * 1e7)
#                     latitude = msg.lat / 1e7
#                     longitude = msg.lon / 1e7
#                 elif msg.get_type() == 'GLOBAL_POSITION_INT':
#                     # GLOBAL_POSITION_INT has lat, lon in 1E7 (integer degrees * 1e7)
#                     latitude = msg.lat / 1e7
#                     longitude = msg.lon / 1e7
#                 #print(msg.get_type())
#                 #print(f"{msg.to_json()}\n")
#                 print(f"Latitude: {latitude}, Longitude: {longitude}")
#             # Get the current timestamp with milliseconds
#             #now = datetime.now()
#             #timestamp = now.strftime('%H:%M:%S.%f')[:-3]  # Truncate to milliseconds
#             # Print the message type with the timestamp
#             #print(f"{timestamp} - {msg.get_type()}")

#     except KeyboardInterrupt:
#         print("Keyboard interrupt")
#     except Exception as e:
#         print(e)
#     finally:
#         master.close()
#         print("Connection closed")

# if __name__ == "__main__":
#     main()