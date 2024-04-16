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


# Serial connection.
master = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)

def main():
    
    try:
        while True:
            msg = master.recv_match(blocking=True)
            if not msg:
                continue
            print (msg.get_type())

    except KeyboardInterrupt:
        print("Keyboard interrupt")
    except Exception as e:
        print(e)
    finally:
        master.close()
        print("Connection closed")

if __name__ == "__main__":
    main()