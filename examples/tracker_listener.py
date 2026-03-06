#!/usr/bin/python3
from pymavlink import mavutil

conn = mavutil.mavlink_connection('udpin:0.0.0.0:8002', dialect='ardupilotmega')
print("Listening on UDP :8002 for tracker GPS messages...\n")

while True:
    msg = conn.recv_match(blocking=True, timeout=5.0)
    if msg is None:
        print("[timeout] No message received")
        continue

    t = msg.get_type()
    if t == 'GPS_RAW_INT':
        fix_names = {0:'NO_GPS', 1:'NO_FIX', 2:'2D', 3:'3D', 4:'DGPS', 5:'RTK_FLOAT', 6:'RTK_FIXED'}
        fix_str = fix_names.get(msg.fix_type, str(msg.fix_type))
        print(f"GPS_RAW_INT | fix={fix_str}({msg.fix_type}) sats={msg.satellites_visible:2d} "
              f"lat={msg.lat/1e7:.6f} lon={msg.lon/1e7:.6f} alt={msg.alt/1000:.1f}m | "
              f"sysid={msg.get_srcSystem()} compid={msg.get_srcComponent()}")
    elif t == 'BAD_DATA':
        print(f"[BAD_DATA] len={msg.length}")
    else:
        print(f"[{t}] {msg}")
