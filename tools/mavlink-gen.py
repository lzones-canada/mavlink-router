#!/usr/bin/env python3
# Generate the MAVLink C headers for the ardupilotmega dialect from the
# mavlink submodule, and print the message definition files it was built
# from (one per line) so meson can re-run setup when any of them change.
#
# usage: mavlink-gen.py <mavlink-dir> <output-dir>

import contextlib
import os
import re
import sys

mavlink_dir, out_dir = sys.argv[1], sys.argv[2]
defs_dir = os.path.join(mavlink_dir, 'message_definitions', 'v1.0')
dialect = 'ardupilotmega.xml'

sys.path.insert(0, mavlink_dir)
from pymavlink.generator import mavgen  # noqa: E402

opts = mavgen.Opts(out_dir, wire_protocol='2.0', language='C', validate=False)
with contextlib.redirect_stdout(sys.stderr):
    if not mavgen.mavgen(opts, [os.path.join(defs_dir, dialect)]):
        sys.exit('mavgen failed')

xmls, todo = [], [dialect]
while todo:
    xml = todo.pop()
    if xml in xmls:
        continue
    xmls.append(xml)
    with open(os.path.join(defs_dir, xml)) as f:
        todo += re.findall(r'<include>\s*(\S+?)\s*</include>', f.read())
print('\n'.join(sorted(xmls)))
