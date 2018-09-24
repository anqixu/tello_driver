#!/usr/bin/env python

# Script to parse output from `iw dev`, and returns the physical device
# listed above an user-specified wifi network device

import os
import sys

if len(sys.argv) < 2:
    sys.stderr.write('Usage: %s <WIFI_DEV>\n' % sys.argv[0])
    sys.exit(1)

wifi_dev = sys.argv[1]
phyid = None
for line in os.popen('iw dev').readlines():
    if len(line) <= 0:
        continue
    elif line.find('phy#') == 0:
        phyid = 'phy'+line.strip()[4:]
    elif line.find(wifi_dev) > 0:
        print(phyid)
        sys.exit(0)
print('')
sys.exit(1)
