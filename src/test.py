#!/usr/bin/env python2

import tello

mDrone = tello.Tello()
while True:
    cmd = raw_input('>').lower()
    if len(cmd) <= 0:
        continue
    if cmd[0] == 'x' or 'q':
        break
    elif cmd.find('takeoff') >= 0:
        mDrone.takeOff()
    elif cmd.find('land') >= 0:
        mDrone.land()
mDrone.stop()
