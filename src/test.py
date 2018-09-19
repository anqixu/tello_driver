#!/usr/bin/env python2

import tello

mDrone = tello.Tello(tello_ip='172.17.0.2')
while True:
    cmd = raw_input('> ').lower()
    if len(cmd) <= 0:
        continue
    if cmd[0] in ('x', 'q'):
        print('Exiting')
        break
    elif cmd.find('takeoff') >= 0:
        print('Taking off')
        mDrone.takeOff()
    elif cmd.find('land') >= 0:
        print('Landing')
        mDrone.land()
mDrone.stop()
