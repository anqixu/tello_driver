#!/usr/bin/env python2
from time import sleep
import tellopy


def test():
    drone1 = tellopy.Tello(
        local_cmd_client_port=8890,
        local_vid_server_port=6038,
        tello_ip='172.17.0.2',
    )
    # tello_ip='192.168.10.1',
    try:
        drone1.connect()
        drone1.wait_for_connection(10.0)
        print('Connected to drone')
        sleep(2.0)

        drone1.takeoff()
        sleep(3.5)

        drone1.set_vspeed(1.0)
        sleep(0.4)
        drone1.set_vspeed(0.0)

        drone1.palm_land()
        sleep(2.0)
    except BaseException as ex:
        print(ex)
    finally:
        drone1.set_roll(0.0)
        drone1.set_pitch(0.0)
        drone1.set_yaw(0.0)
        drone1.set_vspeed(0.0)
        drone1.land()
        sleep(5.0)
        drone1.quit()


if __name__ == '__main__':
    test()
