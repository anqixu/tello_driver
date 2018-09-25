#!/usr/bin/env python2
from time import sleep
import tellopy


def test():
    drone1 = tellopy.Tello(
        local_cmd_client_port=8890,
        local_vid_server_port=6038,
        tello_ip='172.17.0.2',
    )
    drone2 = tellopy.Tello(
        local_cmd_client_port=9890,
        local_vid_server_port=7038,
        tello_ip='172.17.0.3',
    )
    try:
        drone1.connect()
        drone2.connect()
        drone1.wait_for_connection(10.0)
        drone2.wait_for_connection(10.0)
        print('Connected to both drones')
        sleep(2.0)

        drone1.takeoff()
        drone2.takeoff()
        sleep(3.5)

        drone1.set_vspeed(1.0)
        drone2.set_vspeed(1.0)
        sleep(0.8)
        drone1.set_vspeed(0.0)
        drone2.set_vspeed(0.0)

        sleep(7.0)

        drone1.set_pitch(0.5)
        drone1.set_yaw(1.0)
        drone2.set_pitch(-0.5)
        drone2.set_yaw(1.0)
        sleep(2.0)
        drone1.set_pitch(0.0)
        drone1.set_yaw(0.0)
        drone2.set_pitch(0.0)
        drone2.set_yaw(0.0)

        sleep(5.0)

        drone1.set_vspeed(-0.5)
        drone2.set_vspeed(-0.5)
        sleep(0.6)
        drone1.set_vspeed(0.0)
        drone2.set_vspeed(0.0)

        sleep(1.0)

        drone1.palm_land()
        drone2.palm_land()
        sleep(3.0)
    except BaseException as ex:
        print(ex)
    finally:
        drone1.set_roll(0.0)
        drone1.set_pitch(0.0)
        drone1.set_yaw(0.0)
        drone1.set_vspeed(0.0)
        drone2.set_roll(0.0)
        drone2.set_pitch(0.0)
        drone2.set_yaw(0.0)
        drone2.set_vspeed(0.0)
        drone1.land()
        drone2.land()
        sleep(5.0)
        drone1.quit()
        drone2.quit()


if __name__ == '__main__':
    test()
