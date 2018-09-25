#!/usr/bin/env python2
import time
import tellopy


def test():
    drone1 = tellopy.Tello(
        local_cmd_client_port=8890,
        local_vid_server_port=6038,
        tello_ip='192.168.10.1',
    )
    print('Ctor initialized')
    try:
        drone1.connect()
        print('Wait for connection')
        drone1.wait_for_connection(10.0)
        print('Connected to drone')
        # drone1.start_video()
        while True:
            time.sleep(1.0)
    except BaseException as ex:
        print(ex)
    finally:
        if False:
            drone1.reset_cmd_vel()
            drone1.land()
        drone1.quit()


if __name__ == '__main__':
    test()
