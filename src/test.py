#!/usr/bin/env python2
import time
import traceback
import tellopy


def test():
    drone1 = tellopy.Tello(
        local_cmd_client_port=8890,
        local_vid_server_port=6038,
        tello_ip='192.168.10.1',
    )
    try:
        drone1.connect()
        drone1.wait_for_connection(10.0)
        print('Connected to drone')

        # Test raw video
        if True:
            drone1.start_video()
            while True:
                time.sleep(1.0)

        # Test video decoding
        if False:
            import av
            import cv2
            import numpy

            container = av.open(drone1.get_video_stream())
            for frame in container.decode(video=0):
                image = cv2.cvtColor(numpy.array(
                    frame.to_image()), cv2.COLOR_RGB2BGR)
                cv2.imshow('Frame', image)
                key = cv2.waitKey(1)
                if key and chr(key) in ('q', 'Q', 'x', 'X'):
                    break

        # Test flying
        if False:
            time.sleep(2.0)
            drone1.takeoff()
            time.sleep(3.5)

            drone1.set_vspeed(1.0)
            time.sleep(0.6)
            drone1.set_vspeed(0.0)

            time.sleep(2.0)

            drone1.set_pitch(0.4)
            drone1.set_yaw(1.0)
            time.sleep(0.5)
            drone1.set_pitch(0.0)
            drone1.set_yaw(0.0)

            time.sleep(2.0)

            drone1.flip(1)

            time.sleep(2.0)

            drone1.set_vspeed(-0.5)
            time.sleep(0.6)
            drone1.set_vspeed(0.0)

            time.sleep(1.0)

            drone1.palm_land()
            time.sleep(3.0)

    except BaseException:
        traceback.print_exc()
    finally:
        if False:
            drone1.reset_cmd_vel()
            drone1.land()
        drone1.quit()


if __name__ == '__main__':
    test()
