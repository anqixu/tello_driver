#!/usr/bin/env python2
import rospy
from std_msgs.msg import Empty, UInt8, Bool
from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from dynamic_reconfigure.server import Server
from tello_driver.msg import TelloStatus
from tello_driver.cfg import TelloConfig
from cv_bridge import CvBridge, CvBridgeError

import av
import math
import numpy as np
import threading
from tellopy._internal import tello
from tellopy._internal import error
from tellopy._internal import protocol
from tellopy._internal import logger


class RospyLogger(logger.Logger):
    def __init__(self, header=''):
        super(RospyLogger, self).__init__(header)

    def error(self, s):
        if self.log_level < logger.LOG_ERROR:
            return
        rospy.logerr(s)

    def warn(self, s):
        if self.log_level < logger.LOG_WARN:
            return
        rospy.logwarn(s)

    def info(self, s):
        if self.log_level < logger.LOG_INFO:
            return
        rospy.loginfo(s)

    def debug(self, s):
        if self.log_level < logger.LOG_DEBUG:
            return
        rospy.logdebug(s)


def notify_cmd_success(cmd, success):
    if success:
        rospy.loginfo('%s command executed' % cmd)
    else:
        rospy.logwarn('%s command failed' % cmd)


class TelloNode(tello.Tello):
    def __init__(self):
        self.tello_ip = rospy.get_param('~tello_ip', '192.168.10.1')
        self.tello_cmd_port = int(rospy.get_param('~tello_cmd_port', 8889))
        self.connect_timeout_sec = float(
            rospy.get_param('~connect_timeout_sec', 10.0))
        self.bridge = CvBridge()

        # Connect to drone
        log = RospyLogger('Tello')
        log.set_level(self.LOG_WARN)
        super(TelloNode, self).__init__(
            tello_ip=self.tello_ip,
            tello_cmd_port=self.tello_cmd_port,
            log=log)
        rospy.loginfo('Connecting to drone @ %s:%d' % self.tello_addr)
        self.connect()
        try:
            self.wait_for_connection(timeout=self.connect_timeout_sec)
        except error.TelloError as err:
            rospy.logerr(str(err))
            rospy.signal_shutdown(str(err))
            self.quit()
            return
        rospy.loginfo('Connected to drone')
        rospy.on_shutdown(self.cb_shutdown)

        # Setup dynamic reconfigure
        self.cfg = None
        self.srv_dyncfg = Server(TelloConfig, self.cb_dyncfg)

        # Setup topics and services
        # NOTE: ROS interface deliberately made to resemble bebop_autonomy
        self.pub_status = rospy.Publisher(
            'status', TelloStatus, queue_size=1, latch=True)
        self.pub_image_raw = rospy.Publisher('image_raw', Image, queue_size=10)

        self.sub_takeoff = rospy.Subscriber('takeoff', Empty, self.cb_takeoff)
        self.sub_throw_takeoff = rospy.Subscriber(
            'throw_takeoff', Empty, self.cb_throw_takeoff)
        self.sub_land = rospy.Subscriber('land', Empty, self.cb_land)
        self.sub_palm_land = rospy.Subscriber(
            'palm_land', Empty, self.cb_palm_land)
        self.sub_flattrim = rospy.Subscriber(
            'flattrim', Empty, self.cb_flattrim)
        self.sub_flip = rospy.Subscriber('flip', UInt8, self.cb_flip)
        self.sub_cmd_vel = rospy.Subscriber('cmd_vel', Twist, self.cb_cmd_vel)

        self.subscribe(self.EVENT_FLIGHT_DATA, self.cb_status_telem)
        self.frame_thread = threading.Thread(target=self.framegrabber_loop)
        self.frame_thread.start()

        # NOTE: odometry from parsing logs might be possible eventually,
        #       but it is unclear from tests what's being sent by Tello
        # - https://github.com/Kragrathea/TelloLib/blob/master/TelloLib/Tello.cs#L1047
        # - https://github.com/Kragrathea/TelloLib/blob/master/TelloLib/parsedRecSpecs.json
        # self.pub_odom = rospy.Publisher(
        #    'odom', UInt8MultiArray, queue_size=1, latch=True)
        # self.pub_odom = rospy.Publisher(
        #    'odom', Odometry, queue_size=1, latch=True)
        #self.subscribe(self.EVENT_LOG, self.cb_odom_log)

        rospy.loginfo('Tello driver node ready')

    def cb_shutdown(self):
        self.quit()
        self.frame_thread.join()

    def cb_status_telem(self, event, sender, data, **args):
        speed_horizontal_mps = math.sqrt(
            data.north_speed*data.north_speed+data.east_speed*data.east_speed)/10.

        # NOTE: Anecdotally, observed that:
        # data.east_speed points to South
        # data.north_speed points to East
        msg = TelloStatus(
            height_m=data.height/10.,
            speed_northing_mps=-data.east_speed/10.,
            speed_easting_mps=data.north_speed/10.,
            speed_horizontal_mps=speed_horizontal_mps,
            speed_vertical_mps=-data.vertical_speed/10.,
            flight_time_sec=data.fly_time/10.,
            imu_state=data.imu_state,
            pressure_state=data.pressure_state,
            down_visual_state=data.down_visual_state,
            power_state=data.power_state,
            battery_state=data.battery_state,
            gravity_state=data.gravity_state,
            wind_state=data.wind_state,
            imu_calibration_state=data.imu_calibration_state,
            battery_percentage=data.battery_percentage,
            drone_fly_time_left_sec=data.drone_fly_time_left/10.,
            drone_battery_left_sec=data.drone_battery_left/10.,
            is_flying=data.em_sky,
            is_on_ground=data.em_ground,
            is_em_open=data.em_open,
            is_drone_hover=data.drone_hover,
            is_outage_recording=data.outage_recording,
            is_battery_low=data.battery_low,
            is_battery_lower=data.battery_lower,
            is_factory_mode=data.factory_mode,
            fly_mode=data.fly_mode,
            throw_takeoff_timer_sec=data.throw_fly_timer/10.,
            camera_state=data.camera_state,
            electrical_machinery_state=data.electrical_machinery_state,
            front_in=data.front_in,
            front_out=data.front_out,
            front_lsc=data.front_lsc,
            temperature_height_m=data.temperature_height/10.,
        )
        self.pub_status.publish(msg)

    def cb_odom_log(self, event, sender, data, **args):
        odom_msg = UInt8MultiArray()
        odom_msg.data = str(data)
        self.pub_odom.publish(odom_msg)

    def framegrabber_loop(self):
        vs = self.get_video_stream()
        try:
            container = av.open(vs)
        except BaseException as err:
            rospy.logerr(str(err))
            return
        for frame in container.decode(video=0):  # vs blocks, dies on self.stop
            img = np.array(frame.to_image())
            try:
                msg = self.bridge.cv2_to_imgmsg(img, 'rgb8')
            except CvBridgeError as e:
                rospy.logerr(str(e))
                continue
            self.pub_image_raw.publish(msg)

    def cb_dyncfg(self, config, level):
        update_all = False
        if self.cfg is None:
            self.cfg = config
            update_all = True
        self.cfg = config
        return self.cfg

    def cb_takeoff(self, msg):
        success = self.takeoff()
        notify_cmd_success('Takeoff', success)

    def cb_throw_takeoff(self, msg):
        success = self.throw_takeoff()
        if success:
            rospy.loginfo('Drone set to auto-takeoff when thrown')
        else:
            rospy.logwarn('ThrowTakeoff command failed')

    def cb_land(self, msg):
        success = self.land()
        notify_cmd_success('Land', success)

    def cb_palm_land(self, msg):
        success = self.palm_land()
        notify_cmd_success('PalmLand', success)

    def cb_flattrim(self, msg):
        success = self.flattrim()
        notify_cmd_success('FlatTrim', success)

    def cb_flip(self, msg):
        if msg.data < 0 or msg.data > protocol.FLIP_MAX_INT:
            rospy.logwarn('Invalid flip direction: %d' % msg.data)
            return
        success = self.flip(msg.data)
        notify_cmd_success('Flip %d' % msg.data, success)

    # WARNING: need to constantly send gamepad packets, or else props will stop even during takeoff
    def cb_cmd_vel(self, msg):
        self.set_pitch(msg.linear.x)
        self.set_roll(-msg.linear.y)
        self.set_yaw(-msg.angular.z)
        self.set_vspeed(msg.linear.z)
        # rospy.loginfo('> %3.1f %3.1f %3.1f %3.1f' %
        #              (msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)) # TODO: remove debug


def main():
    rospy.init_node('tello_node')
    robot = TelloNode()
    if robot.state != robot.STATE_QUIT:
        rospy.spin()


if __name__ == '__main__':
    main()
