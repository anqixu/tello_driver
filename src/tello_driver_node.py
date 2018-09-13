#!/usr/bin/env python2
import rospy
from std_msgs.msg import Empty, UInt8, Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from dynamic_reconfigure.server import Server
from tello_driver.msg import TelloStatus
from tello_driver.cfg import TelloConfig

import tello


def notify_cmd_success(cmd, success):
    if success:
        rospy.loginfo('%s command executed' % cmd)
    else:
        rospy.logwarn('%s command failed' % cmd)


class TelloNode(tello.Tello):
    def __init__(self):
        self.tello_ip = rospy.get_param('~tello_ip', '192.168.10.1')
        self.tello_cmd_port = int(rospy.get_param('~tello_cmd_port', 8889))

        # Connect to drone
        super(TelloNode, self).__init__(self.tello_ip, self.tello_cmd_port)
        rospy.loginfo('Connected to drone')
        rospy.on_shutdown(self.cb_shutdown)

        # Setup dynamic reconfigure
        self.cfg = None
        self.srv_dyncfg = Server(TelloConfig, self.cb_dyncfg)

        # Setup topics and services
        # NOTE: ROS interface deliberately made to resemble bebop_autonomy
        self.pub_status = rospy.Publisher(
            'status', TelloStatus, queue_size=1, latch=True)
        self.pub_odom = rospy.Publisher(
            'odom', Odometry, queue_size=1, latch=True)

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

        rospy.loginfo('Tello driver node ready')

    def cb_shutdown(self):
        self.stop()

    def process_sensor_update(self, d):
        msg = TelloStatus(
            height_m=d['Height'],
            speed_front_mps=d['NorthSpeed'],
            speed_lateral_mps=d['EastSpeed'],
            speed_planar_mps=d['HorizontalSpeed'],
            speed_vertical_mps=d['VerticalSpeed'],
            flight_time_sec=d['FlyTime'],
            imu_state=d['IMUState'],
            pressure_state=d['PressureState'],
            down_visual_state=d['DownVisualState'],
            power_state=d['PowerState'],
            battery_state=d['BatteryState'],
            gravity_state=d['GravityState'],
            wind_state=d['WindState'],
            imu_calibration_state=d['IMUCalibrationState'],
            battery_percentage=d['BatteryPercentage'],
            drone_fly_time_left_sec=d['DroneFlyTimeLeft'],
            drone_battery_left_sec=d['DroneBatteryLeft'],
            is_flying=d['IsFlying'],
            is_on_ground=d['IsOnGround'],
            is_em_open=d['IsEmOpen'],
            is_drone_hover=d['IsDroneHover'],
            is_outage_recording=d['IsOutageRecording'],
            is_battery_low=d['IsBatteryLow'],
            is_battery_lower=d['IsBatteryLower'],
            is_factory_mode=d['IsFactoryMode'],
            fly_mode=d['FlyMode'],
            throw_takeoff_timer_sec=d['ThrowFlyTimer'],
            camera_state=d['CameraState'],
            electrical_machinery_state=d['ElectricalMachineryState'],
            front_in=d['FrontIn'],
            front_out=d['FrontOut'],
            front_lsc=d['FrontLSC'],
            temperature_height_m=d['TemperatureHeight'],
        )
        self.pub_status.publish(msg)

    def todo_cb_odom(self, msg):
        odom_msg = Odometry()
        odom_msg.child_frame_id = 'Tello'
        odom_msg.pose.pose.position.x = sensor_obj.pos_x
        odom_msg.pose.pose.position.y = sensor_obj.pos_y
        odom_msg.pose.pose.position.z = sensor_obj.pos_z
        odom_msg.pose.pose.orientation.w = sensor_obj.quaternion_w
        odom_msg.pose.pose.orientation.x = sensor_obj.quaternion_x
        odom_msg.pose.pose.orientation.y = sensor_obj.quaternion_y
        odom_msg.pose.pose.orientation.z = sensor_obj.quaternion_z
        odom_msg.twist.twist.linear.x = sensor_obj.speed_x
        odom_msg.twist.twist.linear.y = sensor_obj.speed_y
        odom_msg.twist.twist.linear.z = sensor_obj.speed_z

        self.pub_odom.publish(odom_msg)

    def cb_dyncfg(self, config, level):
        update_all = False
        if self.cfg is None:
            self.cfg = config
            update_all = True
        self.cfg = config
        return self.cfg

    def cb_takeoff(self, msg):
        success = self.takeOff()
        notify_cmd_success('Takeoff', success)

    def cb_throw_takeoff(self, msg):
        success = self.throwTakeoff()
        if success:
            rospy.loginfo('Drone set to auto-takeoff when thrown')
        else:
            rospy.logwarn('ThrowTakeoff command failed')

    def cb_land(self, msg):
        success = self.land()
        notify_cmd_success('Land', success)

    def cb_palm_land(self, msg):
        success = self.palmLand()
        notify_cmd_success('PalmLand', success)

    def cb_flattrim(self, msg):
        success = self.flattrim()
        notify_cmd_success('FlatTrim', success)

    def cb_flip(self, msg):
        if msg.data < 0 or msg.data > 7:
            rospy.logwarn('Invalid flip direction: %d' % msg.data)
            return
        success = self.flip(msg.data)
        notify_cmd_success('Flip', success)

    # WARNING: need to constantly send gamepad packets, or else props will stop even during takeoff
    def cb_cmd_vel(self, msg):
        pitch = msg.linear.x
        roll = msg.linear.y
        yaw = msg.angular.z
        vertical_movement = msg.linear.z
        self.setCmdVel(roll, pitch, yaw, vertical_movement, self.cfg.fast_mode)


def main():
    rospy.init_node('tello_node')
    robot = TelloNode()
    rospy.spin()


if __name__ == '__main__':
    main()
