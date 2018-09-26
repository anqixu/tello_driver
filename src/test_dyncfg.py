#!/usr/bin/env python2

import rospy
from dynamic_reconfigure.server import Server
from tello_driver.cfg import TelloConfig
import pprint
import traceback


def cb_dyncfg(config, level):
    rospy.loginfo('Received cfg')
    pprint.pprint(config)
    return config


def main():
    rospy.init_node('dyncfg_test')
    srv_dyncfg = Server(TelloConfig, cb_dyncfg)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except BaseException:
        traceback.print_exc()
