#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
import time
import threading

# Publisher for sending acceleration commands to flappy bird
pub_acc_cmd = rospy.Publisher('/flappy_acc', Vector3, queue_size=1)


class InfoGetter(object):
    def __init__(self):
        self.current_vel = None
        self.current_laserscan = None

    def velCallback(self, msg):
        self.current_vel = msg

    def laserScanCallback(self, msg):
        self.current_laserscan = msg


def initNode():
    info_getter = InfoGetter()

    # Here we initialize our node running the automation code
    rospy.init_node('flappy_automation_code', anonymous=True)

    # Subscribe to topics for velocity and laser scan from Flappy Bird game
    bind_laserscan_to_info_getter(info_getter)
    bind_velocity_to_info_getter(info_getter)

    # Ros spin to prevent program from exiting
    rospy.spin()


def bind_velocity_to_info_getter(info_getter):
    rospy.Subscriber("/flappy_vel", Vector3, info_getter.velCallback)


def bind_laserscan_to_info_getter(info_getter):
    rospy.Subscriber("/flappy_laser_scan", LaserScan, info_getter.laserScanCallback)


def automate():
    x = 0
    y = 0
    time.sleep(10)
    pub_acc_cmd.publish(Vector3(1., 0., 0))
    if msg.ranges[3] < 1. or msg.ranges[4] < 1.:
        pub_acc_cmd.publish(Vector3(x, y., 0))



if __name__ == '__main__':
    try:
        initNode()
    except rospy.ROSInterruptException:
        pass
