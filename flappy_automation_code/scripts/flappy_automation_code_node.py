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

    automate(info_getter)

    # Ros spin to prevent program from exiting
    rospy.spin()


def bind_velocity_to_info_getter(info_getter):
    rospy.Subscriber("/flappy_vel", Vector3, info_getter.velCallback)


def bind_laserscan_to_info_getter(info_getter):
    rospy.Subscriber("/flappy_laser_scan", LaserScan, info_getter.laserScanCallback)


def automate(info_getter):
    old_ranges = (0., 0., 0., 0., 0., 0., 0., 0., 0.)
    idx = 0

    while True:
        five_congruous = (idx % 5 == 0)

        if info_getter.current_vel is not None and info_getter.current_laserscan is not None:
            current_vel = info_getter.current_vel
            current_ranges = info_getter.current_laserscan.ranges
            current_min_angles = info_getter.current_laserscan.angle_min
            current_time = info_getter.current_laserscan.header.stamp

            # if current_ranges[3] < 1. or current_ranges[6] < 1.:
            #     accelerate(-3., 0.)
            #
            #     if current_ranges[2] > current_ranges[5]:
            #         accelerate(-3., 0.)
            #         accelerate(-3., 0.1)
            #     else:
            #         accelerate(-3., 0.)
            #         accelerate(-3., -0.1)

            if current_vel.x == 0 and current_vel.y == 0:
                indice = np.argmax(current_ranges)
                if indice > 4:
                    accelerate(0., -0.1)
                elif indice < 4:
                    accelerate(0., 0.1)

            if current_ranges[4] >= 3.5:
                accelerate(0.1, 0.)

            if current_vel.x >= 0.5:
                accelerate(-0.3, 0.)

            # if five_congruous:
            #     old_ranges = current_ranges
            #     ranges_diff = compute_ranges_diff(current_ranges, old_ranges)
            #     if np.any(ranges_diff):
            #         print(ranges_diff)


def compute_ranges_diff(ranges_t_1, ranges_t):
    ranges_diff = np.subtract(ranges_t_1, ranges_t)
    return ranges_diff


def accelerate(x, y):
    pub_acc_cmd.publish(Vector3(x, y, 0))


if __name__ == '__main__':
    try:
        initNode()
    except rospy.ROSInterruptException:
        pass
