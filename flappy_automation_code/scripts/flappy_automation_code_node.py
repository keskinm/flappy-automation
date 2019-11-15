#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
import Queue
import random
import math

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
    idx = 0

    while True:
        if info_getter.current_vel is not None and info_getter.current_laserscan is not None:
            current_vel = info_getter.current_vel
            current_ranges = info_getter.current_laserscan.ranges
            angle_min = info_getter.current_laserscan.angle_min
            angle_increment = info_getter.current_laserscan.angle_increment

            caution_decelerate(current_vel)
            emergency_vertical_stabilize(current_ranges)
            go_forward(current_ranges, idx, angle_min, angle_increment)

            idx += 1


def go_through_right_direction(current_ranges, angle_min, angle_increment, idx):
    print("GO THROUGH RIGHT DIRECTION")
    sum_y_acc = (sum(math.sin((angle_min + angle_increment * i)) * current_ranges[i] for i in range(9))) / 9
    if idx % 2 == 0:
        x = 0.05
    else:
        x = -0.05
    y = sum_y_acc

    accelerate(x, y)


def caution_decelerate(current_vel):
    if current_vel.x >= 0.3:
        accelerate(-1., 0.)

    if current_vel.y >= 0.1:
        accelerate(0., -1.)
        print("CAUTIOUS Y DECCELERATE")

    if current_vel.y <= -0.1:
        accelerate(0., 1.)
        print("CAUTIOUS Y DECCELERATE")


def go_forward(current_ranges, idx, angle_min, angle_increment):
    safety_conditions = (current_ranges[4] > 2 and current_ranges[3] > 2 and current_ranges[5] > 2)

    if safety_conditions:
        print("SAFETY CONDITIONS")
        if idx % 2 == 0:
            accelerate(0.3, 0)
        else:
            accelerate(-0.3, 0)

    else:
        go_through_right_direction(current_ranges, angle_min, angle_increment, idx)


def emergency_vertical_stabilize(current_ranges):
    if (current_ranges[8] < 0.4 and current_ranges[7] < 0.4 and current_ranges[6] < 0.4 and current_ranges[5] > 0.4) or (current_ranges[5] < 0.6 and current_ranges[4] > 1.):

        print("EMERGENCY UPPER CASE STABILIZE")

        accelerate(-3., -0.3)

    if (current_ranges[0] < 0.4 and current_ranges[1] < 0.4 and current_ranges[2] < 0.4 and current_ranges[3] > 0.4) or (current_ranges[3] < 0.6 and current_ranges[4] > 1.):

        print("EMERGENCY LOWER CASE STABILIZE")

        accelerate(-3., 0.3)


def accelerate(x, y):
    pub_acc_cmd.publish(Vector3(x, y, 0))


if __name__ == '__main__':
    try:
        initNode()
    except rospy.ROSInterruptException:
        pass