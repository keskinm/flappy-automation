#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
import time
import Queue

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
    ranges_q = Queue.Queue(maxsize=5)

    while True:
        if info_getter.current_vel is not None and info_getter.current_laserscan is not None:
            current_vel = info_getter.current_vel
            current_ranges = info_getter.current_laserscan.ranges
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

            # if current_vel.x == 0 and current_vel.y == 0:
            #     indice = np.argmax(current_ranges)
            #     if indice > 4:
            #         accelerate(0., -0.1)
            #     elif indice < 4:
            #         accelerate(0., 0.1)

            # print(upper_mean_greater(current_ranges))
            # print(current_ranges[4])

            # go_forward = (current_ranges[4] >= 3.5)
            #
            # if go_forward:
            #     accelerate(0.3, 0.)

            if idx >= 5:
                ranges_q.get()
                ranges_q.put(current_ranges)
            else:
                ranges_q.put(current_ranges)

            if idx >= 5:
                forward_laser_sequence = [list(ranges_q.queue)[time_step][4] for time_step in range(5)]

                go_forward = len(set(forward_laser_sequence)) <= 1 and current_ranges[4] >= 3.1 and not(current_ranges[3] < 0.1 or current_ranges[5] < 0.1)

                print(current_ranges[3:6])

                if go_forward:
                    accelerate(1., 0.)

                if upper_mean_greater(current_ranges) and current_ranges[4] < 3.5:
                    accelerate(-3., 0.04)

                elif not upper_mean_greater(current_ranges) and current_ranges[4] < 3.5:
                    accelerate(-3., -0.04)

            # if current_vel.x >= 0.5:
            #     accelerate(-0.3, 0.)

            # ranges_diff = compute_ranges_diff(current_ranges, list(ranges_q.queue)[0])
            # if np.any(ranges_diff):
            #     print(ranges_diff)

            idx += 1


def upper_mean_greater(ranges):
    if ranges[0] + ranges[1] + ranges[2] + ranges[3] < ranges[5] + ranges[6] + ranges[7] + ranges[8]:
        return True
    return False


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
