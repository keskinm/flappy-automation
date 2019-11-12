#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
import time
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
    ranges_q = Queue.Queue(maxsize=5)
    vels_q = Queue.Queue(maxsize=50000)

    while True:
        if info_getter.current_vel is not None and info_getter.current_laserscan is not None:
            current_vel = info_getter.current_vel
            current_ranges = info_getter.current_laserscan.ranges
            current_time = info_getter.current_laserscan.header.stamp
            angle_min = info_getter.current_laserscan.angle_min
            angle_increment = info_getter.current_laserscan.angle_increment

            # print(current_ranges)

            if idx >= 5:
                ranges_q.get()
                ranges_q.put(current_ranges)
            else:
                ranges_q.put(current_ranges)

            if idx >= 50000:
                vels_q.get()
                vels_q.put(current_vel)
            else:
                vels_q.put(current_vel)

            if idx >= 5:
                forward_laser_sequence = [list(ranges_q.queue)[time_step][4] for time_step in range(5)]
                upper_laser_sequence = [list(ranges_q.queue)[time_step][5] for time_step in range(5)]
                lower_laser_sequence = [list(ranges_q.queue)[time_step][3] for time_step in range(5)]

                # print(forward_laser_sequence)

                if idx >= 50000:
                    x_vels_sequence = [list(vels_q.queue)[time_step].x for time_step in range(300)]
                    # start_stuck_handler(x_vels_sequence, current_ranges)

                # go_through_right_direction(current_ranges, angle_min, angle_increment, idx, current_time)

                # if idx >= 10:
                #     time.sleep(100000)

                caution_decelerate(current_vel)
                # hard_case_stabilize(upper_laser_sequence, lower_laser_sequence)
                # emergency_horizontal_decelerate(current_ranges)
                emergency_vertical_stabilize(current_ranges)
                go_forward(current_ranges, forward_laser_sequence, idx, angle_min, angle_increment, current_time)

            idx += 1


def go_through_right_direction(current_ranges, angle_min, angle_increment, idx, current_time):
    biggest_laser_range_index = np.argmax(current_ranges)
    right_dir_angle = angle_min + angle_increment*biggest_laser_range_index

    print("GO THROUGH RIGHT DIRECTION")

    if random.random() < 1.:
        sum_x_acc = sum(math.cos((angle_min + angle_increment * i)) * current_ranges[i] for i in range(9)) / 9
        sum_y_acc = (sum(math.sin((angle_min + angle_increment * i)) * current_ranges[i] for i in range(9))) / 9
        if idx % 2 == 0:
            x = 0.05
        else:
            x = -0.05
        y = sum_y_acc

    else:
        x = 0.05
        y = math.sin(right_dir_angle)/2

    accelerate(x, y)


def start_stuck_handler(x_vels_sequence, current_ranges):
    if not np.any(x_vels_sequence) and current_ranges[4] < 1.:

        print("STUCK HANDLER")

        if random.random() < 0.5:
            accelerate(-3., 0.04)
        else:
            accelerate(-3., 0.04)


def caution_decelerate(current_vel):
    if current_vel.x >= 0.3:
        accelerate(-1., 0.)

    if current_vel.y >= 0.1:
        accelerate(0., -1.)
        print("CAUTIOUS Y DECCELERATE")

    if current_vel.y <= -0.1:
        accelerate(0., 1.)
        print("CAUTIOUS Y DECCELERATE")


def go_forward(current_ranges, forward_laser_sequence, idx, angle_min, angle_increment, current_time):
    safety_conditions = (current_ranges[4] > 2 and current_ranges[3] > 2 and current_ranges[5] > 2)

    if safety_conditions:
        print("SAFETY CONDITIONS")
        if idx % 2 == 0:
            accelerate(0.3, 0)
        else:
            accelerate(-0.3, 0)

    else:
        if random.random() <= 1.:
            go_through_right_direction(current_ranges, angle_min, angle_increment, idx, current_time)

        else:
            stabilize_wrt_means(current_ranges)


def hard_case_stabilize(upper_laser_sequence, lower_laser_sequence):
    if upper_laser_sequence[4] < 0.2 and upper_laser_sequence[0] < 0.2:
        accelerate(-3., 0.04)

        print("UPPER HARD CASE STABILIZE")

    elif lower_laser_sequence[4] < 0.2 and lower_laser_sequence[0] < 0.2:
        accelerate(-3., -0.04)

        print("LOWER HARD CASE STABILIZE")


def emergency_vertical_stabilize(current_ranges):
    if (current_ranges[8] < 0.4 and current_ranges[7] < 0.4 and current_ranges[6] < 0.4 and current_ranges[5] > 0.4) or (current_ranges[5] < 0.6 and current_ranges[4] > 1.):

        print("EMERGENCY UPPER CASE STABILIZE")

        accelerate(-3., -0.3)

    if (current_ranges[0] < 0.4 and current_ranges[1] < 0.4 and current_ranges[2] < 0.4 and current_ranges[3] > 0.4) or (current_ranges[3] < 0.6 and current_ranges[4] > 1.):

        print("EMERGENCY LOWER CASE STABILIZE")

        accelerate(-3., 0.3)


def emergency_horizontal_decelerate(current_ranges):
    if current_ranges[3] < 0.5 and current_ranges[4] < 0.5 and current_ranges[5] <0.5:
        accelerate(-3., 0.)
        print("EMERGENCY HORIZONTAL DECELERATE")


def stabilize_wrt_means(current_ranges):
    if upper_mean_greater(current_ranges):
        accelerate(-1.5, 0.1)

        print("STABILIZE WRT UPPER MEAN")

    elif not upper_mean_greater(current_ranges):
        accelerate(-1.5, -0.1)

        print("STABILIZE WRT LOWER MEAN")


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
