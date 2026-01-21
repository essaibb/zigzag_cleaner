#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import math
import tf


class WallFollowerMapping:
    def __init__(self):
        rospy.init_node("wall_follower_mapping")

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # 🔧 معاملات التحكم
        self.Kp = 0.9
        self.Kd = 0.2
        self.desired_dist = 0.70
        self.safe_front_dist = 0.6

        self.last_error = 0.0
        self.front_dist = 2.0
        self.right_dist = 2.0

        #  الدوران 90°
        self.state = "FOLLOW_WALL"
        self.yaw = 0.0
        self.target_yaw = 0.0

        self.rate = rospy.Rate(10)

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        ranges[np.isinf(ranges)] = msg.range_max
        ranges[np.isnan(ranges)] = msg.range_max

        front_arc = np.concatenate((ranges[0:20], ranges[-20:]))
        self.front_dist = np.mean(front_arc)

        right_arc = ranges[260:300]
        self.right_dist = np.mean(right_arc)

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        (_, _, self.yaw) = tf.transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w]
        )

    def angle_error(self, target):
        return math.atan2(
            math.sin(target - self.yaw),
            math.cos(target - self.yaw)
        )

    def run(self):
        rospy.loginfo(" Wall Following with 90° Left Turn Started")

        while not rospy.is_shutdown():
            cmd = Twist()

            #  حالة لف 90 درجة
            if self.state == "TURN_LEFT_90":
                error = self.angle_error(self.target_yaw)

                if abs(error) < 0.03:
                    cmd.angular.z = 0.0
                    self.state = "FOLLOW_WALL"
                    rospy.loginfo(" 90° turn completed")
                else:
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.5

            #  تتبع الجدار
            else:
                #  جدار أمامي → ابدأ لف 90°
                if self.front_dist < self.safe_front_dist:
                    self.state = "TURN_LEFT_90"
                    self.target_yaw = self.yaw + math.pi / 2
                    rospy.logwarn("Front wall detected → turning 90° left")

                #  فقدان الجدار
                elif self.right_dist > 1.2:
                    cmd.linear.x = 0.15
                    cmd.angular.z = -0.4

                #  Wall Following طبيعي
                else:
                    error = self.right_dist - self.desired_dist
                    derivative = error - self.last_error

                    angular = (self.Kp * error) + (self.Kd * derivative)
                    angular = max(min(angular, 0.6), -0.6)

                    cmd.linear.x = 0.18 - abs(error) * 0.3
                    cmd.linear.x = max(cmd.linear.x, 0.08)

                    cmd.angular.z = angular
                    self.last_error = error

            self.cmd_pub.publish(cmd)
            self.rate.sleep()


if __name__ == "__main__":
    try:
        node = WallFollowerMapping()
        node.run()
    except rospy.ROSInterruptException:
        pass

