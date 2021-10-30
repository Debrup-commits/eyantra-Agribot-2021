#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import cv2

class Controller:
    def __init__(self):
        rospy.init_node('ebot_controller')

        self.bright = 0
        self.fright = 0
        self.front = 0
        self.fleft = 0
        self.bleft = 0

        self.x = 0
        self.y = 0
        self.theta = 0

        self.count_lt1 = 0
        self.count_mt1 = 0

        self.intg, self.last_error = 0.0, 0.0
        self.params = {'KP': 2, 'KD': 3, 'KI': 0, 'SP': 0.5}

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        rospy.Subscriber('/ebot/laser/scan', LaserScan, self.laser_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.rate = rospy.Rate(10)

        self.velocity_msg = Twist()

        self.velocity_msg.linear.x = 0
        self.velocity_msg.angular.z = 0
        self.pub.publish(self.velocity_msg)

        while not rospy.is_shutdown():
            print("left value: ", self.fleft)
            print("right value: ", self.fright)

            # self.rotate(3.14, 1)
            self.followTubs()
            self.pub.publish(self.velocity_msg)
            # print("Controller message pushed at {}".format(rospy.get_time()))
            self.rate.sleep()

        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.velocity_msg.linear.x = 0
            self.velocity_msg.angular.z = 0
            rospy.signal_shutdown("user command")

    def rotate(self, final_orientation, angular_vel):

        if abs(abs(self.theta)-abs(final_orientation)) > 0.1:
            self.velocity_msg.angular.z = angular_vel
        else:
            self.velocity_msg.angular.z = 0

    def pid(self, error, const):
        prop = error
        self.intg = error + self.intg
        diff = error - self.last_error
        balance = const['KP'] * prop + const['KI'] * self.intg + const['KD'] * diff
        self.last_error = error
        return balance

    def followTubs(self):
        if self.fright < 1 and self.fleft > 1:
            error = 0.6-self.fright
            angular_vel = self.pid(error, self.params)

            self.velocity_msg.linear.x = 0.5
            self.velocity_msg.angular.z = angular_vel

        if self.fleft < 1 and self.fright > 1:
            error = 0.6-self.fleft
            angular_vel = self.pid(error, self.params)

            self.velocity_msg.linear.x = 0.5
            self.velocity_msg.angular.z = -angular_vel

        if self.fleft < 1 and self.fright < 1:

            # if self.count_lt1 == 0:
            #     self.velocity_msg.angular.z = 0.5
            #
            # if self.count_lt1 == 2:
            error = self.fleft-self.fright
            angular_vel = self.pid(error, self.params)

            self.velocity_msg.linear.x = 0.5
            self.velocity_msg.angular.z = angular_vel

        if self.fleft > 1 and self.fright > 1:
            # self.velocity_msg.linear.x = 0
            # self.velocity_msg.angular.z = 0
            # if 1.55 < abs(self.theta) < 1.59:
            #     self.count_mt1 +=1
            #     if self.count_mt1 == 1:
            self.velocity_msg.angular.z = -5
            #     if self.count_mt1 == 2:
            #         rospy.loginfo("Task completed")
            #         rospy.signal_shutdown("Task completed")
            # else:
            #     self.velocity_msg.angular.z = 1

    def odom_callback(self, data):
        x  = data.pose.pose.orientation.x;
        y  = data.pose.pose.orientation.y;
        z = data.pose.pose.orientation.z;
        w = data.pose.pose.orientation.w;

        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.theta = euler_from_quaternion([x,y,z,w])[2]

    def laser_callback(self, msg):
        self.bright = min(min(msg.ranges[0:144]), msg.range_max)
        self.fright = min(min(msg.ranges[144:288]), msg.range_max)
        self.front = min(min(msg.ranges[288:432]), msg.range_max)
        self.fleft = min(min(msg.ranges[432:576]), msg.range_max)
        self.bleft = min(min(msg.ranges[576:720]), msg.range_max)

if __name__ == '__main__':
    ct = Controller()
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
