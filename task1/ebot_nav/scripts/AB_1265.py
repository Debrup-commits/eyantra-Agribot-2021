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

        #Parameters for laser scanning
        self.bright = 0
        self.fright = 0
        self.front = 0
        self.fleft = 0
        self.bleft = 0

        #Parameters for Odometry
        self.x = 0
        self.y = 0
        self.theta = 0

        #Finite state machines approach
        self.stage = 0

        #PID Parameters
        self.intg, self.last_error = 0.0, 0.0
        self.params = {'KP': 4, 'KD': 2, 'KI': 0}

        #Publisher
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        #Subscribers
        rospy.Subscriber('/ebot/laser/scan', LaserScan, self.laser_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        #Rate of publishing
        self.rate = rospy.Rate(100)

        #ROS msg
        self.velocity_msg = Twist()

        self.pub.publish(self.velocity_msg)

        while not rospy.is_shutdown():

            #Algorithm:

            if self.stage == 0:
                self.rotate(3.14, 1)
            if self.stage == 1:
                self.orient(2.2, 0.5, -0.5)
            if self.stage == 2:
                self.rotate(1.57, -1)
            self.pub.publish(self.velocity_msg)
            if self.stage == 3:
                self.followTroughs(1)
            if self.stage == 4:
                self.orient(-1.57, 0.5, -2.1)
            if self.stage == 5:
                self.followTroughs(1)
            if self.stage == 6:
                self.orient(1.57, 0.5, 1.3)
            if self.stage == 7:
                self.followTroughs(1)
            if self.stage == 8:
                self.stop()
                self.pub.publish(self.velocity_msg)
                rospy.signal_shutdown("Task 1 finished")

    def pid(self, error, const):
        prop = error
        self.intg = error + self.intg
        diff = error - self.last_error
        balance = const['KP'] * prop + const['KI'] * self.intg + const['KD'] * diff
        self.last_error = error
        return balance

    def stop(self):
        self.velocity_msg.linear.x = 0
        self.velocity_msg.angular.z = 0

    def rotate(self, final_orientation, angular_vel):

        if abs(self.theta-final_orientation) > 0.1:
            self.velocity_msg.angular.z = angular_vel
        else:
            self.velocity_msg.angular.z = 0
            self.stage += 1

    def orient(self, final_orientation, linear_vel, angular_vel):
        if abs(self.theta - final_orientation) > 0.1:
            self.velocity_msg.linear.x = linear_vel
            self.velocity_msg.angular.z = angular_vel
        else:
            self.stop()
            self.stage += 1

    def followTroughs(self, linear_vel):
        if self.bright < 1 and self.bleft > 1:
            error = 0.58-self.bright
            angular_vel = self.pid(error, self.params)

            self.velocity_msg.linear.x = linear_vel
            self.velocity_msg.angular.z = angular_vel

        if self.bleft < 1 and self.bright > 1:
            error = 0.58-self.bleft
            angular_vel = self.pid(error, self.params)

            self.velocity_msg.linear.x = linear_vel
            self.velocity_msg.angular.z = -angular_vel

        if self.bleft < 1 and self.bright < 1:

            error = self.bleft-self.bright
            angular_vel = self.pid(error, self.params)

            self.velocity_msg.linear.x = linear_vel
            self.velocity_msg.angular.z = angular_vel

        if self.bleft > 1 and self.bright > 1:
            self.stop()
            if self.stage < 8:
                self.stage += 1

    def odom_callback(self, data):
        x  = data.pose.pose.orientation.x;
        y  = data.pose.pose.orientation.y;
        z = data.pose.pose.orientation.z;
        w = data.pose.pose.orientation.w;

        self.x = data.pose.pose.position.x # x coordinate of bot
        self.y = data.pose.pose.position.y # y coordinnate of bot
        self.theta = euler_from_quaternion([x,y,z,w])[2] #real time orientation of bot

    def laser_callback(self, msg):
        self.bright = min(min(msg.ranges[0:144]), msg.range_max)    # distance of the closest object in the back right region
        self.fright = min(min(msg.ranges[144:288]), msg.range_max)  # distance of the closest object in the front right region
        self.front = min(min(msg.ranges[288:432]), msg.range_max)   # distance of the closest object in the front region
        self.fleft = min(min(msg.ranges[432:576]), msg.range_max)   # distance of the closest object in the front left region
        self.bleft = min(min(msg.ranges[576:720]), msg.range_max)   # distance of the closest object in the back lrft region

if __name__ == '__main__':
    ct = Controller()
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
