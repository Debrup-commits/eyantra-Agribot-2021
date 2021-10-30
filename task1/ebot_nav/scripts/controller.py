#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class Controller:
    def __init__(self):
        rospy.init_node('ebot_controller')

        self.regions = {}
        self.pose = []

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        rospy.Subscriber('/ebot/laser/scan', LaserScan, self.laser_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.rate = rospy.Rate(10)

        self.velocity_msg = Twist()

        self.velocity_msg.linear.x = 0
        self.velocity_msg.angular.z = 0
        self.pub.publish(self.velocity_msg)

        while not rospy.is_shutdown():
            print(self.pose)
            # if self.pose[2] != 3.14:
            #     self.velocity_msg.angular.z = 1

            self.pub.publish(self.velocity_msg)
            print("Controller message pushed at {}".format(rospy.get_time()))
            self.rate.sleep()

    def odom_callback(self, data):
        x  = data.pose.pose.orientation.x;
        y  = data.pose.pose.orientation.y;
        z = data.pose.pose.orientation.z;
        w = data.pose.pose.orientation.w;
        self.pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]

    def laser_callback(self, msg):
        self.regions = {
            'bright': min(min(msg.ranges[0:144]), msg.range_max),
            'fright': min(min(msg.ranges[144:288]), msg.range_max),
            'front':  min(min(msg.ranges[288:432]), msg.range_max),
            'fleft':  min(min(msg.ranges[432:576]), msg.range_max),
            'bleft':  min(min(msg.ranges[576:720]), msg.range_max),
        }

if __name__ == '__main__':
    ct = Controller()
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
