#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import cv2
import sys

class Revolve:
    def __init__(self):
        rospy.init_node('node_turtle_revolve')

        self.sub = rospy.Subscriber('/turtle1/pose', Pose, self.callback)
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 1)

        self.rate = rospy.Rate(10)
        self.flag=0

        self.bot_pose = 0

        self.pose = Pose()
        self.vel = Twist()

    def callback(self, data):

        if not rospy.is_shutdown():
            self.pose = data
            x = round(self.pose.x, 4)
            y = round(self.pose.y, 4)
            z = round(self.pose.theta, 4)
            if(z>0.3):
                self.flag = 1
            if(z>1.34):
                self.flag = -1


            print(x, y,z)
            print(self.flag)
            if(self.flag==0):
                self.vel.linear.x = 3
                self.vel.angular.z = 3
            elif self.flag==-1:
                self.vel.linear.x = 0
                self.vel.angular.z = 0
            else:
                self.vel.linear.x = 3
                self.vel.angular.z = -3
            
            self.pub.publish(self.vel)

            rospy.loginfo("Moving in a circle")
            self.rate.sleep()

            if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.signal_shutdown("user command")

if __name__== '__main__':
    rv = Revolve()
    try:
        if not rospy.is_shutdown():
            rospy.spin()
            rospy.loginfo("shutdown....")
    except rospy.ROSInterruptException as e:
        print(e)