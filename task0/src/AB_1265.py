#!/usr/bin/env python3

#Importing all important libraries

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import cv2
from cv_bridge import CvBridge, CvBridgeError

class Revolve:
    def __init__(self):
        rospy.init_node('turtlesim_revolver')

        #Subscriber, Publisher
        self.sub = rospy.Subscriber('/turtle1/pose', Pose, self.callback)
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 10)

        #Determines rate of publishing on the publisher node
        self.rate = rospy.Rate(10)

        self.bot_pose = 0

        self.pose = Pose()
        self.vel = Twist()

    def callback(self, data):

        try:
            self.pose = data
            a = self.pose.theta

            #logic
            if self.bot_pose == 0:
                self.vel.linear.x = 3
                self.vel.angular.z = -3

                if 0 < a < 0.05:
                    self.bot_pose +=1

            if self.bot_pose == 1:
                self.vel.linear.x = 3
                self.vel.angular.z = 3

                if -0.05 < a < 0:
                    self.bot_pose += 1

            if self.bot_pose == 2:
                self.vel.linear.x = 0
                self.vel.angular.z = 0

            #Publishing the messages
            self.pub.publish(self.vel)

            if self.bot_pose < 2:
                rospy.loginfo("Moving in a circle")
            else:
                rospy.loginfo("Target reached")
                rospy.signal_shutdown("Task 0 completed")

        except CvBridgeError as e:
            print(e)

if __name__== '__main__':
    rv = Revolve()
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
