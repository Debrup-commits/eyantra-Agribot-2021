#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

def control_loop():
    rospy.init_node('ebot_controller')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)

    rate = rospy.Rate(10)

    velocity_msg = Twist()
    pose_msg = Odometry()
    laser_msg = LaserScan()

    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)

    while not rospy.is_shutdown():

        #
        # Your algorithm to complete the obstacle course
        #

        # velocity_msg.linear.x =
        # velocity_msg.angular.z =
        pub.publish(velocity_msg)
        print("Controller message pushed at {}".format(rospy.get_time()))
        rate.sleep()

def odom_callback(data):
    global pose
    x  = data.pose.pose.orientation.x;
    y  = data.pose.pose.orientation.y;
    z = data.pose.pose.orientation.z;
    w = data.pose.pose.orientation.w;
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]

def laser_callback(msg):
    global regions
    regions = {
        'bright': min(min(msg.ranges[0:144]), msg.range_max),
        'fright': min(min(msg.ranges[144:288]), msg.range_max),
        'front':  min(min(msg.ranges[288:432]), msg.range_max),
        'fleft':  min(min(msg.ranges[432:576]), msg.range_max),
        'bleft':  min(min(msg.ranges[576:720]), msg.range_max),
    }

    # rospy.loginfo(regions)


if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass
