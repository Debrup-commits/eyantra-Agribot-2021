#!/usr/bin/env python3

import actionlib
import copy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
import moveit_commander
import moveit_msgs.msg
from nav_msgs.msg import Odometry
import roslib
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
import sys
import tf2_ros
import tf_conversions
from tf.transformations import euler_from_quaternion

class Controller:
    def __init__(self):
        rospy.init_node('ebot_controller')

        #arm stuff
        #planning groups
        self._planning_group_1 = "arm"
        self._planning_group_2 = "gripper"

        #commander
        self._commander = moveit_commander.roscpp_initialize(sys.argv)

        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()

        #groups
        self._group_1 = moveit_commander.MoveGroupCommander(self._planning_group_1)
        self._group_2 = moveit_commander.MoveGroupCommander(self._planning_group_2)

        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        #planning frames
        self._planning_frame_1 = self._group_1.get_planning_frame()
        self._planning_frame_2 = self._group_2.get_planning_frame()

        #end effector links
        self._eef_link_1 = self._group_1.get_end_effector_link()
        self._eef_link_2 = self._group_2.get_end_effector_link()

        #group names
        self._group_names = self._robot.get_group_names()

        # rospy.loginfo(
        #     '\033[94m' + "Planning Group 1: {}".format(self._planning_frame_1) + '\033[0m')
        # rospy.loginfo(
        #     '\033[94m' + "End Effector Link 1: {}".format(self._eef_link_1) + '\033[0m')
        # rospy.loginfo(
        #     '\033[94m' + "Planning Group 2: {}".format(self._planning_frame_2) + '\033[0m')
        # rospy.loginfo(
        #     '\033[94m' + "End Effector Link 2: {}".format(self._eef_link_2) + '\033[0m')
        # rospy.loginfo(
        #     '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')
        #
        # rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

        #camera feed
        self.image_sub = rospy.Subscriber("/camera/color/image_raw2", Image, self.image_callback)

        #Bot movement stuff
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
                self.go_to_predefined_pose("rotate_90")
                moveit_commander.roscpp_shutdown()
                self.rotate(0, -1)
            if self.stage == 1:
                self.orient(0.94, 0.75, 0.5)
            if self.stage == 2:
                self.rotate(1.57, 1)
            if self.stage == 3:
                self.followTroughs(1)
            if self.stage == 4:
                self.orient(-1.57, 0.9, 1)
            if self.stage == 5:
                self.followTroughs(1)
            if self.stage == 6:
                self.stop()
                self.pub.publish(self.velocity_msg)
                rospy.loginfo("Task completed")
                rospy.signal_shutdown("Task 3 finished")

            self.pub.publish(self.velocity_msg)

    def go_to_predefined_pose(self, arg_pose_name):
        rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')

        if arg_pose_name == "open" or arg_pose_name == "close":
            self._group_2.set_named_target(arg_pose_name)
            self._group_2.plan()
            flag_plan = self._group_2.go(wait=True)
        else:
             self._group_1.set_named_target(arg_pose_name)
             self._group_1.plan()
             flag_plan = self._group_1.go(wait=True)

        rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')

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

            print("Rotating...current orientation: ", self.theta)
        else:
            self.velocity_msg.angular.z = 0
            print("Target orientation achieved!")
            self.stage += 1

    def orient(self, final_orientation, linear_vel, angular_vel):
        if abs(self.theta - final_orientation) > 0.1:
            self.velocity_msg.linear.x = linear_vel
            self.velocity_msg.angular.z = angular_vel

            print("Orienting...current orientation: ", self.theta)
        else:
            self.stop()
            print("Target orientation achieved!")
            self.stage += 1

    def followTroughs(self, linear_vel):
        if self.bright < 1 and self.bleft > 1:
            error = 0.58-self.bright
            angular_vel = self.pid(error, self.params)

            self.velocity_msg.linear.x = linear_vel
            self.velocity_msg.angular.z = angular_vel

            print("Following troughs...distance from trough row: ", self.bright)

        if self.bleft < 1 and self.bright > 1:
            error = 0.58-self.bleft
            angular_vel = self.pid(error, self.params)

            self.velocity_msg.linear.x = linear_vel
            self.velocity_msg.angular.z = -angular_vel

            print("Following troughs...distance from trough row: ", self.bleft)

        if self.bleft < 1 and self.bright < 1:

            error = self.bleft-self.bright
            angular_vel = self.pid(error, self.params)

            self.velocity_msg.linear.x = linear_vel
            self.velocity_msg.angular.z = angular_vel

            print("Troughs on both sides...distance from left & right row respectively: ", self.bleft, self.bright)

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

    def image_callback(self, data):
        # Initializing variables
        global cv_image
        focal_length = 554.387
        center_x = 320.5
        center_y = 240.5
        aruco_dimension = 0.1
        try:
            bridge = CvBridge()
            frame = bridge.imgmsg_to_cv2(data, "bgr8")
            # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # # load the dictionary that was used to generate the markers
            # dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_7X7_1000)
            #
            # # initializing the detector parameters with default values
            # parameters =  cv.aruco.DetectorParameters_create()
            #
            # # detect the markers in the frame
            # corners, ids, rejectedCandidates = cv.aruco.detectMarkers(frame, dictionary, parameters=parameters)
            #
            # if len(corners) > 0:
            #     # Flatten the ArUco IDs list
            #     ids = ids.flatten()
            #     # loop over the detected ArUCo corners
            #     for (markerCorner, markerID) in zip(corners, ids):
            #         # extract the marker corners (which are always returned
            #         # in top-left, top-right, bottom-right, and bottom-left
            #         # order)
            #         corners = markerCorner.reshape((4, 2))
            #         (topLeft, topRight, bottomRight, bottomLeft) = corners
            #         # convert each of the (x, y)-coordinate pairs to integers
            #         topRight = (int(topRight[0]), int(topRight[1]))
            #         bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            #         bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            #         topLeft = (int(topLeft[0]), int(topLeft[1]))
            #
            #         # draw the bounding box of the ArUCo detection
            #         cv.line(frame, topLeft, topRight, (0, 255, 0), 2)
            #         cv.line(frame, topRight, bottomRight, (0, 255, 0), 2)
            #         cv.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
            #         cv.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)
            #         # compute and draw the center (x, y)-coordinates of the ArUco
            #         # marker
            #         cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            #         cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            #         cv.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
            #
            #         pixel_width = topLeft[1] - bottomRight[1]
            #
            #         # draw the ArUco marker ID on the frame
            #         cv.putText(frame, str(markerID),
            #             (topLeft[0], topLeft[1] - 15), cv.FONT_HERSHEY_SIMPLEX,
            #             0.5, (0, 255, 0), 2)
            #
            #         '''uncomment to view aruco ID and verify the working of the code'''
            #         #print("[INFO] ArUco marker ID: {}".format(markerID))
            #
            #         # obtain depth for each ArUco marker
            #         distance = (focal_length*aruco_dimension)/pixel_width
            #
            #         # transforming pixel coordinates to world coordinates
            #         world_x = (cX - center_x)/focal_length*distance
            #         world_y = (cY - center_y)/focal_length*distance
            #         world_z = distance
            #
            #         # broadcasting TF for each aruco marker
            #         br = tf2_ros.TransformBroadcaster()
            #         t = geometry_msgs.msg.TransformStamped()
            #         t.header.stamp = rospy.Time.now()
            #         t.header.frame_id = "sjcam_link"
            #         t.child_frame_id = "aruco"+str(markerID)
            #
            #         # putting world coordinates coordinates as viewed for sjcam frame
            #         t.transform.translation.x = world_z
            #         t.transform.translation.y = -world_x
            #         t.transform.translation.z = world_y
            #         # not extracting any orientation thus orientation is (0, 0, 0)
            #         q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
            #         t.transform.rotation.x = q[0]
            #         t.transform.rotation.y = q[1]
            #         t.transform.rotation.z = q[2]
            #         t.transform.rotation.w = q[3]
            #
            #         br.sendTransform(t)

            '''uncoment to view the visual of detection'''
            cv2.imshow("frame", frame)
            cv2.waitKey(1)
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    ct = Controller()
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
