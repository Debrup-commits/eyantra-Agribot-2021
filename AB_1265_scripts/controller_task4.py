#!/usr/bin/env python3

#This script is responsible for the state wise maneuvering, coordinate broadcasting and arm manipulation of the Bot

import actionlib
import copy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
from nav_msgs.msg import Odometry
import message_filters
import numpy as np
import roslib
import rospy
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import LaserScan
import sys
import tf2_ros
import tf_conversions
import tf
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

        #action server
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

        #camera feed
        self.image_sub = message_filters.Subscriber("/camera/color/image_raw2", Image)
        self.depth_sub = message_filters.Subscriber("/camera/depth/image_raw2", Image)
        self.aruco_sub = message_filters.Subscriber("/ebot/camera1/image_raw", Image)

        self.rgb_frame = None
        self.depth_frame = None
        self.aruco_frame = None

        #some flags
        self.call_counter = 0
        self.trough_count = 0
        self.tomato_count = 0
        self.contour_count = 0
        self.trough_traversal_count = 0
        self.identification_count = 0
        self.pose_status = None

        #array to store info about missed tomatoes
        self.missed_tomatoes = []

        self.bridge = CvBridge()

        #Time synchronizer to work with multiple camera topics
        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.depth_sub, self.aruco_sub], queue_size = 30, slop = 0.5)
        self.ts.registerCallback(self.image_callback)

        #listens to the broadcasted tf of the tomatoes
        listener = tf.TransformListener()

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
        self.stage = -1

        #PID Parameters
        self.intg, self.last_error = 0.0, 0.0
        self.params = {'KP': 8, 'KD': 4, 'KI': 0}

        #Publisher
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        #Subscribers
        rospy.Subscriber('/ebot/laser/scan', LaserScan, self.laser_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        #ROS msg
        self.velocity_msg = Twist()

        self.pub.publish(self.velocity_msg)

        while not rospy.is_shutdown():

            #Algorithm:
            ur5_pose = geometry_msgs.msg.Pose()

            if self.stage == -1:
                rospy.loginfo("Started Run!")
                self.go_to_predefined_pose("rotate_90_2")
                self.stage += 1

            if self.stage == 0:
                self.rotate(0, 1)

            if self.stage == 1:
                self.orient(0.9, 0.8, 0.53)

            if self.stage == 2:
                self.rotate(1.57, 1)

            if self.stage == 3:
                #detects aruco markers
                aruco_detected = self.detect_aruco(self.aruco_frame)

                if aruco_detected == True:
                    if self.call_counter == 0:
                        self.stop()
                        if self.trough_traversal_count == 0:
                            if self.trough_count < 10:
                                rospy.loginfo("Trough_no 0{} reached".format(self.trough_count))
                            else:
                                rospy.loginfo("Trough_no {} reached".format(self.trough_count))

                            self.trough_traversal_count += 1

                        rospy.sleep(1)

                        #broadcasts the tomatoes detected under depth 1 metre and returns number of such tomatoes available
                        tomato_num = self.broadcast_tomato_coordinates(self.rgb_frame, self.depth_frame)

                        if tomato_num > 0:
                            try:
                                for i in range(tomato_num):
                                    if self.identification_count == 0:
                                        if self.trough_count < 10:
                                            rospy.loginfo("Obj_{} Identified at Trough_no 0{}".format(self.tomato_count, self.trough_count))
                                        else:
                                            rospy.loginfo("Obj_{} Identified at Trough_no {}".format(self.tomato_count, self.trough_count))

                                        self.identification_count += 1

                                    #gives the final converted tfs where the arm has to go
                                    trans, rot = listener.lookupTransform(target_frame='/ebot_base', source_frame='/obj0', time=rospy.Time(0))

                                    #arm movement
                                    self.take_to_pose(ur5_pose, trans, 0.4)
                                    self.take_to_pose(ur5_pose, trans, 0.27)

                                    #collection of the tomatos
                                    #attempts to pick only if current state of the arm is roughly the same as the pose of the tomato to be picked
                                    if abs(self.pose_status-ur5_pose.position.y)<0.01:
                                        self.go_to_predefined_pose("close")
                                        rospy.loginfo("Obj_{} Picked".format(self.tomato_count))
                                        self.go_to_predefined_pose("home")
                                        self.go_to_predefined_pose("open")
                                        rospy.loginfo("Obj_{} Dropped in front basket".format(self.tomato_count))
                                    else:
                                        #In case of a miss, relevant info regarding the missed tomato is noted
                                        self.missed_tomatoes.append([self.tomato_count, self.trough_count])

                                    self.go_to_predefined_pose("rotate_90_2")
                                    self.tomato_count += 1
                                    self.identification_count = 0

                            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                                continue

                        self.trough_count += 1
                        self.call_counter += 1
                    else:
                        self.followTroughs(0.5)
                else:
                    self.trough_traversal_count = 0
                    self.call_counter = 0
                    self.followTroughs(0.5)

            if self.stage == 4:
                self.orient(2.45, 0.5, 0.85)

            if self.stage == 5:
                self.rotate(3.14, 1)

            if self.stage == 6:
                self.orient(-2, 0.6, 1.1)

            if self.stage == 7:
                self.rotate(-1.57, 1)

            if self.stage == 8:
                #detects aruco markers
                aruco_detected = self.detect_aruco(self.aruco_frame)

                if aruco_detected == True:
                    if self.call_counter == 0:
                        self.stop()
                        if self.trough_traversal_count == 0:
                            if self.trough_count < 10:
                                rospy.loginfo("Trough_no 0{} reached".format(self.trough_count))
                            else:
                                rospy.loginfo("Trough_no {} reached".format(self.trough_count))

                            self.trough_traversal_count += 1

                        rospy.sleep(1)

                        #broadcasts the tomatoes detected under depth 1 metre and returns number of such tomatoes available
                        tomato_num = self.broadcast_tomato_coordinates(self.rgb_frame, self.depth_frame)

                        if tomato_num > 0:
                            try:
                                for i in range(tomato_num):
                                    #gives the final converted tfs where the arm has to go
                                    trans, rot = listener.lookupTransform(target_frame='/ebot_base', source_frame='/obj0', time=rospy.Time(0))

                                    #arm movement
                                    self.take_to_pose(ur5_pose, trans, 0.4)
                                    self.take_to_pose(ur5_pose, trans, 0.27)

                                    #collection of the missed tomatos
                                    if abs(self.pose_status-ur5_pose.position.y)<0.01:
                                        self.go_to_predefined_pose("close")
                                        rospy.loginfo("Obj_{} Picked".format(self.missed_tomatoes[0][0]))
                                        self.go_to_predefined_pose("home")
                                        self.go_to_predefined_pose("open")
                                        rospy.loginfo("Obj_{} Dropped in front basket".format(self.missed_tomatoes[0][0]))

                                        #if the tomato is picked, its corresponding info is popped out of the list
                                        self.missed_tomatoes.pop(0)

                                    self.go_to_predefined_pose("rotate_90_2")

                            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                                continue

                        self.trough_count += 1
                        self.call_counter += 1
                    else:
                        self.followTroughs(0.5)
                else:
                    self.trough_traversal_count = 0
                    self.call_counter = 0
                    self.followTroughs(0.5)

            if self.stage == 9:
                self.rotate(0, 1)

            if self.stage == 10:
                self.rotate(1.57, 1)

            if self.stage == 11:
                self.followTroughs(1)

            if self.stage == 12:
                self.orient(2.55, 0.5, 1)

            if self.stage == 13:
                self.rotate(3.14, 1)

            if self.stage == 14:
                self.orient(-2.15, 0.6, 1.1)

            if self.stage == 15:
                #since the bot goes back to collect tomatoesfrom the other troughs, its trough track needs to be updated
                self.trough_count = 10
                self.rotate(-1.57, 1)

            if self.stage == 16:
                #detects aruco markers
                aruco_detected = self.detect_aruco(self.aruco_frame)

                if aruco_detected == True:
                    if self.call_counter == 0:
                        self.stop()
                        rospy.sleep(1)

                        #broadcasts the tomatoes detected under depth 1 metre and returns number of such tomatoes available
                        tomato_num = self.broadcast_tomato_coordinates(self.rgb_frame, self.depth_frame)

                        if tomato_num > 0:
                            try:
                                for i in range(tomato_num):
                                    if self.identification_count == 0:
                                        if self.trough_count < 10:
                                            rospy.loginfo("Obj_{} Identified at Trough_no 0{}".format(self.tomato_count, self.trough_count))
                                        else:
                                            rospy.loginfo("Obj_{} Identified at Trough_no {}".format(self.tomato_count, self.trough_count))

                                        self.identification_count += 1

                                    #gives the final converted tfs where the arm has to go
                                    trans, rot = listener.lookupTransform(target_frame='/ebot_base', source_frame='/obj0', time=rospy.Time(0))

                                    #arm movement
                                    self.take_to_pose(ur5_pose, trans, 0.4)
                                    self.take_to_pose(ur5_pose, trans, 0.27)

                                    #collection of the tomatos
                                    #attempts to pick only if current state of the arm is roughly the same as the pose of the tomato to be picked
                                    if abs(self.pose_status-ur5_pose.position.y)<0.01:
                                        self.go_to_predefined_pose("close")
                                        rospy.loginfo("Obj_{} Picked".format(self.tomato_count))
                                        self.go_to_predefined_pose("home")
                                        self.go_to_predefined_pose("open")
                                        rospy.loginfo("Obj_{} Dropped in front basket".format(self.tomato_count))
                                    else:
                                        #In case of a miss, relevant info regarding the missed tomato is noted
                                        self.missed_tomatoes.append([self.tomato_count, self.trough_count])

                                    self.go_to_predefined_pose("rotate_90_2")
                                    self.tomato_count += 1
                                    self.identification_count = 0

                            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                                continue

                        self.trough_count += 1
                        self.call_counter += 1
                    else:
                        self.followTroughs(0.5)
                else:
                    self.call_counter = 0
                    self.followTroughs(0.5)

            if self.stage == 17:
                #Stops the agribot and terminates the task
                self.stop()
                self.pub.publish(self.velocity_msg)
                rospy.loginfo("Mission Accomplished!")
                rospy.signal_shutdown("Mission Accomplished!")

            self.pub.publish(self.velocity_msg)

    def go_to_predefined_pose(self, arg_pose_name):
        #takes arm to a predefined pose

        #Moves the cups
        if arg_pose_name == "open" or arg_pose_name == "close":
            self._group_2.set_named_target(arg_pose_name)
            self._group_2.plan()
            flag_plan = self._group_2.go(wait=True)
        #Moves the arm
        else:
             self._group_1.set_named_target(arg_pose_name)
             self._group_1.plan()
             flag_plan = self._group_1.go(wait=True)

    def go_to_pose(self, arg_pose):

        #takes arm to a given coordinate
        pose_values = self._group_1.get_current_pose().pose

        self._group_1.set_pose_target(arg_pose)
        flag_plan = self._group_1.go(wait=True)  # wait=False for Async Move

        pose_values = self._group_1.get_current_pose().pose

        self.pose_status = pose_values.position.y

        list_joint_values = self._group_1.get_current_joint_values()

        return flag_plan

    def take_to_pose(self, pose_msg, trans, y_diff_factor):
        pose_msg.position.x = trans[0]
        pose_msg.position.y = trans[1]-y_diff_factor
        pose_msg.position.z = trans[2]
        pose_msg.orientation.x = 0
        pose_msg.orientation.y = 0
        pose_msg.orientation.z = 0
        pose_msg.orientation.w = 1

        self.go_to_pose(pose_msg)

    def pid(self, error, const):

        #controller responsible for maintaing uniform distance from the troughs
        prop = error
        self.intg = error + self.intg
        diff = error - self.last_error
        balance = const['KP'] * prop + const['KI'] * self.intg + const['KD'] * diff
        self.last_error = error
        return balance

    def stop(self):
        #Function to halt the bot wherever its called
        self.velocity_msg.linear.x = 0
        self.velocity_msg.angular.z = 0
        self.pub.publish(self.velocity_msg)

    def rotate(self, final_orientation, angular_vel):
        #Rotates the agribot
        self.velocity_msg.linear.x = 0
        error = final_orientation - self.theta
        abs_angle_diff = abs(abs(final_orientation)-abs(self.theta))

        if abs_angle_diff > 0.1:
            if error > 3.14:
                ang_vel = self.pid((error-6.28), self.params)
            elif error < -3.14:
                ang_vel = self.pid((error+6.28), self.params)
            else:
                ang_vel = self.pid(error, self.params)

            if ang_vel > 0:
                ang_vel = angular_vel
            else:
                ang_vel = -angular_vel

            self.velocity_msg.angular.z = ang_vel
        else:
            self.stop()
            rospy.sleep(0.5)
            self.stage += 1

    def orient(self, final_orientation, linear_vel, angular_vel):
        #Provides the bot with a constant linear and angular velocity to help it orient along a given orientation
        if abs(self.theta - final_orientation) > 0.1:
            self.velocity_msg.linear.x = linear_vel
            self.velocity_msg.angular.z = angular_vel
        else:
            self.stop()
            self.stage += 1

    def followTroughs(self, linear_vel):
        #uses pid and info from LIDAR to move the bot

        if self.bright < 1 and self.bleft > 1:
            error = 0.45-self.bright
            angular_vel = self.pid(error, self.params)

            self.velocity_msg.linear.x = linear_vel
            self.velocity_msg.angular.z = angular_vel

        if self.bleft < 1 and self.bright > 1:
            error = 0.45-self.bleft
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
            rospy.sleep(0.5)
            if self.stage < 17:
                self.stage += 1

    def odom_callback(self, data):
        x  = data.pose.pose.orientation.x;
        y  = data.pose.pose.orientation.y;
        z = data.pose.pose.orientation.z;
        w = data.pose.pose.orientation.w;

        self.x = data.pose.pose.position.x # x coordinate of bot
        self.y = data.pose.pose.position.y # y coordinate of bot
        self.theta = euler_from_quaternion([x,y,z,w])[2] #real time orientation of bot

    def laser_callback(self, msg):
        self.bright = min(min(msg.ranges[0:144]), msg.range_max)    # distance of the closest object in the back right region
        self.fright = min(min(msg.ranges[144:288]), msg.range_max)  # distance of the closest object in the front right region
        self.front = min(min(msg.ranges[288:432]), msg.range_max)   # distance of the closest object in the front region
        self.fleft = min(min(msg.ranges[432:576]), msg.range_max)   # distance of the closest object in the front left region
        self.bleft = min(min(msg.ranges[576:720]), msg.range_max)   # distance of the closest object in the back lrft region

    def image_callback(self, rgb_data, depth_data, aruco_data):
        try:
            self.rgb_frame = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")
            self.depth_frame = self.bridge.imgmsg_to_cv2(depth_data, "32FC1")
            self.aruco_frame = self.bridge.imgmsg_to_cv2(aruco_data, "bgr8")

            if cv2.waitKey(27) & 0xFF == ord('q'):
                rospy.signal_shutdown('user command')

        except CvBridgeError as e:
            print(e)

    def detect_aruco(self, aruco_frame):
        #This function detects the presence of aruco markers in the sjcam feed and returns a boolean flag(True/False)

        # load the dictionary that was used to generate the markers
        dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_7X7_1000)

        # initializing the detector parameters with default values
        parameters =  cv2.aruco.DetectorParameters_create()

        # detect the markers in the frame
        corners, ids, rejectedCandidates = cv2.aruco.detectMarkers(aruco_frame, dictionary, parameters=parameters)

        if len(corners) > 0:
            return True
        else:
            return False

    def broadcast_tomato_coordinates(self, rgb_frame, depth_frame):
        #This function detects red tomatoes and if within 1 m distance, their tfs are broadcasted

        # Initializing variables
        focal_length = 554.387
        center_x = 320.5
        center_y = 240.5

        #Image masking and thresholding
        img_HSV = cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2HSV)

        low_HSV = np.array([0, 53, 137])
        upper_HSV = np.array([10, 255, 255])

        mask = cv2.inRange(img_HSV, low_HSV, upper_HSV)
        output = cv2.bitwise_and(rgb_frame, rgb_frame, mask = mask)
        output = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)

        ret, threshold = cv2.threshold(output, 25, 255, cv2.THRESH_BINARY)

        #Contour detection
        contours, hierarchy = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        i=0

        for contour in contours:
            C = cv2.moments(contour)

            #centroid calculation
            cX = int(C['m10']/(C['m00']+1e-4))
            cY = int(C['m01']/(C['m00']+1e-4))
            depth = depth_frame[cY][cX]

            # transforming pixel coordinates to world coordinates
            world_x = (cX - center_x)/focal_length*depth
            world_y = (cY - center_y)/focal_length*depth
            world_z = depth

            # broadcasting TF for each tomato coordinate

            if depth < 1:
                br = tf2_ros.TransformBroadcaster()
                t = geometry_msgs.msg.TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = "camera_depth_frame2"
                t.child_frame_id = "obj" + str(i)

                # putting world coordinates coordinates as viewed for sjcam frame
                t.transform.translation.x = world_x
                t.transform.translation.y = world_y
                t.transform.translation.z = world_z

                # not extracting any orientation thus orientation is (0, 0, 0)
                q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
                t.transform.rotation.x = q[0]
                t.transform.rotation.y = q[1]
                t.transform.rotation.z = q[2]
                t.transform.rotation.w = q[3]

                br.sendTransform(t)

                i += 1

        if cv2.waitKey(27) & 0xFF == ord('q'):
            rospy.signal_shutdown('user command')

        #i = No.of tomatoes whose tfs are broadcasted
        return i


if __name__ == '__main__':
    ct = Controller()
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
