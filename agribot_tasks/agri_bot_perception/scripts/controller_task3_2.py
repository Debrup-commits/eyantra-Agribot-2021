#! /usr/bin/env python3

import actionlib
import copy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import geometry_msgs.msg
import math
import message_filters
import moveit_commander
import moveit_msgs.msg
import numpy as np
import rospy
from sensor_msgs.msg import Image
import sys
import tf2_ros
import tf_conversions
import tf
from tf.transformations import euler_from_quaternion


class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('node_eg3_set_joint_angles', anonymous=True)

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

        self._box_name = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        #camera feed
        self.image_sub = message_filters.Subscriber("/camera/color/image_raw2", Image)
        self.depth_sub = message_filters.Subscriber("/camera/depth/image_raw2", Image)

        self.rgb_frame = None
        self.depth_frame = None

        self.bridge = CvBridge()

        #synchronizes depth and rgb camera feed
        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.depth_sub], queue_size=10, slop=0.5)
        self.ts.registerCallback(self.image_callback)

        #listens to the broadcasted tf of the tomatoes
        listener = tf.TransformListener()


        while not rospy.is_shutdown():
            #rotates arm for camera to be able to detect tomatoes to be plucked
            self.go_to_predefined_pose("rotate_90_2")

            #broadcasts and returns number of tomatoes to be plucked
            no_of_tomatoes = self.broadcast_tomato_coordinates(self.rgb_frame, self.depth_frame)

            #msg to be used to give arm the info of coordinates it has to move to
            ur5_pose = geometry_msgs.msg.Pose()
            try:
                for i in range(no_of_tomatoes):
                    #gives the final converted tfs where the arm has to go
                    trans, rot = listener.lookupTransform(target_frame='/ebot_base', source_frame='/obj0', time=rospy.Time(0))

                    #arm movement
                    self.take_to_pose(ur5_pose, trans, 0.4)
                    self.take_to_pose(ur5_pose, trans, 0.25)

                    self.go_to_predefined_pose("close")
                    self.go_to_predefined_pose("home")
                    self.go_to_predefined_pose("open")

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            rospy.loginfo("task3_2 completed")
            rospy.signal_shutdown("task done")

    def go_to_predefined_pose(self, arg_pose_name):

        if arg_pose_name == "open" or arg_pose_name == "close":
            self._group_2.set_named_target(arg_pose_name)
            self._group_2.plan()
            flag_plan = self._group_2.go(wait=True)
        else:
             self._group_1.set_named_target(arg_pose_name)
             self._group_1.plan()
             flag_plan = self._group_1.go(wait=True)

    def go_to_pose(self, arg_pose):

        pose_values = self._group_1.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        self._group_1.set_pose_target(arg_pose)
        flag_plan = self._group_1.go(wait=True)  # wait=False for Async Move

        pose_values = self._group_1.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group_1.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

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

    def broadcast_tomato_coordinates(self, rgb_frame, depth_frame):
        # Initializing variables
        focal_length = 554.387
        center_x = 320.5
        center_y = 240.5

        img_HSV = cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2HSV)

        low_HSV = np.array([0, 53, 137])
        upper_HSV = np.array([10, 255, 255])

        mask = cv2.inRange(img_HSV, low_HSV, upper_HSV)
        output = cv2.bitwise_and(rgb_frame, rgb_frame, mask = mask)
        output = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)

        ret, threshold = cv2.threshold(output, 25, 255, cv2.THRESH_BINARY)

        contours, hierarchy = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        i=0

        for contour in contours:
            C = cv2.moments(contour)

            cX = int(C['m10']/(C['m00']+1e-4))
            cY = int(C['m01']/(C['m00']+1e-4))
            depth = depth_frame[cY][cX]

            # transforming pixel coordinates to world coordinates
            world_x = (cX - center_x)/focal_length*depth
            world_y = (cY - center_y)/focal_length*depth
            world_z = depth

            # broadcasting TF for each tomato coordinate
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

            cv2.putText(rgb_frame, 'obj{}'.format(i),(cX, cY-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            cv2.circle(rgb_frame, (cX, cY), 2, (255, 255, 255), -1)
            cv2.circle(depth_frame, (cX, cY), 2, (0, 255, 0), -1)

            i += 1

        cv2.imshow("frame", rgb_frame)
        cv2.waitKey(1)

        return len(contours)


    def image_callback(self, rgb_data, depth_data):

        try:
            self.rgb_frame = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")
            self.depth_frame = self.bridge.imgmsg_to_cv2(depth_data, "32FC1")

        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    ur5 = Ur5Moveit()
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
