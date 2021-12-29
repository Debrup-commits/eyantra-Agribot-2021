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

        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.depth_sub], queue_size=10, slop=0.5)
        self.ts.registerCallback(self.image_callback)

        self.bridge = CvBridge()

        while not rospy.is_shutdown():
            self.go_to_predefined_pose("rotate_90_2")
            moveit_commander.roscpp_shutdown()

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

    def go_to_pose(self, arg_pose):

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    def image_callback(self, rgb_data, depth_data):

        # Initializing variables
        focal_length = 554.387
        center_x = 320.5
        center_y = 240.5

        try:
            frame = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")
            depth_frame = self.bridge.imgmsg_to_cv2(depth_data, "32FC1")
            depth_array = np.array(depth_frame, dtype=np.float32)

            img_HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            low_HSV = np.array([0, 53, 137])
            upper_HSV = np.array([10, 255, 255])

            mask = cv2.inRange(img_HSV, low_HSV, upper_HSV)
            output = cv2.bitwise_and(frame, frame, mask = mask)
            output = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)

            ret, threshold = cv2.threshold(output, 25, 255, cv2.THRESH_BINARY)

            contours, hierarchy = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            # cv2.drawContours(frame, contours, -1, (255, 255, 255), 2)

            i=0

            for contour in contours:
                C = cv2.moments(contour)

                cX = int(C['m10']/(C['m00']+1e-4))
                cY = int(C['m01']/(C['m00']+1e-4))
                depth = depth_frame[cY][cX]

                # transforming pixel coordinates to world coordinates
                if depth < 1:
                    world_x = (cX - center_x)/focal_length*depth
                    world_y = (cY - center_y)/focal_length*depth
                    world_z = depth

                    # broadcasting TF for each tomato coordinate
                    br = tf2_ros.TransformBroadcaster()
                    t = geometry_msgs.msg.TransformStamped()
                    t.header.stamp = rospy.Time.now()
                    t.header.frame_id = "camera_link2"
                    t.child_frame_id = "obj"+str(i)

                    # putting world coordinates coordinates as viewed for sjcam frame
                    t.transform.translation.x = world_z
                    t.transform.translation.y = -world_x
                    t.transform.translation.z = world_y

                    # not extracting any orientation thus orientation is (0, 0, 0)
                    q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
                    t.transform.rotation.x = q[0]
                    t.transform.rotation.y = q[1]
                    t.transform.rotation.z = q[2]
                    t.transform.rotation.w = q[3]

                    br.sendTransform(t)

                cv2.putText(frame, 'obj{}'.format(i),(cX, cY-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.circle(frame, (cX, cY), 2, (255, 255, 255), -1)
                cv2.circle(depth_frame, (cX, cY), 2, (0, 255, 0), -1)

                i+=1

            cv2.imshow("frame_2", frame)
            # cv2.imshow("depth_frame", depth_frame)
            cv2.waitKey(1)

        except CvBridgeError as e:
            print(e)

    # Destructor
    # def __del__(self):
    #     moveit_commander.roscpp_shutdown()
    #     rospy.loginfo(
    #         '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


# def main():
#
#     ur5 = Ur5Moveit()

    # ur5_pose_1 = geometry_msgs.msg.Pose()
    # ur5_pose_1.position.x = -0.817261772949
    # ur5_pose_1.position.y = -0.109110076352
    # ur5_pose_1.position.z = 0.94446979642
    # ur5_pose_1.orientation.x = -0.999999995957
    # ur5_pose_1.orientation.y = 4.37354574363e-05
    # ur5_pose_1.orientation.z = 7.85715579538e-05
    # ur5_pose_1.orientation.w = 2.12177767514e-09
    #
    # ur5_pose_2 = geometry_msgs.msg.Pose()
    # ur5_pose_2.position.x = -0.414925357653
    # ur5_pose_2.position.y = 0.284932768677
    # ur5_pose_2.position.z = 1.78027849967
    # ur5_pose_2.orientation.x = -0.199396929724
    # ur5_pose_2.orientation.y = 1.64394297608e-05
    # ur5_pose_2.orientation.z = 0.979918803013
    # ur5_pose_2.orientation.w = 6.03911583936e-05
    #
    # ur5_pose_3 = geometry_msgs.msg.Pose()
    # ur5_pose_3.position.x = 0.061218702528
    # ur5_pose_3.position.y = 0.150917431354
    # ur5_pose_3.position.z = 1.20083763657
    # ur5_pose_3.orientation.x = 0.635613875737
    # ur5_pose_3.orientation.y = 0.77190802743
    # ur5_pose_3.orientation.z = 0.00233308772292
    # ur5_pose_3.orientation.w = 0.0121472162087

    # while not rospy.is_shutdown():
    #     ur5.go_to_pose(ur5_pose_1)
    #     rospy.sleep(2)
    #     ur5.go_to_pose(ur5_pose_2)
    #     rospy.sleep(2)
    #     ur5.go_to_pose(ur5_pose_3)
    #     rospy.sleep(2)

    # del ur5


if __name__ == '__main__':
    ur5 = Ur5Moveit()
    try:
        if not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        print(e)
