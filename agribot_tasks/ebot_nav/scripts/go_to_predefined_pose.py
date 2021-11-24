#! /usr/bin/env python3

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib

class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('go_to_predefined_pose', anonymous=True)

        self._planning_group_1 = "arm"
        self._planning_group_2 = "gripper"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()

        self._group_1 = moveit_commander.MoveGroupCommander(self._planning_group_1)
        self._group_2 = moveit_commander.MoveGroupCommander(self._planning_group_2)

        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame_1 = self._group_1.get_planning_frame()
        self._planning_frame_2 = self._group_2.get_planning_frame()

        self._eef_link_1 = self._group_1.get_end_effector_link()
        self._eef_link_2 = self._group_2.get_end_effector_link()

        self._group_names = self._robot.get_group_names()


        rospy.loginfo(
            '\033[94m' + "Planning Group 1: {}".format(self._planning_frame_1) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link 1: {}".format(self._eef_link_1) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Planning Group 2: {}".format(self._planning_frame_2) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link 2: {}".format(self._eef_link_2) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

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


    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

    ur5 = Ur5Moveit()

    while not rospy.is_shutdown():
        ur5.go_to_predefined_pose("go_to_tomato1")
        ur5.go_to_predefined_pose("close")
        ur5.go_to_predefined_pose("home")
        ur5.go_to_predefined_pose("open")

        ur5.go_to_predefined_pose("go_to_tomato2")
        ur5.go_to_predefined_pose("close")
        ur5.go_to_predefined_pose("home")
        ur5.go_to_predefined_pose("open")

        ur5.go_to_predefined_pose("go_to_tomato3")
        ur5.go_to_predefined_pose("close")
        ur5.go_to_predefined_pose("home")
        ur5.go_to_predefined_pose("open")

        del ur5

        rospy.signal_shutdown("Task completed")

if __name__ == '__main__':
    main()
