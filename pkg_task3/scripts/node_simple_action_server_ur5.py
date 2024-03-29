#! /usr/bin/env python

import rospy
import sys
import copy

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math

from pkg_vb_sim.srv import *

from pkg_task3.msg import myActionMsgAction
from pkg_task3.msg import myActionMsgGoal
from pkg_task3.msg import myActionMsgResult
from pkg_task3.msg import myActionMsgFeedback


class SimpleActionServerUr5:

    def __init__(self):
        # Initialize Simple Action Server
        self._sas = actionlib.SimpleActionServer('/action_ur5',
                                                 myActionMsgAction,
                                                 execute_cb=self.func_on_rx_goal,
                                                 auto_start=False)
        self._planning_group = "ur5_1_planning_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        # Declare constants
        box_length = 0.15               # Length of the Package
        vacuum_gripper_width = 0.115    # Vacuum Gripper Width
        delta = vacuum_gripper_width + (box_length/2)  # 0.19

        self.ur5_2_home_pose = geometry_msgs.msg.Pose()
        self.ur5_2_home_pose.position.x = -0.8
        self.ur5_2_home_pose.position.y = 0
        self.ur5_2_home_pose.position.z = 1 + vacuum_gripper_width + (box_length/2)
        self.ur5_2_home_pose.orientation.x = -0.5
        self.ur5_2_home_pose.orientation.y = -0.5
        self.ur5_2_home_pose.orientation.z = 0.5
        self.ur5_2_home_pose.orientation.w = 0.5

        self.lst_joint_angles = [math.radians(6),
                          math.radians(-137),
                          math.radians(-63),
                          math.radians(-66),
                          math.radians(90),
                          math.radians(0)]
        self.lst_joint_angles_red = [math.radians(-80),
                          math.radians(-120),
                          math.radians(-63),
                          math.radians(-66),
                          math.radians(90),
                          math.radians(0)]
        self.lst_joint_angles_green = [math.radians(-10),
                          math.radians(-45),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0),
                          math.radians(0)]
        self.lst_joint_angles_blue = [math.radians(90),
                          math.radians(-120),
                          math.radians(-63),
                          math.radians(-66),
                          math.radians(90),
                          math.radians(0)]
        
        # Start the Action Server
        self._sas.start()
        rospy.loginfo("Started Turtle Simple Action Server.")
        

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
            rospy.loginfo('\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan
    

    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan
    
    def activate_gripper(self, option):  # True or False
        rospy.wait_for_service('/eyrc/vb/ur5_1/activate_vacuum_gripper')
        gripper_service = rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper',vacuumGripper)
        gripper_obj = vacuumGripperRequest()
        gripper_obj.activate_vacuum_gripper = option
        result = gripper_service(gripper_obj)
        print(result)
    

    # Function to process Goals and send Results
    def func_on_rx_goal(self, obj_msg_goal):
        rospy.loginfo("Received a Goal from Client.")
        rospy.loginfo(obj_msg_goal)

        flag_success = False        # Set to True if Goal is successfully achieved
        flag_preempted = False      # Set to True if Cancel req is sent by Client

        # --- Goal Processing Section ---
        if(obj_msg_goal.destination == "home_use_pose"):
            self.go_to_pose(self.ur5_2_home_pose)
        elif(obj_msg_goal.destination == "home_use_joint_angles"):
            self.set_joint_angles(self.lst_joint_angles)
        elif(obj_msg_goal.destination == "red_basket_use_joint_angles"):
            self.set_joint_angles(self.lst_joint_angles_red)
            self.activate_gripper(False)
            self.set_joint_angles(self.lst_joint_angles)
        elif(obj_msg_goal.destination == "green_basket_use_joint_angles"):
            self.set_joint_angles(self.lst_joint_angles_green)
            self.activate_gripper(False)
            self.set_joint_angles(self.lst_joint_angles)
        elif(obj_msg_goal.destination == "blue_basket_use_joint_angles"):
            self.set_joint_angles(self.lst_joint_angles_blue)
            self.activate_gripper(False)
            self.set_joint_angles(self.lst_joint_angles)

        # Send Result to the Client
        obj_msg_result = myActionMsgResult()
        obj_msg_result.complete = True
        
        rospy.loginfo("Send goal result to client")
        self._sas.set_succeeded(obj_msg_result)

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class CartesianPath Deleted." + '\033[0m')
    

# Main Function
def main():
    rospy.init_node('node_simple_action_server_ur5')

    ur5 =SimpleActionServerUr5()

    rospy.spin()


if __name__ == '__main__':
    main()
