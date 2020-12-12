#! /usr/bin/env python

import rospy
import sys
import copy

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib

from pkg_vb_sim.srv import *
from hrwros_gazebo.msg import LogicalCameraImage

import tf2_ros
import tf2_msgs.msg


class CartesianPath:

    # Constructor
    def __init__(self):

        rospy.init_node('node_eg5_waypoints', anonymous=True)

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

        rospy.loginfo('\033[94m' + " >>> Init done." + '\033[0m')

        self.logical_camera_subscriber = rospy.Subscriber('eyrc/vb/logical_camera_2', LogicalCameraImage, self.update_camera)
        self.logical_camera = LogicalCameraImage()

        self.cam_y = -999

        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)

        self.tf_offset_x = 0
        self.tf_offset_y = 0
        self.tf_offset_z = 0
        

    def update_camera(self,data):
        self.logical_camera = data 
        if(len(self.logical_camera.models) == 1 and self.logical_camera.models[0].type != "ur5"):
            self.cam_y = round((self.logical_camera.models[0].pose.position.y),1)
        elif(len(self.logical_camera.models) > 1 and self.logical_camera.models[0].type == "ur5"):
            self.cam_y = round((self.logical_camera.models[1].pose.position.y),1)
        elif(len(self.logical_camera.models) > 1 and self.logical_camera.models[0].type != "ur5"):
            self.cam_y = round((self.logical_camera.models[0].pose.position.y),1)


    def func_get_tf(self, arg_frame_1, arg_frame_2):
        try:
            trans = self._tfBuffer.lookup_transform(arg_frame_1, arg_frame_2, rospy.Time())
            self.tf_translation_x = trans.transform.translation.x
            self.tf_translation_y = trans.transform.translation.y
            self.tf_translation_z = trans.transform.translation.z

            self.tf_offset_x = - self.tf_translation_z
            self.tf_offset_y = self.tf_translation_x
            self.tf_offset_z = - (self.tf_translation_y - 0.19)
            
            # rospy.loginfo(  "\n" +
            #                 "Translation: \n" +
            #                 "x: {} \n".format(trans.transform.translation.x) +
            #                 "y: {} \n".format(trans.transform.translation.y) +
            #                 "z: {} \n".format(trans.transform.translation.z) +
            #                 "\n" +
            #                 "Orientation: \n" +
            #                 "x: {} \n".format(trans.transform.rotation.x) +
            #                 "y: {} \n".format(trans.transform.rotation.y) +
            #                 "z: {} \n".format(trans.transform.rotation.z) +
            #                 "w: {} \n".format(trans.transform.rotation.w) )

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("TF error")


    def go_to_predefined_pose(self, arg_pose_name): # {"straightUp", "allZeros"}
        rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._group.set_named_target(arg_pose_name)
        plan = self._group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')
    

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


    def ee_cartesian_translation(self, trans_x, trans_y, trans_z):
        # 1. Create a empty list to hold waypoints
        waypoints = []

        # 2. Add Current Pose to the list of waypoints
        waypoints.append(self._group.get_current_pose().pose)

        # 3. Create a New waypoint
        wpose = geometry_msgs.msg.Pose()
        wpose.position.x = waypoints[0].position.x + (trans_x)  
        wpose.position.y = waypoints[0].position.y + (trans_y)  
        wpose.position.z = waypoints[0].position.z + (trans_z)
        # This to keep EE parallel to Ground Plane
        wpose.orientation.x = -0.5
        wpose.orientation.y = -0.5
        wpose.orientation.z = 0.5
        wpose.orientation.w = 0.5


        # 4. Add the new waypoint to the list of waypoints
        waypoints.append(copy.deepcopy(wpose))


        # 5. Compute Cartesian Path connecting the waypoints in the list of waypoints
        (plan, fraction) = self._group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # Step Size, distance between two adjacent computed waypoints will be 1 cm
            0.0)         # Jump Threshold
        rospy.loginfo("Path computed successfully. Moving the arm.")

        # The reason for deleting the first two waypoints from the computed Cartisian Path can be found here,
        # https://answers.ros.org/question/253004/moveit-problem-error-trajectory-message-contains-waypoints-that-are-not-strictly-increasing-in-time/?answer=257488#post-id-257488
        num_pts = len(plan.joint_trajectory.points)
        if (num_pts >= 3):
            del plan.joint_trajectory.points[0]
            del plan.joint_trajectory.points[1]

        # 6. Make the arm follow the Computed Cartesian Path
        self._group.execute(plan)

    
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


    def conveyor_power(self, arg_power):  # {0, [11,100]}
        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        conveyor_service = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power',conveyorBeltPowerMsg)
        conveyor_obj = conveyorBeltPowerMsgRequest()
        conveyor_obj.power = arg_power
        result = conveyor_service(conveyor_obj)
        print(result)


    def activate_gripper(self, option):  # {True, False}
        rospy.wait_for_service('/eyrc/vb/ur5_1/activate_vacuum_gripper')
        gripper_service = rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper',vacuumGripper)
        gripper_obj = vacuumGripperRequest()
        gripper_obj.activate_vacuum_gripper = option
        result = gripper_service(gripper_obj)
        print(result)
    

    def __del__(self):  # Destructor
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class CartesianPath Deleted." + '\033[0m')


def main():
    ur5 = CartesianPath()

    box_length = 0.15               # Length of the Package
    vacuum_gripper_width = 0.115    # Vacuum Gripper Width
    delta = vacuum_gripper_width + (box_length/2)  # 0.19
    # Teams may use this info in Tasks

    ur5_2_home_pose = geometry_msgs.msg.Pose()
    ur5_2_home_pose.position.x = -0.8
    ur5_2_home_pose.position.y = 0
    ur5_2_home_pose.position.z = 1 + vacuum_gripper_width + (box_length/2)
    # This to keep EE parallel to Ground Plane
    ur5_2_home_pose.orientation.x = -0.5
    ur5_2_home_pose.orientation.y = -0.5
    ur5_2_home_pose.orientation.z = 0.5
    ur5_2_home_pose.orientation.w = 0.5

    ur5.conveyor_power(30)
    ur5.go_to_pose(ur5_2_home_pose)

    while not rospy.is_shutdown():
        if(ur5.cam_y == 0.0):
            ur5.conveyor_power(0)
            rospy.loginfo("Red package detected")
            break
    
    ur5.activate_gripper(True)
    ur5.ee_cartesian_translation(0.8, 0.5, 0)
    ur5.activate_gripper(False)

    ur5.conveyor_power(20)
    ur5.go_to_pose(ur5_2_home_pose)

    while not rospy.is_shutdown():
        if(ur5.cam_y == 0.0):
            ur5.conveyor_power(0)
            rospy.loginfo("Green package detected")
            break
    ur5.func_get_tf("ur5_wrist_3_link","logical_camera_2_packagen2_frame")
    ur5.ee_cartesian_translation(ur5.tf_offset_x, ur5.tf_offset_y, ur5.tf_offset_z)

    ur5.activate_gripper(True)
    ur5.ee_cartesian_translation(0.8, 0.5, 0)
    ur5.ee_cartesian_translation(0.8, -0.5, 0)

    ur5.activate_gripper(False)

    ur5.conveyor_power(20)
    ur5.go_to_pose(ur5_2_home_pose)

    while not rospy.is_shutdown():
        if(ur5.cam_y == 0.0):
            ur5.conveyor_power(0)
            rospy.loginfo("Blue package detected")
            break
    ur5.func_get_tf("ur5_wrist_3_link","logical_camera_2_packagen3_frame")
    ur5.ee_cartesian_translation(ur5.tf_offset_x, ur5.tf_offset_y, ur5.tf_offset_z)

    ur5.activate_gripper(True)
    ur5.ee_cartesian_translation(0.8, -0.5, 0)
    ur5.activate_gripper(False)

    del ur5


if __name__ == '__main__':
    main()
