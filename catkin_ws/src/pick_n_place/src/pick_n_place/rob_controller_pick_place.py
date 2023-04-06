#!/usr/bin/env python3.8

import copy
import sys
from math import pi, dist, fabs, sin, cos

import moveit_commander
import rospy
import tf
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Point, Pose, PoseStamped
from moveit_msgs.msg import Constraints, JointConstraint, OrientationConstraint
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory
# from pilz_robot_programming import *

# todo: add comments
class RobControllerPickPlace:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('rob_controller')
        rospy.Rate(500)

        rospy.on_shutdown(self.cleanup)
        rospy.wait_for_service('/gazebo/get_model_state')
        # wait for move_group loading
        rospy.wait_for_service('/compute_cartesian_path')
        self.get_gazebo_model_state = rospy.ServiceProxy('/gazebo/get_model_state/', GetModelState)

        self.gripper_pub = rospy.Publisher('/gripper_joint_position/command', Float64, queue_size=10)
        self.joint_traj_pub = rospy.Publisher('/pos_joint_traj_controller/command', JointTrajectory, queue_size=10)
        self.tf = tf.TransformListener()

        self.robot = moveit_commander.RobotCommander()
        rospy.sleep(3)
        self.arm = self.robot.manipulator
        self.arm.set_start_state(self.robot.get_current_state())

        # planning setting
        self.arm.allow_replanning(True)
        self.arm.set_goal_position_tolerance(0.005)
        self.arm.set_goal_orientation_tolerance(0.01)
        self.arm.set_planning_time(10)
        self.arm.set_max_acceleration_scaling_factor(.8)
        self.arm.set_max_velocity_scaling_factor(.8)

        self.joint_names = self.arm.get_active_joints()

        # key joint poses for main loop trajectories
        self.home_pose_j = [0, - 0.5 * pi, 0.5 * pi, -0.5 * pi, -0.5 * pi, 0.5 * pi]
        self.above_table1_pose_j = [-0.3 * pi, - 0.5 * pi, 0.5 * pi, -0.5 * pi, -0.5 * pi, 0.2 * pi]
        self.above_table2_pose_j = [0.3 * pi, - 0.5 * pi, 0.5 * pi, -0.5 * pi, -0.5 * pi, 0.8 * pi]

        # joint constraints: base rotate only forward, shoulder forward_up, elbow up, wrist 1 down, wrist 2 straight down
        self._joints_above = [3, 2, 3, 3, 0.5, 6.28]
        self._joint_below = [1, 1, 2, 3, 0.5, 6.28]

        # world coordinates of robot and table2
        self.gazebo_world_robot_position = self.get_robot_position()
        self.gazebo_first_place_position = Point(0, 0.6, 0.755)  # self.get_table2_position()

        # init pick_n_place items variables
        self._cur_block_num = 0
        self.blocks_count = 5
        self.next_block_pick_pose = None
        self.next_block_place_pose = None

        # make robot ready
        self.open_gripper()
        self.go_home()
        self.look_down_ori = copy.deepcopy(self.arm.get_current_pose().pose.orientation)

        # turn off joint constraints, cause it slow down planners (need time to check)
        self.path_constraints = self.make_arm_path_constraints(constraint_ee_ori_by_current=True,
                                                               constraint_joints=False)
        self.arm.set_path_constraints(self.path_constraints)

    def make_arm_path_constraints(self, constraint_ee_ori_by_current=True, constraint_joints=True):
        constraints = Constraints()
        constraints.name = "lookdown"

        if constraint_ee_ori_by_current:
            _ee_name = self.arm.get_end_effector_link()
            ee_ori = self.arm.get_current_pose(_ee_name).pose.orientation

            orientation_constraint = OrientationConstraint()
            orientation_constraint.header.frame_id = self.arm.get_current_pose().header.frame_id
            orientation_constraint.link_name = self.arm.get_end_effector_link()
            orientation_constraint.orientation = copy.deepcopy(ee_ori)
            orientation_constraint.absolute_x_axis_tolerance = 6.28  # gripper x is wrist3 z
            orientation_constraint.absolute_y_axis_tolerance = 0.3
            orientation_constraint.absolute_z_axis_tolerance = 0.3
            orientation_constraint.weight = 1
            constraints.orientation_constraints.append(orientation_constraint)

        jn_tmp = self.robot.get_current_state().joint_state.name
        jp_tmp = self.robot.get_current_state().joint_state.position
        if constraint_joints:
            for i in range(len(jn_tmp)):  # self.joint_names)):
                joint_constraint = JointConstraint()
                joint_constraint.joint_name = jn_tmp[i]  # self.joint_names[i]
                joint_constraint.position = jp_tmp[i]  # self.home_pose[i]

                try:
                    active_i = self.joint_names.index(jn_tmp[i])
                    above = self._joints_above[active_i]
                    below = self._joint_below[active_i]
                except:
                    above, below = 6, 6
                joint_constraint.tolerance_above = above
                joint_constraint.tolerance_below = below
                joint_constraint.weight = 1
                constraints.joint_constraints.append(joint_constraint)

        return constraints

    def try_get_next_block(self):
        if self._cur_block_num >= self.blocks_count:
            return False
        self._cur_block_num += 1
        self.next_block_pick_pose = self.get_block_pose(self._cur_block_num)
        self.next_block_place_pose = self.get_next_block_target()
        return True

    def get_next_block_target(self):
        block_z_offset = 0.037
        next_block_target_pose = Pose(copy.deepcopy(self.gazebo_first_place_position), self.look_down_ori)
        next_block_target_pose.position.x = self.gazebo_first_place_position.x + 0.2 * ((self._cur_block_num - 1) % 2)
        next_block_target_pose.position.y = self.gazebo_first_place_position.y - 0.2 + 0.1 * (
                (self._cur_block_num - 1) // 2)
        next_block_target_pose.position.z = self.gazebo_first_place_position.z - self.gazebo_world_robot_position.z + block_z_offset

        # todo: try some orientation patterns =)
        # circle_part = self._cur_block_num / self.blocks_count
        # q_rot = [0, 0, sin(pi * circle_part), cos(pi * circle_part)]
        # q_orig = [self.look_down_ori.x, self.look_down_ori.y, self.look_down_ori.z, self.look_down_ori.w]
        #
        # next_block_target_pose.orientation = Quaternion(*tf.transformations.quaternion_multiply(q_rot, q_orig))

        return next_block_target_pose

    def get_block_pose(self, block_num):
        try:
            gazebo_world_block_pose = self.get_gazebo_model_state('block_' + str(block_num), None).pose
            block_pose_for_arm = gazebo_world_block_pose
            block_pose_for_arm.position.z -= self.gazebo_world_robot_position.z
            return block_pose_for_arm
        except rospy.ServiceException as e:
            print("Service '/gazebo/get_model_state' did not process request: " + str(e))
            return None

    def get_robot_position(self):
        try:
            return self.get_gazebo_model_state('robot', None).pose.position
        except rospy.ServiceException as e:
            print("Service '/gazebo/get_model_state' did not process request: " + str(e))
            return None

    def get_table2_position(self):
        try:
            res = self.get_gazebo_model_state('cafe_table_2', 'surface')

            return res.pose.position
        except rospy.ServiceException as e:
            print("Service '/gazebo/get_model_state' did not process request: " + str(e))
            return None

    def all_close(self, goal, actual, tolerance=0.01):
        """
        Convenience method for testing if the values in two lists are within a tolerance of each other.
        For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
        between the identical orientations q and -q is calculated correctly).
        @param: goal       A list of floats, a Pose or a PoseStamped
        @param: actual     A list of floats, a Pose or a PoseStamped
        @param: tolerance  A float
        @returns: bool
        """
        if type(goal) is list:
            for index in range(len(goal)):
                if abs(actual[index] - goal[index]) > tolerance:
                    return False

        elif type(goal) is PoseStamped:
            return self.all_close(goal.pose, actual.pose, tolerance)

        elif type(goal) is Pose:
            x0, y0, z0, qx0, qy0, qz0, qw0 = moveit_commander.conversions.pose_to_list(actual)
            x1, y1, z1, qx1, qy1, qz1, qw1 = moveit_commander.conversions.pose_to_list(goal)
            # Euclidean distance
            d = dist((x1, y1, z1), (x0, y0, z0))
            # phi = angle between orientations
            cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
            return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

        return True

    def go_home(self):
        if self.arm is None:
            return False
        self.arm.clear_pose_targets()
        success = self.arm.go(self.home_pose_j, wait=True)
        self.arm.stop()
        return success

    def open_gripper(self, wait_time=0.5):
        self.gripper_pub.publish(Float64(0.1))
        rospy.sleep(wait_time)

    def close_gripper(self, wait_time=0.5):
        self.gripper_pub.publish(Float64(-0.3))
        rospy.sleep(wait_time)

    def pick_n_place(self, block_pick_pose, block_place_pose, approach_z=0.06, deapproach_z=0.06):
        if self.arm is None:
            return False
        cur_pose = copy.deepcopy(self.arm.get_current_pose().pose)

        pick_p = copy.deepcopy(block_pick_pose)
        pick_p.orientation = copy.deepcopy(cur_pose.orientation)

        appr_pick_p = copy.deepcopy(pick_p)
        appr_pick_p.position.z += approach_z

        deappr_pick_p = copy.deepcopy(pick_p)
        deappr_pick_p.position.z += deapproach_z

        place_p = copy.deepcopy(block_place_pose)
        place_p.orientation = self.look_down_ori

        appr_place_p = copy.deepcopy(place_p)
        appr_place_p.position.z += approach_z
        appr_place_p.orientation = copy.deepcopy(cur_pose.orientation)

        deappr_place_p = copy.deepcopy(place_p)
        deappr_place_p.position.z += deapproach_z
        deappr_place_p.orientation = copy.deepcopy(cur_pose.orientation)

        self.open_gripper(0)
        pathes_n_actions = [
            {"path": [self.above_table1_pose_j, appr_pick_p, pick_p],
             "action": self.close_gripper,
             "move_type": "movej"},
            {"path": [deappr_pick_p, self.above_table1_pose_j, self.home_pose_j, self.above_table2_pose_j, appr_place_p,
                      place_p],
             "action": self.open_gripper,
             "move_type": "movej"},
            {"path": [deappr_place_p, self.above_table2_pose_j, self.home_pose_j],
             "action": None,
             "move_type": "movej"}
        ]

        for path_n_action in pathes_n_actions:
            if path_n_action["move_type"] in ["movej"]:
                # movej
                for goal in path_n_action["path"]:
                    success = self.arm.go(goal, wait=True)
                    if not success:
                        print("Can't find the path, please check configuration")
                        self.open_gripper()
                        self.go_home()
                        return False
            else:
                # movel, cartesian
                (plan, fraction) = self.arm.compute_cartesian_path(
                    path_n_action["path"],
                    0.05,  # eef_step
                    0.0,
                    avoid_collisions=True,
                    path_constraints=self.path_constraints)  # jump_threshold

                success = False
                if fraction > 0.9:
                    self.joint_traj_pub.publish(plan.joint_trajectory)
                    # success = self.arm.execute(plan, wait=True)
                    while not self.all_close(self.arm.get_current_pose().pose, path_n_action["path"][-1]):
                        rospy.sleep(10.)
                    # self.arm.stop()
                if not success:
                    print("low fraction")
                    for goal in path_n_action["path"]:
                        (plan, fraction) = self.arm.compute_cartesian_path(
                            [copy.deepcopy(self.arm.get_current_pose().pose), goal],
                            0.01,  # eef_step
                            0.0)  # jump_threshold

                        if fraction > 0.9:
                            print("fraction is ok")
                            # success = self.arm.execute(plan, wait=True)
                            self.joint_traj_pub.publish(plan.joint_trajectory)
                            while not self.all_close(self.arm.get_current_pose().pose, goal):
                                rospy.sleep(10.)
                            # self.joint_traj_pub.publish(JointTrajectory())
                            # self.arm.stop()
                        else:
                            print("low fraction, try movej")
                            self.arm.go(goal, wait=True)

            if path_n_action["action"] is not None:
                path_n_action["action"]()
        return True

    def main_loop(self):
        while self.try_get_next_block():
            if self.pick_n_place(self.next_block_pick_pose, self.next_block_place_pose):
                print("Pick n place block ", self._cur_block_num, " is done")
        print("Task is done")

    def cleanup(self):
        rospy.loginfo("Stopping the robot")
        # Stop any current arm movement
        self.arm.stop()
        # Shut down MoveIt! cleanly
        rospy.loginfo("Shutting down Moveit!")
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


if __name__ == "__main__":
    try:
        RobControllerPickPlace().main_loop()
    except KeyboardInterrupt:
        print("Exit")
