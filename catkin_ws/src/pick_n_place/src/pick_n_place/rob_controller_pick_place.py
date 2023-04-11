#!/usr/bin/env python3.8

import copy
import sys
from math import pi, sin, cos
from typing import List

import moveit_commander
import numpy as np
import rospy
import tf
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.msg import Constraints, JointConstraint, OrientationConstraint, RobotTrajectory, RobotState
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

# todo: compare with pilz
# from pilz_robot_programming import *

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
        # self.joint_traj_pub = rospy.Publisher('/pos_joint_traj_controller/command', JointTrajectory, queue_size=10)
        self.tr = tf.listener.TransformerROS()

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

    ### GAZEBO world section ###
    def try_get_next_block(self):
        if self._cur_block_num >= self.blocks_count:
            return False
        self._cur_block_num += 1
        # rotate target gripper pose to block yaw rotation
        next_block_pose = self.get_block_pose(self._cur_block_num)
        q_block = [next_block_pose.orientation.x, next_block_pose.orientation.y,
                   next_block_pose.orientation.z, next_block_pose.orientation.w]
        (r, p, y) = tf.transformations.euler_from_quaternion(q_block)
        q_gripper_def = [self.look_down_ori.x,
                         self.look_down_ori.y,
                         self.look_down_ori.z,
                         self.look_down_ori.w]
        q_rot = tf.transformations.quaternion_from_euler(0, 0, y)
        q_gripper_new = Quaternion(*tf.transformations.quaternion_multiply(q_rot, q_gripper_def))

        self.next_block_pick_pose = Pose(next_block_pose.position, q_gripper_new)
        self.next_block_place_pose = self.get_next_block_place_pose()
        return True

    def get_next_block_place_pose(self, circle_pattern=True):
        block_z_offset = 0.037
        next_block_target_pose = copy.deepcopy(Pose(self.gazebo_first_place_position, self.look_down_ori))

        next_block_target_pose.position.z = self.gazebo_first_place_position.z - self.gazebo_world_robot_position.z + block_z_offset

        if circle_pattern:
            circle_part = self._cur_block_num / self.blocks_count
            q_rot = [0, 0, sin(pi * circle_part), cos(pi * circle_part)]
            transfmat = self.tr.fromTranslationRotation([next_block_target_pose.position.x,
                                                         next_block_target_pose.position.y,
                                                         next_block_target_pose.position.z], q_rot)
            posemat = self.tr.fromTranslationRotation([0., 0.1, 0],
                                                      [next_block_target_pose.orientation.x,
                                                       next_block_target_pose.orientation.y,
                                                       next_block_target_pose.orientation.z,
                                                       next_block_target_pose.orientation.w])
            newmat = np.dot(transfmat, posemat)
            xyz = tuple(newmat[:3, 3])
            quat = tuple(tf.transformations.quaternion_from_matrix(newmat))

            next_block_target_pose = Pose(Point(*xyz), Quaternion(*quat))
        else:
            # square pattern
            next_block_target_pose.position.x = self.gazebo_first_place_position.x + 0.2 * (
                    (self._cur_block_num - 1) % 2)
            next_block_target_pose.position.y = self.gazebo_first_place_position.y - 0.2 + 0.1 * (
                    (self._cur_block_num - 1) // 2)

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

    ### SIMPLE ACTIONS
    def open_gripper(self, wait_time=0.5):
        self.gripper_pub.publish(Float64(0.1))
        rospy.sleep(wait_time)

    def close_gripper(self, wait_time=0.5):
        self.gripper_pub.publish(Float64(-0.3))
        rospy.sleep(wait_time)

    def go_home(self):
        if self.arm is None:
            return False
        self.arm.clear_pose_targets()
        success = self.arm.go(self.home_pose_j, wait=True)
        self.arm.stop()
        self.arm.set_start_state_to_current_state()
        return success

    ### PLANNING HELPER METHODS
    def robot_state_from_pose(self, pose):
        js = JointState()
        js.name = self.joint_names
        js.position = pose
        moveit_robot_state = RobotState()
        moveit_robot_state.joint_state = js
        return moveit_robot_state

    def append_points_to_traj(self, traj1: RobotTrajectory, traj2: RobotTrajectory):
        if len(traj1.joint_trajectory.points) == 0:
            traj1 = traj2
        else:
            traj1.joint_trajectory.points += traj2.joint_trajectory.points
        return traj1

    ### PLANNER METHODS
    def mixed_traj_j(self, waypoints_list, execute=True):
        # waypoints list can be mixed (points or poses)
        # method calc and execute trajectory if execute == True
        # start state must be correctly setted!
        if execute:
            cur_rob_state = self.robot.get_current_state()
            self.arm.set_start_state(cur_rob_state)

        concatenated_plan = RobotTrajectory()
        for p in waypoints_list:
            if type(p) is Pose:
                (success, plan, time, err) = self.arm.plan(p)
                if not success:
                    return (False, None)
                moveit_robot_state = self.robot_state_from_pose(plan.joint_trajectory.points[-1].positions)
            else:
                # list of joint positions
                js = JointState()
                js.name = self.joint_names
                js.position = p
                (success, plan, time, err) = self.arm.plan(js)
                if not success:
                    return (False, None)
                moveit_robot_state = RobotState()
                moveit_robot_state.joint_state = js

            self.arm.set_start_state(moveit_robot_state)
            concatenated_plan = self.append_points_to_traj(concatenated_plan, plan)

        if execute:
            time_optimized_plan = self.arm.retime_trajectory(cur_rob_state, concatenated_plan,
                                                             algorithm="time_optimal_trajectory_generation")
            success = self.arm.execute(time_optimized_plan, wait=True)
            return (success, time_optimized_plan)

        return (success, concatenated_plan)

    def movel(self, goal_waypoints: List[Pose], execute=True):
        if execute:
            cur_rob_state = self.robot.get_current_state()
            self.arm.set_start_state(cur_rob_state)

        (plan, fraction) = self.arm.compute_cartesian_path(
            goal_waypoints,
            0.01,  # eef_step
            0.0,  # jump_threshold
            avoid_collisions=True,
            path_constraints=self.path_constraints)
        moveit_robot_state = self.robot_state_from_pose(plan.joint_trajectory.points[-1].positions)
        self.arm.set_start_state(moveit_robot_state)
        success = False
        if fraction > 0.9:
            success = True
            if execute:
                plan = self.arm.retime_trajectory(cur_rob_state, plan,
                                                  algorithm="time_optimal_trajectory_generation")
                success = self.arm.execute(plan, wait=True)
        return (success, plan)

    def execute_pathes_n_actions(self, pathes_n_actions):
        # concat waypoints while action is None
        # execute concatenated path and action when action is not None
        concatenated_plan = RobotTrajectory()
        for sub_path_act in pathes_n_actions:
            if sub_path_act["move_type"] == "movel":
                (success, plan) = self.movel(sub_path_act["path"], False)
            else:
                (success, plan) = self.mixed_traj_j(sub_path_act["path"], False)
            if not success:
                print("Can't plan path for block ", self._cur_block_num)
                return False
            concatenated_plan = self.append_points_to_traj(concatenated_plan, plan)
            if sub_path_act["action"] is not None:
                cur_rob_state = self.robot.get_current_state()
                time_optimized_plan = self.arm.retime_trajectory(cur_rob_state, concatenated_plan,
                                                                 algorithm="time_optimal_trajectory_generation")
                self.arm.set_start_state_to_current_state()
                success = self.arm.execute(time_optimized_plan, wait=True)
                if not success:
                    return False
                sub_path_act["action"]()
                self.arm.set_start_state_to_current_state()
                concatenated_plan = RobotTrajectory()
        return True

    ### METHOD WITH TASK SEQUENCE CONSTRUCTION AND EXECUTION
    def pick_n_place(self, block_pick_pose, block_place_pose, approach_z=0.06, deapproach_z=0.06):
        if self.arm is None:
            return False

        pick_p = block_pick_pose

        appr_pick_p = copy.deepcopy(pick_p)
        appr_pick_p.position.z += approach_z

        deappr_pick_p = copy.deepcopy(pick_p)
        deappr_pick_p.position.z += deapproach_z

        place_p = block_place_pose

        appr_place_p = copy.deepcopy(place_p)
        appr_place_p.position.z += approach_z

        deappr_place_p = copy.deepcopy(place_p)
        deappr_place_p.position.z += deapproach_z

        self.open_gripper(0)

        # construct moves and action sequence
        pathes_n_actions = [
            {"path": [self.above_table1_pose_j, appr_pick_p],
             "action": None,
             "move_type": "movej"},

            {"path": [pick_p],
             "action": self.close_gripper,
             "move_type": "movel"},

            {"path": [deappr_pick_p],
             "action": None,
             "move_type": "movel"},

            {"path": [self.home_pose_j, self.above_table2_pose_j, appr_place_p],
             "action": None,
             "move_type": "movej"},

            {"path": [place_p],
             "action": self.open_gripper,
             "move_type": "movel"},

            {"path": [deappr_place_p],
             "action": None,
             "move_type": "movel"},

            {"path": [self.home_pose_j],
             "action": self.arm.set_start_state_to_current_state,  # add zero action for execution
             "move_type": "movej"}
        ]

        if not self.execute_pathes_n_actions(pathes_n_actions):
            self.open_gripper()
            self.go_home()
            return False
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
