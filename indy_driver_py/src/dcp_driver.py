#!/usr/bin/python3
# -*- coding: utf-8 -*-

import sys
import json
import time

import numpy as np
from scipy import interpolate

from indy_utils import indydcp_client
from indy_utils import indy_program_maker

import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Bool, Empty, Int32MultiArray
from geometry_msgs.msg import Pose
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import FollowJointTrajectoryFeedback
import utils_transf

from indy_custom_comp_msgs import msg as comp_msg
from indy_custom_comp_msgs import srv as comp_srv


ROBOT_STATE = {
    0: "robot_ready",
    1: "busy",
    2: "direct_teaching_mode",
    3: "collided",
    4: "emergency_stop",
}


class IndyROSConnector:
    def __init__(self, robot_ip, robot_name, joint_names):
        self.robot_name = robot_name
        self.joint_names = joint_names

        # Connect to Robot
        self.indy = indydcp_client.IndyDCPClient(robot_ip, robot_name)

        # Initialize ROS node
        rospy.init_node("indy_driver_py")

        # Custom Component Elements
        self.algorithm_switch_srv = rospy.Service("indy/algorithm_switch", comp_srv.AlgorithmSwitch, self.algorithm_switch_cb)
        self.current_algorithm_pub = rospy.Publisher("indy/current_algorithm", comp_msg.CompAlgorithm, queue_size=1)
        self._current_algorithm_msg = comp_msg.CompAlgorithm()

        # Publish current robot state
        self.joint_state_pub = rospy.Publisher("joint_states", JointState, queue_size=10)
        self.indy_state_pub = rospy.Publisher("indy/status", GoalStatusArray, queue_size=10)
        self.control_state_pub = rospy.Publisher("feedback_states", FollowJointTrajectoryFeedback, queue_size=1)

        # Subscribe desired joint position
        self.joint_execute_plan_sub = rospy.Subscriber(
            "joint_path_command", JointTrajectory, self.execute_plan_result_cb, queue_size=1
        )

        # Subscribe command
        self.execute_joint_state_sub = rospy.Subscriber(
            "indy/execute_joint_state", JointState, self.execute_joint_state_cb, queue_size=1
        )
        self.stop_sub = rospy.Subscriber("stop_motion", Bool, self.stop_robot_cb, queue_size=1)
        self.set_motion_param_sub = rospy.Subscriber(
            "indy/motion_parameter", Int32MultiArray, self.set_motion_param_cb, queue_size=1
        )

        # Misc variable
        self.joint_state_list = []
        self.execute = False
        self.vel = 3
        self.blend = 5

        # store latest robot state
        self._state_time_out = 0.7
        self._last_joint_state_msg = JointState()
        self._last_control_state_msg = FollowJointTrajectoryFeedback()

    def __del__(self):
        self.indy.disconnect()

    def execute_joint_state_cb(self, msg):
        self.joint_state_list = [msg.position]

        if self.execute == False:
            self.execute = True

    def execute_plan_result_cb(self, msg):
        # download planned path from ros moveit
        self.joint_state_list = []
        if msg.points:
            self.joint_state_list = [p.positions for p in msg.points]
        else:
            self.indy.stop_motion()

        if self.execute == False:
            self.execute = True

    def stop_robot_cb(self, msg):
        if msg.data == True:
            self.indy.stop_motion()

    def set_motion_param_cb(self, msg):
        param_array = msg.data
        self.vel = param_array[0]
        self.blend = param_array[1]
        rospy.loginfo("Set motion parmameters. vel: {}, blend: {}".format(self.vel, self.blend))

    def move_robot(self):
        if self.joint_state_list:
            start_time = time.time()
            prog = indy_program_maker.JsonProgramComponent(policy=0, resume_time=2)
            prog_time = time.time()
            for j_pos in self.joint_state_list:
                prog.add_joint_move_to(utils_transf.rads2degs(j_pos), vel=self.vel, blend=self.blend)
            add_time = time.time()
            json_string = json.dumps(prog.json_program)
            json_time = time.time()
            self.indy.set_and_start_json_program(json_string)
            set_time = time.time()

            # TODO: Figure out Indy7 communication problem.
            # If ROS Industrial can't get joint states in 1s, it aborts execution.
            # Indy7 RC series has problem that it takes more than 1s while it upload
            # the planned path. It invokes the abort execution.
            if set_time - start_time > 1:
                rospy.logerr("[DEBUG] trajectory length: {}".format(len(self.joint_state_list)))
                rospy.logerr(
                    "prog time: {}, add time: {}, json time: {}, set time: {}".format(
                        prog_time - start_time, add_time - prog_time, json_time - add_time, set_time - json_time
                    )
                )
            self.joint_state_list = []

    def joint_state_publisher(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = utils_transf.degs2rads(self.indy.get_joint_pos())

        control_state_msg = FollowJointTrajectoryFeedback()
        control_state_msg.header.stamp = rospy.Time.now()
        control_state_msg.joint_names = self.joint_names
        control_state_msg.actual.positions = utils_transf.degs2rads(self.indy.get_joint_pos())
        control_state_msg.desired.positions = utils_transf.degs2rads(self.indy.get_joint_pos())
        control_state_msg.error.positions = [0 for i in control_state_msg.joint_names]

        self._last_joint_state_msg = joint_state_msg
        self._last_control_state_msg = control_state_msg
        # self.joint_state_pub.publish(joint_state_msg)
        # self.control_state_pub.publish(control_state_msg)

    def robot_state_publisher(self):
        if self.current_robot_status["ready"]:
            state_num = 0

        if self.current_robot_status["busy"]:
            state_num = 1

        if self.current_robot_status["direct_teaching"]:
            state_num = 2

        if self.current_robot_status["collision"]:
            state_num = 3

        if self.current_robot_status["emergency"]:
            state_num = 4

        status_msg = GoalStatusArray()
        status_msg.header.stamp = rospy.Time.now()

        status = GoalStatus()
        status.goal_id.stamp = rospy.Time.now()
        status.goal_id.id = ""
        status.status = state_num
        status.text = ROBOT_STATE[state_num]

        status_msg.status_list = [status]

        self.indy_state_pub.publish(status_msg)

    def fake_publish_state_cb(self, msg):
        cur_time = rospy.Time.now()
        last_joint_state_time = self._last_joint_state_msg.header.stamp
        if cur_time - last_joint_state_time > rospy.Duration(self._state_time_out):
            self._last_control_state_msg.header.stamp = cur_time
            self._last_joint_state_msg.header.stamp = cur_time
        else:
            pass
        self.joint_state_pub.publish(self._last_joint_state_msg)
        self.control_state_pub.publish(self._last_control_state_msg)

    def publish_state_cb(self, msg):
        self.joint_state_publisher()
        self.robot_state_publisher()

    def move_robot_cb(self, msg):
        self.current_robot_status = self.indy.get_robot_status()
        if self.execute:
            self.execute = False
            if self.current_robot_status["busy"]:
                pass
            if self.current_robot_status["direct_teaching"]:
                pass
            if self.current_robot_status["collision"]:
                pass
            if self.current_robot_status["emergency"]:
                pass
            if self.current_robot_status["ready"]:
                self.move_robot()

    def current_algorithm_cb(self, _):
        """Select algorithm to use.

        Service Flow:
            1. ros_node -(ROS Service)-> dcp_driver.py
            2. dcp_driver.py -(IndyDCP: req_from_ros)-> Indy7 component
            3. Indy7 component -(IndyDCP: resp_to_ros)-> dcp_driver.py

        Publishing Flow of
            1. Indy7 component -(IndyDCP: resp_current_status)-> dcp_driver.py
            2. dcp_driver.py -(ROS Topic)-> ros_node

        IndyDCP Mapping:
            [uint8, B000] resp_to_ros (Not Implemented)
            [int32, I000] req_from_ros
                - HEX Map:
                    0x0000 0000: Not initialized
                    0x**__ ____: Joint Control Component
                    0x__** ____: Joint Trajectory Interpolation Component
                    0x____ **__: Task Control Component
                    0x____ __**: Task Trajectory Interpolation Component
                - Controller Value:
                    0x01__: H-infinity PID control
                    0x02__: TBD
                - Trajectory Interpolation Value:
                    0x__01: TBD
                    0x__02: TBD
            [int32, I001] resp_current_status
        """
        dv_value = self.indy.read_direct_variable(indydcp_client.DIRECT_VAR_TYPE_DWORD, 1)
        dv_value = int(dv_value)
        self._current_algorithm_msg.joint_component = (dv_value >> 16) & 0xFFFF
        self._current_algorithm_msg.task_component = dv_value & 0xFFFF
        self.current_algorithm_pub.publish(self._current_algorithm_msg)

    def algorithm_switch_cb(self, req: comp_srv.AlgorithmSwitchRequest):
        # [int32, I000] req_from_ros
        dv_value: int = (req.req.joint_component << 16) | req.req.task_component
        self.indy.write_direct_variable(indydcp_client.DIRECT_VAR_TYPE_DWORD, 0, dv_value)
        rospy.logwarn(f"Algorithm Switched to: {req.req}")
        rospy.logwarn(f"dv_value: {dv_value}")
        # [uint8, B000] resp_to_ros (Not Implemented)
        _ = self.indy.read_direct_variable(indydcp_client.DIRECT_VAR_TYPE_BYTE, 0)
        # ROS Response
        resp = comp_srv.AlgorithmSwitchResponse()

        # Temporary implementation of waiting for the algorithm to be switched
        timeout = 10.0
        while not rospy.is_shutdown():
            if (self._current_algorithm_msg.joint_component == req.req.joint_component) and (
                self._current_algorithm_msg.task_component == req.req.task_component
            ):
                resp.success = True
                break
            rospy.sleep(1.0)
            timeout -= 1.0
            if timeout <= 0.0:
                resp.success = False
                break
        return resp

    def run(self):
        self.indy.connect()

        rospy.Timer(rospy.Duration(0.05), self.publish_state_cb)
        rospy.Timer(rospy.Duration(0.1), self.fake_publish_state_cb)
        rospy.Timer(rospy.Duration(0.05), self.move_robot_cb)
        rospy.Timer(rospy.Duration(1.0), self.current_algorithm_cb)
        rospy.spin()

        self.indy.disconnect()

    @staticmethod
    def interpolate_joint_trajectory(msg, time_step):
        # allocate the pos and time list
        trajectory_pos = []
        trajectory_time = []

        # get trajectory information
        trajectory_time = [p.time_from_start.secs + p.time_from_start.nsecs * 1e-9 for p in msg.points]
        trajectory_pos = [p.positions for p in msg.points]

        # make numpy
        trajectory_time = np.array(trajectory_time)
        trajectory_pos = np.array(trajectory_pos).transpose()

        # remove duplicated points on the trajectory
        _, unique_indices = np.unique(trajectory_time, return_index=True)
        trajectory_time = trajectory_time[unique_indices]
        trajectory_pos = trajectory_pos[:, unique_indices]

        # adjust the final point time to times of time_step
        new_points_num = round(trajectory_time[-1] / time_step)
        trajectory_time[-1] = (new_points_num - 1) * time_step

        # generate new trajectory points time
        new_trajectory_time = np.linspace(trajectory_time[0], trajectory_time[-1], new_points_num)
        new_trajectory_time = np.arange(0, trajectory_time[-1] + 0.001, time_step)

        # interpolate the trajectory
        if len(trajectory_time) < 3:
            f = interpolate.interp1d(trajectory_time, trajectory_pos, kind="linear")
        elif len(trajectory_time) == 3:
            f = interpolate.interp1d(trajectory_time, trajectory_pos, kind="quadratic")
        else:
            f = interpolate.interp1d(trajectory_time, trajectory_pos, kind="cubic")
        new_trajectory_pos = f(new_trajectory_time).transpose()

        return new_trajectory_pos.tolist()


def main():
    robot_ip = rospy.get_param("robot_ip_address")
    robot_name = rospy.get_param("robot_name")

    # get joint names for dual arm configuration on MoveIt!
    try:
        joint_names = rospy.get_param("controller_joint_names")
    except KeyError:
        rospy.logwarn("No joint names found on the parameter server")

        # if not found, use default joint names
        if robot_name == "NRMK-IndyRP2":
            joint_names = ["joint0", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        else:
            joint_names = ["joint0", "joint1", "joint2", "joint3", "joint4", "joint5"]

    t = IndyROSConnector(robot_ip, robot_name, joint_names)
    t.run()


if __name__ == "__main__":
    main()
