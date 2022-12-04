#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
from indy_custom_comp_msgs import msg as comp_msg
from indy_custom_comp_msgs import srv as comp_srv


def main():
    rospy.init_node("request_algorithm_switch")
    rospy.wait_for_service("indy/algorithm_switch")
    proxy = rospy.ServiceProxy("indy/algorithm_switch", comp_srv.AlgorithmSwitch)

    req = comp_srv.AlgorithmSwitchRequest()
    req.req.joint_component = comp_msg.CompAlgorithm.H_INFINITY_PID
    # req.req.joint_component = comp_msg.CompAlgorithm.CONTROL_TBD

    rospy.loginfo(f"Request: {req}")
    resp = proxy(req)
    rospy.loginfo(f"Response: {resp}")

    rospy.spin()


if __name__ == "__main__":
    main()
