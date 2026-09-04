#!/usr/bin/env python3
"""
led_emitter_shim.py  --  minimal stand-in for the missing led_emitter_node.

WHY THIS EXISTS
---------------
duck4's base image (dt-duckiebot-interface v4.3.2) dropped the daffy `led_emitter_node`
and its `set_pattern` service. The daffy `indefinite_navigation` demo's fsm_node is a
REQUIRED node that calls  /duck4/led_emitter_node/set_pattern  on startup and on state
changes. When that service is missing, fsm_node dies and roslaunch tears down the WHOLE
demo. fsm_node does NOT actually need the LEDs to light up -- it only needs the service
to EXIST and return success. This node provides exactly that: it advertises the service
and answers "OK" to every call, doing nothing else. Result: fsm_node stays alive and
lane following can run. (The LEDs just won't visually change -- cosmetic only.)

HOW THE NAME RESOLVES
---------------------
Run it as node name `led_emitter_node` inside namespace `duck4`, with the service
declared PRIVATE (`~set_pattern`). That resolves to exactly:
    /duck4/led_emitter_node/set_pattern
which is the name fsm_node looks up. Set the namespace with:  export ROS_NAMESPACE=duck4

IF THE SERVICE TYPE IS WRONG
---------------------------
This assumes the daffy type `duckietown_msgs/ChangePattern` (request: std_msgs/String
pattern_name; empty response). If starting the demo still errors about set_pattern with a
TYPE mismatch, check the expected type and adjust the import/return below:
    rossrv show duckietown_msgs/ChangePattern
"""

import rospy
from duckietown_msgs.srv import ChangePattern, ChangePatternResponse


def handle_set_pattern(req):
    # Just acknowledge. We ignore the requested pattern name entirely.
    try:
        name = req.pattern_name.data
    except Exception:
        name = "<unknown>"
    rospy.loginfo("[led_emitter_shim] set_pattern('%s') -> OK (no-op)", name)
    return ChangePatternResponse()


if __name__ == "__main__":
    # Node name must be led_emitter_node; combined with ROS_NAMESPACE=duck4 this makes
    # the private service ~set_pattern resolve to /duck4/led_emitter_node/set_pattern.
    rospy.init_node("led_emitter_node")
    srv = rospy.Service("~set_pattern", ChangePattern, handle_set_pattern)
    rospy.loginfo("[led_emitter_shim] READY - advertising %s", srv.resolved_name)
    rospy.spin()
