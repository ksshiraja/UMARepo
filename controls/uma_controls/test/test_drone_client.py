#!/usr/bin/env python

import actionlib
import uma_controls_external_interface.msg as controls_msgs
import geometry_msgs
import std_msgs
import rospy
import time


def feedback_cbk(fb):
    if(fb.drone_state == 3):
        print "Drone has been cancelled"


def testClient():
    rospy.init_node("drone-coms")  # Half a day in 100 degree weather for this one line because I'm stupid
    boat = actionlib.SimpleActionClient('drone_controller_server', controls_msgs.DroneControlAction)

    dock_header = std_msgs.msg.Header(0, rospy.get_rostime(), "map")
    dock_point = geometry_msgs.msg.Point(0, 0, 0)
    dock_msg = geometry_msgs.msg.PointStamped(dock_header, dock_point)
    goal = controls_msgs.DroneControlGoal(dock_location=dock_msg)

    boat.wait_for_server()

    print "Server started up"
    boat.send_goal(goal, feedback_cb=feedback_cbk)

    print "Goal sent"
    boat.wait_for_result()

    print "Result obtained"
    print boat.get_result()

if __name__ == "__main__":
    testClient()
