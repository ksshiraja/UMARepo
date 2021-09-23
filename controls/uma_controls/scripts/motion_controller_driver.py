#!/usr/bin/env python

# Copyright 2019-20 UM::Autonomy
import rospy

import signal
# Add ../src/motion_controller to the import path
import os
import sys
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0, parentdir + "/src/motion_controller")

# For reasoning on this "noqa" comment, see
# http://docs.ros.org/lunar/api/roslint/html/pep8_8py_source.html Line 1545
# Why write documentation when you can encode magical answers 1500 lines into your code?
from motion_controller import MotionController  # noqa


if __name__ == "__main__":
    rospy.init_node('motion_controller', disable_signals=True)
    controller = MotionController()
    signal.signal(signal.SIGINT, controller._signalHandler)
    rospy.spin()
