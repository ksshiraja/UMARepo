import unittest
from math import *
import numpy as np
# Add ../src/motion_controller to the import path
import os
import sys
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0, parentdir + "/src/motion_controller")

from lqr_controller import LQRController  # noqa

steps = 1000
inverse_rate = 0.1  # Will update at 1 / inverse_rate Hz
tSpan = [x * 0.1 for x in range(steps)]

Q = np.array([[10000, 0, 0, 0, 0, 0],
             [0, 10000, 0, 0, 0, 0],
             [0, 0, 131.3, 0, 0, 0],
             [0, 0, 0, 10000, 0, 0],
             [0, 0, 0, 0, 10000, 0],
             [0, 0, 0, 0, 0, 131.3]])

R = np.array([[1, 0, 0, 0],
             [0, 1, 0, 0],
             [0, 0, 8, 0],
             [0, 0, 0, 8]])

rho = 1

D = np.array([[1, 1],
             [1, 1],
             [1, 1]])

a = [0.1778, 0.1778, 0.1778, 0.1778]
i_z = 2.7782509999999996
mass = 22.6796


class TestLQRStateVectorGeneration(unittest.TestCase):
    def setUp(self):
        self.controller = LQRController(Q, R, rho, D, mass, i_z, a, 1 / inverse_rate)

    def test_x_y_points(self):
        x_channel = [0, 1]
        y_channel = [1, 0]

        state_vectors, is_valid = self.controller._genStateVectors(x_channel, y_channel, tSpan)
        self.assertTrue(is_valid)

        self.assertTrue(state_vectors[0][0] == 0)
        self.assertTrue(state_vectors[0][1] == 1)
        self.assertTrue(state_vectors[1][0] == 1)
        self.assertTrue(state_vectors[1][1] == 0)

    def test_angle(self):
        angle_tolerance = 0.087  # 5 degrees (in radians)

        a = [0, 0]
        b = [0, 1]

        self.assertTrue(self.controller._findAngle(a[0], a[1], b[0], b[1]) - pi / 2 < angle_tolerance)

    def test_circle(self):
        position_tolerence = 0.05  # 5 cm
        angle_tolerance = 0.087  # 5 degrees (in radians)

        X = np.empty((steps, 6))
        for i in range(steps):
            X[i][0] = cos(tSpan[i] / 10.0)
            X[i][1] = sin(tSpan[i] / 10.0)
            X[i][2] = 0  # Tmp
            X[i][3] = -sin(tSpan[i] / 10.0)
            X[i][4] = cos(tSpan[i] / 10.0)
            X[i][5] = 1 / 10.0
        for i in range(steps):
            X[i, 2] = (atan2(X[i, 1], X[i, 0]) + pi / 2) % (2 * pi)
        X[-1][3] = 0
        X[-1][4] = 0
        X[-1][5] = 0

        state_vectors, is_valid = self.controller._genStateVectors(X[:, 0], X[:, 1], tSpan)
        self.assertTrue(is_valid)

        for i in range(steps):
            # Debugging
            # print("Gen   state: {}".format(state_vectors[i]))
            # print("Ideal state: {}".format(X[i]))
            self._assertState(state_vectors[i], X[i], position_tolerence, angle_tolerance)

    # Helper function to assert an entire state vector to be equal to an ideal vector
    def _assertState(self, cur_state, ideal_state, position_tolerence, angle_tolerance):
        self.assertTrue(abs(cur_state[0] - ideal_state[0]) < position_tolerence)
        self.assertTrue(abs(cur_state[1] - ideal_state[1]) < position_tolerence)
        # https://gamedev.stackexchange.com/questions/4467/comparing-angles-and-working-out-the-difference
        self.assertTrue(pi - abs(abs(cur_state[2] - ideal_state[2]) - pi) < angle_tolerance)
        self.assertTrue(abs(cur_state[3] - ideal_state[3]) < position_tolerence)
        self.assertTrue(abs(cur_state[4] - ideal_state[4]) < position_tolerence)
        self.assertTrue(abs(cur_state[5] - ideal_state[5]) < angle_tolerance)

if __name__ == '__main__':
    unittest.main()
