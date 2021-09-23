#!/usr/bin/env python
from math import cos, sin, pi, atan2, sqrt
import numpy as np
import unittest
import traceback
import csv
# Add ../src/motion_controller to the import path
import os
import sys
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0, parentdir + "/src/motion_controller")

from lqr_controller import LQRController  # noqa

is_plotting = False  # Set by sys args, global variables are hard

target_velocity = 0.1
steps = 1000
inverse_rate = 0.1  # Will update at 1 / inverse_rate Hz
tSpan = [x * 0.1 for x in range(steps)]

Q = np.array([[10000, 0, 0, 0, 0, 0],
             [0, 10000, 0, 0, 0, 0],
             [0, 0, 821, 0, 0, 0],
             [0, 0, 0, 1000, 0, 0],
             [0, 0, 0, 0, 1000, 0],
             [0, 0, 0, 0, 0, 131]])

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


class TestLQRController(unittest.TestCase):
    def setUp(self):
        self.controller = LQRController(Q, R, rho, D, mass, i_z, a, 1 / inverse_rate)

    def test_circle_path(self, to_plot=True):
        position_tolerence = 0.05  # 5 cm
        angle_tolerance = 0.087  # 5 degrees (in radians)
        cur_state = np.array([1, 0, pi/2, 0, 0.1, 0.1])
        correct_by_turn = 0

        circle_steps = min(steps, 628)  # Don't wrap the circle... it's a headache

        X = np.empty((circle_steps, 6))
        for i in range(circle_steps):
            X[i][0] = cos(tSpan[i] / 10.0)
            X[i][1] = sin(tSpan[i] / 10.0)
            X[i][2] = (atan2(X[i, 1], X[i, 0]) + pi / 2) % (2 * pi)
            X[i][3] = -sin(tSpan[i] / 10.0)
            X[i][4] = cos(tSpan[i] / 10.0)
            X[i][5] = 1 / 10.0
        X[-1][3] = 0
        X[-1][4] = 0
        X[-1][5] = 0
        self.controller.hardcodedPathInit(X)

        self._run_test(position_tolerence, angle_tolerance, cur_state, correct_by_turn, X, to_plot)

    def test_circle_extended(self, to_plot=True):
        position_tolerence = 0.05  # 5 cm
        angle_tolerance = 0.087  # 5 degrees (in radians)
        cur_state = np.array([1, 0, pi/2, 0, 0.1, 0.1])
        correct_by_turn = 0

        circle_steps = min(steps, 628)  # Don't wrap the circle... it's a headache

        X = np.empty((circle_steps, 6))
        for i in range(circle_steps):
            X[i][0] = cos(tSpan[i] / 10.0)
            X[i][1] = sin(tSpan[i] / 10.0)
            X[i][2] = (atan2(X[i, 1], X[i, 0]) + pi / 2) % (2 * pi)
            X[i][3] = -sin(tSpan[i] / 10.0)
            X[i][4] = cos(tSpan[i] / 10.0)
            X[i][5] = 1 / 10.0
        X[-1][3] = 0
        X[-1][4] = 0
        X[-1][5] = 0
        self.controller.hardcodedPathInit(X)

        self._run_test_extended(position_tolerence, angle_tolerance, cur_state, correct_by_turn, X, to_plot)

    def test_horizontal_line(self, to_plot=False):
        position_tolerence = 0.05  # 5 cm
        angle_tolerance = 0.087  # 5 degrees (in radians)
        cur_state = np.array([0, 0.1, 0, 0, 0, 0])
        correct_by_turn = 20

        X = np.empty((steps, 6))
        for i in range(steps):
            X[i][0] = tSpan[i] / 10.0
            X[i][1] = 0
            X[i][2] = 0
            X[i][3] = 1
            X[i][4] = 0
            X[i][5] = 0
        X[-1][3] = 0
        self.controller.hardcodedPathInit(X)

        self._run_test(position_tolerence, angle_tolerance, cur_state, correct_by_turn, X, to_plot)

    def test_vertical_line(self, to_plot=False):
        position_tolerence = 0.05  # 5 cm
        angle_tolerance = 0.087  # 5 degrees (in radians)
        cur_state = np.array([0.1, 0.1, pi/4, -0.2, -0.1, -0.03])
        correct_by_turn = 40

        X = np.empty((steps, 6))
        for i in range(steps):
            X[i][0] = 0
            X[i][1] = tSpan[i] / 10.0
            X[i][2] = pi / 2
            X[i][3] = 0
            X[i][4] = 1
            X[i][5] = 0
        X[-1][4] = 0
        self.controller.hardcodedPathInit(X)

        self._run_test(position_tolerence, angle_tolerance, cur_state, correct_by_turn, X, to_plot)

    def test_diagonal_line(self, to_plot=False):
        # y = x
        position_tolerence = 0.05  # 5 cm
        angle_tolerance = 0.087  # 5 degrees (in radians)
        cur_state = np.array([0.5, 0, pi/2, 0, 0, 0])
        correct_by_turn = 40

        X = np.empty((steps, 6))
        for i in range(steps):
            X[i][0] = tSpan[i] / 10.0
            X[i][1] = tSpan[i] / 10.0
            X[i][2] = pi/4
            X[i][3] = 1
            X[i][4] = 1
            X[i][5] = 0
        X[-1][3] = 0
        X[-1][4] = 0
        self.controller.hardcodedPathInit(X)

        self._run_test(position_tolerence, angle_tolerance, cur_state, correct_by_turn, X, to_plot)

    def test_gen_vertical_line(self, to_plot=False):
        position_tolerence = 0.05  # 5 cm
        angle_tolerance = 0.087  # 5 degrees (in radians)
        cur_state = np.array([0.1, 0.1, pi/4, -0.2, -0.1, -0.03])
        correct_by_turn = 40

        X = np.empty((steps, 2))
        for i in range(steps):
            X[i][0] = 0
            X[i][1] = tSpan[i] / 10.0
        is_valid = self.controller.pathInit(X[:, 0], X[:, 1])
        self.assertTrue(is_valid)

        self._run_test(position_tolerence, angle_tolerance, cur_state, correct_by_turn, X, to_plot)

    def test_gen_big_circle(self, to_plot=False):
        position_tolerence = 0.05  # 5 cm
        angle_tolerance = 0.087  # 5 degrees (in radians)
        cur_state = np.array([0.1, 0.1, pi/4, -0.2, -0.1, -0.03])
        correct_by_turn = 40

        X = self._readInCsv("big_circle.csv")
        is_valid = self.controller.pathInit(X[:, 0], X[:, 1])
        self.assertTrue(is_valid)

        self._run_test(position_tolerence, angle_tolerance, cur_state, correct_by_turn, X, to_plot)

    def test_gen_big_circle_backwards(self, to_plot=False):
        position_tolerence = 0.05  # 5 cm
        angle_tolerance = 0.087  # 5 degrees (in radians)
        cur_state = np.array([0.1, 0.1, pi/4, -0.2, -0.1, -0.03])
        correct_by_turn = 40

        X = self._readInCsv("big_circle.csv", True)
        is_valid = self.controller.pathInit(X[:, 0], X[:, 1])
        self.assertTrue(is_valid)

        self._run_test(position_tolerence, angle_tolerance, cur_state, correct_by_turn, X, to_plot)

    # TODO(tejas): Make a test case where we go past the length of X. Extend X by 1 to be stationary at the end
    # TODO(tejas): Make more hardcoded X tests, like y = x
    # TODO(tejas): Make larger system tests where you only pass the x, y channels, so recreate the hardcoded tests,
    #              add parametric curves, y = sin(x), or other, more complex curves

    def _run_test(self, position_tolerence, angle_tolerance, cur_state, correct_by_turn, X, to_plot):
        boat_line = np.empty((steps, 2))
        last_index = -1
        try:
            for i in range(steps):
                if i >= correct_by_turn:
                    lower = 0 if last_index - 10 < 0 else last_index - 10
                    upper = len(X) if last_index + 10 > len(X) or last_index == -1 else last_index + 10
                    nearest_index = min(range(lower, upper),
                                        key=lambda x:
                                        sqrt(self.controller._distanceSquared(cur_state[0], cur_state[1],
                                                                              X[x][0], X[x][1])))
                    last_index = nearest_index
                    nearest_ideal = X[nearest_index]
                    if nearest_index < len(X) - 1:
                        self._assertState(cur_state, nearest_ideal, position_tolerence, angle_tolerance)
                boat_line[i] = [cur_state[0], cur_state[1]]

                u_vector, A, B = self.controller.testGetGain(cur_state, target_velocity)

                cur_state = self._updateState(cur_state, u_vector, A, B)
        except AssertionError as e:
            # Manually catch the assertion error, plot, and throw it again
            if to_plot:
                self._plotResults(boat_line[:i], X)
            raise AssertionError(traceback.format_exc())

        if to_plot:
            self._plotResults(boat_line, X)

    def _run_test_extended(self, position_tolerence, angle_tolerance, cur_state, correct_by_turn, X, to_plot):
        boat_line = np.empty((steps, 2))
        last_index = -1
        try:
            for i in range(steps + 5):
                if i >= correct_by_turn:
                    lower = 0 if last_index - 10 < 0 else last_index - 10
                    upper = len(X) if last_index + 10 > len(X) or last_index == -1 else last_index + 10
                    nearest_index = min(range(lower, upper),
                                        key=lambda x:
                                        sqrt(self.controller._distanceSquared(cur_state[0], cur_state[1],
                                                                              X[x][0], X[x][1])))
                    last_index = nearest_index
                    nearest_ideal = X[nearest_index]
                    if nearest_index < len(X) - 1:
                        self._assertState(cur_state, nearest_ideal, position_tolerence, angle_tolerance)

                if i < steps:
                    boat_line[i] = [cur_state[0], cur_state[1]]

                u_vector, A, B = self.controller.testGetGain(cur_state, target_velocity)

                cur_state = self._updateState(cur_state, u_vector, A, B)
        except AssertionError as e:
            # Manually catch the assertion error, plot, and throw it again
            if to_plot:
                self._plotResults(boat_line[:i], X)
            raise AssertionError(traceback.format_exc())

        if to_plot:
            self._plotResults(boat_line, X)

    def _updateState(self, cur_state, u_vector, A, B):
        a_x = np.dot(A[3, :], cur_state) + np.dot(B[3, :], u_vector)
        a_y = np.dot(A[4, :], cur_state) + np.dot(B[4, :], u_vector)
        a_w = np.dot(A[5, :], cur_state) + np.dot(B[5, :], u_vector)

        cur_state[3] = cur_state[3] + a_x*inverse_rate
        cur_state[4] = cur_state[4] + a_y*inverse_rate
        cur_state[5] = cur_state[5] + a_w*inverse_rate
        cur_state[0] = cur_state[0] + cur_state[3]*inverse_rate
        cur_state[1] = cur_state[1] + cur_state[4]*inverse_rate
        cur_state[2] = (cur_state[2] + cur_state[5]*inverse_rate) % (2*pi)

        return cur_state

    def _assertState(self, cur_state, ideal_state, position_tolerence, angle_tolerance):
        self.assertTrue(abs(cur_state[0] - ideal_state[0]) < position_tolerence)
        self.assertTrue(abs(cur_state[1] - ideal_state[1]) < position_tolerence)
        if len(ideal_state) > 2:
            # https://gamedev.stackexchange.com/questions/4467/comparing-angles-and-working-out-the-difference
            self.assertTrue(pi - abs(abs(cur_state[2] - ideal_state[2]) - pi) < angle_tolerance)
            self.assertTrue(abs(cur_state[3] - ideal_state[3] * target_velocity) < position_tolerence)
            self.assertTrue(abs(cur_state[4] - ideal_state[4] * target_velocity) < position_tolerence)
            self.assertTrue(abs(cur_state[5] - ideal_state[5]) < angle_tolerance)

    def _plotResults(self, boat_line, X):
        if is_plotting:
            global fig, plot_count
            subplot = fig.add_subplot(220 + plot_count)

            subplot.plot(X[:, 0], X[:, 1], label="Ideal State")
            subplot.plot(boat_line[:, 0], boat_line[:, 1], label="Boat  State")
            subplot.set_title("Test {}".format(plot_count - 1))
            # subplot.legend()
            subplot.grid()
            subplot.spines['top'].set_color('none')
            subplot.spines['bottom'].set_position('zero')
            subplot.spines['left'].set_position('zero')
            subplot.spines['right'].set_color('none')

            plot_count += 1

    def _readInCsv(self, file_name, reverse=False):
        rows = []
        with open(file_name) as fp:
            reader = csv.reader(fp)
            for row in reader:
                rows.append(row)

        X = np.empty((len(rows[0]), 2))
        X[:, 0] = np.array(rows[0])
        X[:, 1] = np.array(rows[1])

        if reverse:
            for i in range(len(rows)):
                X[len(rows) - 1 - i][0] = rows[i][0]
                X[len(rows) - 1 - i][1] = rows[i][1]

        return X

if __name__ == '__main__':
    if len(sys.argv) > 1:
        plot_arg = sys.argv[-1]
        if plot_arg == "-p" or plot_arg == "--plot":
            sys.argv.pop()  # Pop the argument so unittest doesn't get it
            is_plotting = True
            import matplotlib.pyplot as plt
        elif plot_arg == "-h" or plot_arg == "--help":
            print("Use -p or --plot as the last argument to plot the tests. \
                   Below is the unittest package's --help:\n\n")

    if is_plotting:
        global fig, plot_count
        fig = plt.figure()
        plot_count = 1

    unittest.main(exit=False)

    if is_plotting:
        fig.subplots_adjust(hspace=.5, wspace=0.5)
        plt.show()
