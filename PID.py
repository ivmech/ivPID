#!/usr/bin/python
#
# This file is part of IvPID.
# Copyright (C) 2015 Ivmech Mechatronics Ltd. <bilgi@ivmech.com>
#
# IvPID is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# IvPID is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

# title           :PID.py
# description     :python pid controller
# author          :Caner Durmusoglu
# date            :20151218
# version         :0.1
# notes           :
# python_version  :2.7
# ==============================================================================

"""Ivmech PID Controller is simple implementation of a Proportional-Integral-Derivative (PID) Controller at Python Programming Language.
More information about PID Controller: http://en.wikipedia.org/wiki/PID_controller
"""
import time

class PID:
    """PID Controller
    """

    def __init__(self, P=2.0, I=0.0, D=1.0):

        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.sample_time = 0.01
        self.current_time = time.time()
        self.last_time = self.current_time

        self.clear()

    def clear(self):
        """Clears PID computations and coefficients"""
        self.SetPoint = 0.0

        self.Cp = 0.0
        self.Ci = 0.0
        self.Cd = 0.0
        self.last_error = 0.0

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 20.0

        self.output = 0.0

    def update(self, feedback_value):
        """Calculates PID value for given reference input and feedback
        """
        error = self.SetPoint - feedback_value

        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if (delta_time >= self.sample_time):
            self.Cp = self.Kp * error
            self.Ci += error * delta_time

            if (self.Ci < -self.windup_guard):
                self.Ci = -self.windup_guard
            elif (self.Ci > self.windup_guard):
                self.Ci = self.windup_guard

            self.Cd = 0.0
            if delta_time > 0:
                self.Cd = delta_error / delta_time

            self.last_time = self.current_time
            self.last_error = error

            self.output = self.Cp + (self.Ki * self.Ci) + (self.Kd * self.Cd)

    def setKp(self, P):
        """Sets Proportional Coefficient"""
        self.Kp = P

    def setKi(self, I):
        """Sets Integral Coefficient"""
        self.Ki = I

    def setKd(self, D):
        """Sets Derivative Coefficient"""
        self.Kd = D

    def setWindup(self, W):
        """Sets Windup Guard Limit"""
        self.windup_guard = W

    def setSampleTime(self, sample_time):
        """Sets PID Computation Time Interval"""
        self.sample_time = sample_time
