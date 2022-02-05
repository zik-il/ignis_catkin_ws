#!/usr/bin/env python3
import time
from jetbot import Robot

# This code is mainly extracted from examples of the jetbot notebooks

class Wheels(object):

    def __init__(self):
        self.robot = Robot()
        self.stop()

    def cmd_vel_process(self, linear_speed, angular_speed):

        if angular_speed != 0:
            # We turn
            if angular_speed > 0:
                self.go_left(abs(angular_speed))
            else:
                self.go_right(abs(angular_speed))
        else:
            if linear_speed > 0:
                self.go_forward(abs(linear_speed))
            elif linear_speed < 0:
                self.go_backward(abs(linear_speed))
            else:
                # We have to stop
                self.stop()
    

    def stop(self):
        self.robot.stop()

    def go_forward(self, speed=0.4):
        self.robot.forward(speed)

    def go_backward(self, speed=0.4):
        self.robot.backward(speed)

    def go_left(self, speed=0.3):
        self.robot.left(speed)

    def go_right(self, speed=0.3):
        self.robot.right(speed)
        


