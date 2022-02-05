#!/usr/bin/env python3
import time
import math
import RPi.GPIO as GPIO
from PCA9685 import PCA9685

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

from ignisbot_wheels import Wheels

from numpy import interp

class IgnisBotMove(object):

    def __init__(self):
        self._pwm = PCA9685()

        self._PWM_FREQ = 50

        self._pwm.setPWMFreq(self._PWM_FREQ)

        self.CAMERA_PITCH_INIT_ANGLE = 90
        self.TAIL_YAW_INIT_ANGLE = 80

        self.CAMERA_PITCH_RANGES = {"max":130,
                                    "min":10}
        self.TAIL_YAW_RANGES = {"max":130,
                                "min":50}
        
        self.CAMERA_PITCH_RANGES_RAD = {"max":math.radians(self.CAMERA_PITCH_RANGES["max"]),
                                        "min":math.radians(self.CAMERA_PITCH_RANGES["min"])}
        self.TAIL_YAW_RANGES_RAD = {"max":math.radians(self.TAIL_YAW_RANGES["max"]),
                                    "min":math.radians(self.TAIL_YAW_RANGES["min"])}
        
        # Extracted from simulation
        self.CAMERA_PITCH_SIM_RANGES = {"max":0.86,
                                        "min":-1.21}
        self.TAIL_YAW_SIM_RANGES = {"max":0.45,
                                    "min":-0.45}
        
        self.SERVO_RANGES = {"camera":self.CAMERA_PITCH_RANGES,
                            "tail":self.TAIL_YAW_RANGES}

        self.move_ignis_to_pose(yaw=self.TAIL_YAW_INIT_ANGLE,
                               pitch=self.CAMERA_PITCH_INIT_ANGLE)
        
        self.ignisbot_wheels_obj = Wheels()

        # How much we reduse the cmdvel values incoming, because its too much for real robot
        self.cmd_vel_factor = 5

        rospy.Subscriber("/jetdog/camera_tilt_joint_position_controller/command", Float64, self.pitch_camera_callback)
        rospy.Subscriber("/jetdog/tail_pan_joint_position_controller/command", Float64, self.yaw_tail_callback)
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)

        rospy.loginfo("IgnisBotMove READY")

    def __del__(self):
        self._pwm.exit_PCA9685()
        print("\nCleaning IgnisBotMove...")
        GPIO.cleanup()
        print("\nIgnisBotMove END")
    
    def pitch_camera_callback(self, msg):
        """
        Data is in radians therefore we have to change to degrees as input.
        """

        degrees = math.degrees(interp(msg.data,[  self.CAMERA_PITCH_SIM_RANGES["min"],
                                        self.CAMERA_PITCH_SIM_RANGES["max"]]
                                    ,[  self.CAMERA_PITCH_RANGES_RAD["min"],
                                        self.CAMERA_PITCH_RANGES_RAD["max"]]))

        self.move_pitch(pitch_angle=degrees)
    
    def yaw_tail_callback(self, msg):
        """
        Data is in radians therefore we have to change to degrees as input.
        """
        degrees = math.degrees(interp(msg.data,[  self.TAIL_YAW_SIM_RANGES["min"],
                                        self.TAIL_YAW_SIM_RANGES["max"]]
                                    ,[  self.TAIL_YAW_RANGES_RAD["min"],
                                        self.TAIL_YAW_RANGES_RAD["max"]]))
        self.move_yaw(yaw_angle=degrees)
    
    def cmd_vel_callback(self, msg):
        self.ignisbot_wheels_obj.cmd_vel_process(linear_speed=(msg.linear.x / self.cmd_vel_factor),
                                                angular_speed=(msg.angular.z / (2*self.cmd_vel_factor)))


    def move_ignis_to_pose(self, yaw, pitch):
        self.move_yaw(yaw_angle=yaw)
        self.move_pitch(pitch_angle=pitch)


    def move_yaw(self, yaw_angle):
        n_yaw_angle = self.normalise_angles(angle=yaw_angle, servo_name="tail")
        self._pwm.setRotationAngle(0, n_yaw_angle)

    def move_pitch(self, pitch_angle):
        n_pitch_angle = self.normalise_angles(angle=pitch_angle, servo_name="camera")
        self._pwm.setRotationAngle(1, n_pitch_angle)
        
    def yaw_range_test(self):
        """
        It tests the range of yaw servo once
        :return:
        """
        for angle in range(10,170,1):
            rospy.logdebug("Moving Yaw="+str(angle))
            self.move_yaw(yaw_angle=angle)
            time.sleep(0.1)

        for angle in range(170,10,-1):
            rospy.logdebug("Moving Yaw="+str(angle))
            self.move_yaw(yaw_angle=angle)
            time.sleep(0.1)


    def pitch_range_test(self):
        """
        It tests the range of pitch servo once
        :return:
        """
        for angle in range(10,170,1):
            rospy.logdebug("Moving Pitch="+str(angle))
            self.move_pitch(pitch_angle=angle)
            time.sleep(0.1)

        for angle in range(170,10,-1):
            rospy.logdebug("Moving Pitch="+str(angle))
            self.move_pitch(pitch_angle=angle)
            time.sleep(0.1)
            
    def input_camera_tail_test(self):
        
        while True:
            input_tail_yaw = input("Type Tail Angle to move to")            
            input_tail_yaw_angle = int(input_tail_yaw)
            rospy.logdebug("Moving Yaw=" + str(input_tail_yaw_angle))
            self.move_yaw(yaw_angle=input_tail_yaw_angle)
            
            input_cam_pitch = input("Type Camera Pitch Angle to move to")
            input_cam_pitch_angle = int(input_cam_pitch)
            rospy.logdebug("Moving Pitch=" + str(input_cam_pitch_angle))
            self.move_pitch(pitch_angle=input_cam_pitch_angle)

    
    def normalise_angles(self, angle, servo_name):
        """
        It gives the clamped value of the angle given
        Based on the real robots limits.
        angle: angle to be normalised
        servo_name: Name to identify the servo to get one range or another
        """

        if servo_name in self.SERVO_RANGES:
            ranges_obj = self.SERVO_RANGES[servo_name]
            min_value= ranges_obj["min"]
            max_value= ranges_obj["max"]

            normalised_angle = self.clamp(  num=angle,
                                            min_value=min_value,
                                            max_value=max_value)
            rospy.logdebug("Servo exist==>"+str(servo_name)+", clamped="+str(normalised_angle))
        else:
            rospy.logdebug("Servo doesnt exist==>"+str(servo_name))
            normalised_angle = 0

        return normalised_angle

    def clamp(self, num, min_value, max_value):
        return max(min(num, max_value), min_value)
        

    def input_yaw_test(self):

        while True:
            input_x = input("Type Angle to move to")
            angle = int(input_x)
            rospy.logdebug("Moving Yaw=" + str(angle))
            self.move_yaw(yaw_angle=angle)

    def input_pitch_test(self):

        while True:
            input_x = input("Type Angle to move to")
            angle = int(input_x)
            rospy.logdebug("Moving Pitch=" + str(angle))
            self.move_pitch(pitch_angle=angle)


def InitTest():
    ignis_object = IgnisBotMove()
    input("Start Init Ignis Test...Press Key to end")

def InputCameraPitchTailYawTest():
    ignis_object = IgnisBotMove()
    ignis_object.input_camera_tail_test()

def TailWigleTest():
    ignis_object = IgnisBotMove()
    ignis_object.tail_wag(tail_hz=5.0)

def InputThroughTopicsCameraPitchTailYawTest():
    ignis_object = IgnisBotMove()
    ignis_object.input_camera_tail_test()

def main_start_ros_loop():
    rospy.init_node("ignisbot_move_node", log_level=rospy.INFO)
    ignis_object = IgnisBotMove()
    rospy.spin()


if __name__ == "__main__":
    #InitTest()
    #InputCameraPitchTailYawTest()
    main_start_ros_loop()
