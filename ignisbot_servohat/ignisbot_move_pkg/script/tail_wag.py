#!/usr/bin/env python
import time
import math
import rospy
from std_msgs.msg import Float64

class TailWag(object):

    def __init__(self):
        rospy.loginfo("TailWag...")
        self.current_tailwag_hz = 0.0
        self.published_stop = False
        self.TAIL_YAW_SIM_RANGES = {"max":0.35,
                                    "min":-0.35,
                                    "middle":0.0}

        self.yaw_tail_pub = rospy.Publisher("/jetdog/tail_pan_joint_position_controller/command", Float64, queue_size=1)
        rospy.Subscriber("/ignisbot/tailwag_freq", Float64, self.tailwag_freq_callback)

        rospy.loginfo("TailWag...READY")


    def tailwag_freq_callback(self, msg):
        """
        Data will be the hz of the tail wag
        """
        self.current_tailwag_hz = msg.data


    def tail_wag(self, period):
        """
        Do one tailwag
        """
        self.move_tail(yaw_angle=self.TAIL_YAW_SIM_RANGES["min"])
        time.sleep(period)
        self.move_tail(yaw_angle=self.TAIL_YAW_SIM_RANGES["max"])
        time.sleep(period)

    def tail_wag_stop(self):
        self.move_tail(yaw_angle=self.TAIL_YAW_SIM_RANGES["middle"])

    def move_tail(self,yaw_angle):
        angle_tail = Float64()
        angle_tail.data = yaw_angle
        self.yaw_tail_pub.publish(angle_tail)

    def start_loop(self):
        # To avoid overflow we use rate 
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            if self.current_tailwag_hz > 0.0:
                period = 1/ self.current_tailwag_hz
                self.tail_wag(period=period)
                self.published_stop = False
            else:
                if not self.published_stop:
                    self.tail_wag_stop()
                    self.published_stop = True
                rate.sleep()

if __name__ == "__main__":
    rospy.init_node("ignisbot_tailwag_node", log_level=rospy.INFO)
    tailwag_obj = TailWag()
    tailwag_obj.start_loop()
