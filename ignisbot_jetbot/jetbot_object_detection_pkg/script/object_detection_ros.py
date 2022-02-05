#!/usr/bin/env python3
import sys
import time
# ROS imports
import rospy

# Object detection imports
from jetbot_pkg.object_detection import ObjectDetector




class IgnisBotObjectDetector(object):

    def __init__(self, object_to_track="", simulated_camera=False):

        # Import Object detection model
        self.model = ObjectDetector('ssd_mobilenet_v2_coco.engine')


    def start_object_follow_loop(self):

        while not rospy.is_shutdown():
            self.object_search()


    def object_search(self):

        # TODO: This is a dummy method
        time.sleep(0.5)


if __name__ == "__main__":
    rospy.init_node("object_detection_test_node", log_level=rospy.INFO)
    if len(sys.argv) < 2:
        print("usage: object_detection_ros.py object_to_track")
    else:
        object_to_track = str(sys.argv[1])

        ot_object = IgnisBotObjectDetector(object_to_track=object_to_track, simulated_camera=True)
        ot_object.start_object_follow_loop()
