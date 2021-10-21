#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from detection_msgs.msg import DetectionArray, Detection
import numpy as np


class ControlFromDetection:
    def __init__(self):
        rospy.init_node('control_from_detection')

        self.rate = rospy.get_param("~rate", 10)
        self.stopping_distance = rospy.get_param("~stopping_distance", 2.0)

        self.velocity_publisher = rospy.Publisher(rospy.get_param("~velocity_topic", "/cmd_vel_mux/detection_control"), Twist, queue_size=10)
        self.detection_subscriber = rospy.Subscriber(rospy.get_param("~detection_topic", "/detection"), DetectionArray, self.update_detection, queue_size=10)

        self.detections = None

    def update_detection(self, detections):
        print("new detections:", detections)
        self.detections = detections

    def stop(self):
        vel_msg = Twist()   
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.velocity_publisher.publish(vel_msg)
        print("Stopping robot.")
    
    def closest_distance_to_detection(self):
        #TODO: get closest detection distance from detection array.
        closest_distance = None
        # print(self.detections)

        return closest_distance

    def check_collision(self):
        distance_to_detection = self.closest_distance_to_detection()
        if distance_to_detection and distance_to_detection <= self.stopping_distance:
            self.stop()


def start_listeners(cfd):
    while not rospy.is_shutdown():
        cfd.check_collision()
        rospy.Rate(cfd.rate)


def main():
    cfd = ControlFromDetection()
    start_listeners(cfd)
    

if __name__ == '__main__':
    main()