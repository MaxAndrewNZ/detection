#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


def create_detection_boxes(detections): 
    frame = detections.header.frame_id
    box_array = MarkerArray()
    for detection in detections.detections:
        box = create_detection_box(detection, frame)
        box_array.markers.append(box)

    for i in range(0, len(box_array.markers)):
        box_array.markers[i].id = i

    return box_array


def create_detection_box(detection, frame):
    marker = Marker()
    marker.header.frame_id = frame
    marker.type = marker.CUBE
    marker.action = marker.ADD
    
    marker.lifetime = rospy.rostime.Duration(1)

    L = detection.box.lower
    U = detection.box.upper

    center_point = Point(
        L.x + U.x / 2,
        L.y + U.y / 2,
        L.z + U.z / 2,
    )

    width = abs(L.x - U.x)
    height = abs(L.y - U.y)
    depth = abs(L.z - U.z)

    # marker scale
    marker.scale.x = width
    marker.scale.y = height
    marker.scale.z = depth

    # marker color
    marker.color.a = 0.7
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0

    # marker orientation
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    # marker position
    marker.pose.position.x = center_point.x
    marker.pose.position.y = center_point.y
    marker.pose.position.z = center_point.z

    return marker