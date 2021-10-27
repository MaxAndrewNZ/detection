#!/usr/bin/env python3

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


def create_detection_markers(detections): 
    frame = detections.header.frame_id
    marker_array =  MarkerArray()
    for detection in detections.detections:
        marker = create_detection_marker(detection, frame)
        marker_array.markers.append(marker)

    for i in range(0, len(marker_array.markers)):
        marker_array.markers[i].id = i

    return marker_array


def create_detection_marker(detection, frame):
    marker = Marker()
    marker.header.frame_id = frame
    marker.type = marker.LINE_STRIP
    marker.action = marker.ADD

    # marker scale
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05

    # marker color
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0

    # marker orientation
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    # marker position
    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.0

    # marker line points
    marker.points = []

    lower = detection.box.lower
    upper = detection.box.upper

    # first point
    first_line_point = Point()
    first_line_point.x = lower.x
    first_line_point.y = lower.y
    first_line_point.z = lower.z
    marker.points.append(first_line_point)

    # second point
    second_line_point = Point()
    second_line_point.x = upper.x
    second_line_point.y = upper.y
    second_line_point.z = upper.z
    marker.points.append(second_line_point)

    return marker