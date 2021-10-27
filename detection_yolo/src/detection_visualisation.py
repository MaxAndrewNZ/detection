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
    marker.type = marker.LINE_LIST
    marker.action = marker.ADD
    
    marker.lifetime = 1

    # marker scale
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05

    # marker color
    marker.color.a = 0.8
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

    L = detection.box.lower
    upper = detection.box.upper

    p0 = L
    p1 = Point(L.x, U.y, L.z)
    p2 = Point(L.x, U.y, U.z)
    p3 = Point(L.x, L.y, U.z)
    p4 = Point(U.x, L.y, L.z)
    p5 = Point(U.x, U.y, L.z)
    p6 = U
    p7 = Point(U.x, L.y, U.z)

    points = [p0, p1, p2, p3, p4, p5, p6, p7]

    for i in range(0, len(points)-1):
        start = points[i]
        end = points[i+1]
        marker.points.append(start)
        marker.points.append(end)

    return marker