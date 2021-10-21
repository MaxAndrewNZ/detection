#!/usr/bin/env python3

from os import path

import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from detection_msgs.msg import Box, Detection, DetectionArray
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
import tf
import numpy as np
from cv_bridge import CvBridge
import message_filters


def convert_depth_pixel_to_point(depth, pixel_x, pixel_y, camera_intrinsics):
    # TODO: Check these are the correct values
    fx = camera_intrinsics.K[0]
    fy = camera_intrinsics.K[4]
    ppx = camera_intrinsics.K[2]
    ppy = camera_intrinsics.K[5]

    X = (pixel_x - ppx) / fx * depth
    Y = (pixel_y - ppy) / fy * depth

    new_point = Point(X, Y, depth)

    return new_point


def get_cuboid(distance_to_object, xmin, ymin, xmax, ymax, camera_intrinsics):
    lower_point = convert_depth_pixel_to_point(distance_to_object, xmin, ymin, camera_intrinsics)
    upper_point = convert_depth_pixel_to_point(distance_to_object, xmax, ymax, camera_intrinsics)

    radius = abs(upper_point.x - lower_point.x) / 2
    upper_point.z += radius
    lower_point.z -= radius

    cuboid = Box(lower_point, upper_point)

    return cuboid


def create_detection_cuboid(object_box, cv_depth_image, camera_intrinsics):
    cropped_depth_image = cv_depth_image[object_box.ymin:object_box.ymax, object_box.xmin:object_box.xmax]
    depth_array = np.array(cropped_depth_image, dtype = np.float32)
    depth_array = depth_array.flatten()

    percentile = rospy.get_param("/bounding_box_to_cuboid/average_distance_percentile")

    depth_array = depth_array[depth_array != 0] 
    if len(depth_array) > 0:  
        nth_smallest_ind = int(len(depth_array) * percentile) 
        distance_to_object = np.partition(depth_array, nth_smallest_ind)[nth_smallest_ind] / 1000
        rospy.logdebug(("Object detected {distance} m away.").format(distance = str(distance_to_object)))

        detection = Detection()
        detection.box = get_cuboid(distance_to_object, object_box.xmin, object_box.ymin,
                                            object_box.xmax, object_box.ymax, camera_intrinsics)
        detection.score = object_box.probability
        detection.label = object_box.Class

        return detection
    
    else:
        return None


def process_bounding_boxes(bounding_boxes, depth_image, camera_intrinsics, cuboid_publisher):
    detection_cuboids = []
    cv_bridge = CvBridge()
    cv_depth_image = cv_bridge.imgmsg_to_cv2(depth_image, desired_encoding="passthrough")

    for object_box in bounding_boxes.bounding_boxes:
        detected_cuboid = create_detection_cuboid(object_box, cv_depth_image, camera_intrinsics)
        if detected_cuboid:
            detection_cuboids.append(detected_cuboid)

    detection_array = DetectionArray()
    detection_array.header = depth_image.header
    detection_array.detections = detection_cuboids

    cuboid_publisher.publish(detection_array)


def start_listeners():
    box_sub = message_filters.Subscriber(rospy.get_param("/bounding_box_to_cuboid/bounding_box_topic"), BoundingBoxes)
    depth_image_sub = message_filters.Subscriber(rospy.get_param("/bounding_box_to_cuboid/depth_image_topic"), Image)

    ts = message_filters.ApproximateTimeSynchronizer([box_sub, depth_image_sub], 1, 1)

    cuboid_publisher = rospy.Publisher(rospy.get_param("/bounding_box_to_cuboid/publish_topic"), DetectionArray)
    camera_intrinsics = rospy.wait_for_message(rospy.get_param("/bounding_box_to_cuboid/camera_intrinsics_topic"), CameraInfo)

    ts.registerCallback(process_bounding_boxes, camera_intrinsics, cuboid_publisher)

    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('bounding_box_to_cuboid')

    while not rospy.is_shutdown():
        try:
            start_listeners()
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue