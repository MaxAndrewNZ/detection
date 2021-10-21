# ROS Yolo Detection

The package subscribes to an RGB and Depth stream and outputs an array of detections.

## Dependencies

### Python Packages

`pip3 install -r dependencies.txt`

### [Darknet ROS](https://github.com/MaxAndrewNZ/darknet_ros)

`git clone --recursive git@github.com:MaxAndrewNZ/darknet_ros.git`

## Running

### Build and Source

1. `catkin build`
2. `source devel/setup.bash`

## Using Bags

1. `rosbag play {bag_name} --clock --loop`
2. `rosparam set use_sim_time true`

## Configuration

### Bounding boxes to 3D Detections

`config/bounding_box_to_cuboid_params.yaml`

- average distance percentile
- bounding box topic
- depth image topic
- camera intrinsics topic
- detection publishing topic

### Darknet

`config/darknet_ros_config.yaml`

- RGB image topic

### Yolo

`config/yolov3.yaml` 

- Detection classes 
- Required confidence for detection 
