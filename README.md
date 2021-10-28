# Detection

## Getting Started

- Follow the [README](detection_yolo/README.md) in detection_yolo to install YOLO detection dependencies.
- Update the `serial_number` argument in `detection_control/launch/detection_full.launch` to that of your D435 camera.


## Building

- First build your workspace using `catkin build`.
- Ensure that the workspace is sourced correctly using `source devel/setup.bash`.


## Launching

`roslaunch detection_control detection_full.launch`

- This will launch the D435 camera, Rviz, YOLO detection, and the robot control node.
- Each of these nodes can be toggled through the arguments in the launch file.


## Robot Control Node

- The robot control node is setup to publish a Twist message with 0 velocities when a human is detected within the stopping distance.
- This command will be published each time this event occurs.
- A command velocity multiplexer (MUX) is recommended with use of this node.
- A MUX will allow this message to be set with a high priority and long timeout. This will override commands from other velocity publishers for the set timeout duration.
- There are alternatives to a MUX that may be more appropriate for your applications requirements.