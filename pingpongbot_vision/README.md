# pingpongbot_vision

ROS2 package for pingpongbot computer vision functionality.

## Nodes
- `tracker` - This node tracks the position of an orange ping pong ball and publishes the position of the ball as a PointStamped message to the `ball_pos` topic.
- `arena` - This node sets up an arena in RVIZ to represent the half of the ping pong table where the robot is located. It broadcasts transforms between apriltags, markers representing the borders of the table, the camera, and the robot.

## Launch Files
`vision.launch.xml` - This file launches the `tracker` and `arena` nodes as well as a node for identifing apriltags and a launchfile for starting the realsense camera.