# pingpongbot_control

ROS2 package for controlling the pingpongbot.

This packages handes the kinematics, odometry updates, and PID control necessary for operating the robot.

## Nodes
- `commander` - This node subscribes to the `/ball_pos` topic. It takes the position and determines a suitable goal_pose for the robot and publishes it as a PoseStamped message to the `/goal_pose` topic.
- `controller` - This node subscribes to the `/goal_pose` topic. It compares this goal pose with the current robot position and generates a Twist message using a proportional controller (integral and derivative may be added later) that is published to the `/cmd_vel` topic.
- `jointStateUpdate` - This node subscribes to the `/wheel_angles` and `/wheel_speeds` topics and publishes to the `/joint_states` so the robot's joints update accordingly in rviz.
- `odomUpdate` - This node subscribes to the `/wheel_angles` and `/cmd_vel` topics. It uses the wheel angles to update the transform between the odom frame and the base_footprint, and converts the twist message to wheel speeds, which is then published to the `/wheel_speeds` topic. The kinematics and odometry calculations completed by this node are based on the equations below from "Modern Robotics: Mechanics, Planning, and Control" by Kevin M. Lynch and Frank C. Park.
<center><img src="https://github.com/user-attachments/assets/497bb22f-d664-4b97-b608-4d714f2c75ba" width="600"/></center>
<center><img src="https://github.com/user-attachments/assets/875af214-bc25-4129-842a-63c907995127" width="600"/></center>


## Launch Files
`control.launch.xml` - This file launches the `commander`, `controller`, `odomUpdate`, and `jointStateUpdate` nodes, each with their own yaml files to specify necessary parameters.