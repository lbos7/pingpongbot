# pingpongbot

Author: Logan Boswell

<p align="center">
  <img src="https://github.com/user-attachments/assets/16626542-416e-4f46-9ec2-85a44c1898e3" width="600"/>
</p>

This is a 10 week project with the ideal goal of building a omnidirectional robot from scratch that is capable of returning ping pong balls to a player. The idea behind this project is that this robot would sit on the opposite side of a ping pong table from a player and move accordingly to return the balls. Within this timeframe, my objective was to achieve the most effective level of functionality possible—whether that meant reliably returning the balls or at least tracking their movement and responding accordingly.

## Packages
- `pingpongbot_bringup` - contains the necessary launchfile to run on the host computer for operating the robot
- `pingpongbot_control` - contains the nodes for controlling the robot - includes handling kinematics, odometry updates, and PID control
- `pingpongbot_description` - robot definition, urdf, meshes
- `pingpongbot_driver` - contains the nodes that run on the Raspberry Pi to interact with hardware
- `pingpongbot_msgs` - custom messages for operation
- `pingpongbot_vision` - conatins the nodes for pingpongbot computer vision functionality

## Operation

#### Physical Setup
To prepare the system for operation, the following components are necessary: host computer running ROS2 Jazzy, the physical robot using a Raspberry Pi 5 running ROS2 Jazzy, Intel RealSense D435 camera, the two apriltags used for setting up the arena.

#### ROS2 Setup
In order to operate the system, clone this repository in a `ros2_ws/src/` directory on both the host computer and the Raspberry Pi with ROS2 Jazzy installed and make sure that the following repos are installed: 
- https://github.com/IntelRealSense/realsense-ros
- https://github.com/christianrauch/apriltag_ros

#### Building

Building on the Pi:

`colcon build --packages-select pingpongbot_driver pingpongbot_msgs`

Building on the Host Computer:

`colcon build`

#### Launching
After the packages have been built on both the Pi and host computer, make sure both devices are on the same wifi network and ssh into the Pi from the host computer. If the physical setup has been completed and the realsense is connected to the host computer, the system is ready for operation.

On the Pi:

`ros2 launch pingpongbot_driver driver.launch.xml`

On the Host Computer:

`ros2 launch pingpongbot_bringup bringup.launch.xml`

Refer to the READMEs for specific packages if there are questions regarding specific nodes, launchfiles, etc.