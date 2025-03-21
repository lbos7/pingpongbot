# pingpongbot_driver

ROS2 package for interacting with the robot hardware.

This package contains a node that runs on the Raspberry Pi 5 to 

## Nodes
`driver` - This node subscribes to the `/wheel_speeds` topic to command the motors and publishes to the `/wheel_angles` and `/imu/data_raw` topics so the robot odometry can be updated.


## Launch Files
`driver.launch.xml` - This file launches the `driver` node with the necessary parameters set using the `params.yaml` file in the `config` directory.