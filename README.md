# 3DOF Robotic Arm Control with ROS and Gazebo
Playing around with ROS Control with a simple 3DOF arm.

## Installation
Clone the repository into your catkin workspace, run rosdep to install dependencies, and finally run catkin_make in the base workspace folder.

## Usage
To launch the robot in Gazebo and start all control related nodes, run the command:\
`roslaunch arm_control arm.launch`\

To make the arm move the end effector in a circular trajectory, run the command:\
`rosrun arm_control move_circle`
