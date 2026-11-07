#!/usr/bin/env bash
# colcon does not search inside a package, and Robotics_Project has its own
# package.xml, so we point it at panda_kinematics explicitly.
source /opt/ros/jazzy/setup.bash
cd "$HOME/project_name" || exit 1
colcon build --base-paths src/mujoco_ros2 src/Robotics_Project/RPC_1/panda_kinematics "$@"
