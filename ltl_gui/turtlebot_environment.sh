#!/bin/bash

shell=`basename $SHELL`
echo "Activating ROS with shell: $SHELL"
source /opt/ros/indigo/setup.$shell

export ROS_HOSTNAME=12.0.5.1
echo "Set ROS_HOSTNAME to: $ROS_HOSTNAME"

echo "Activating development environment..."
source ~/tiago_public_ws/devel/setup.$shell

export TURTLEBOT_BASE=kobuki
echo "TURTLEBOT_BASE set to $TURTLEBOT_BASE"

export ROS_MASTER_URI=http://12.0.5.1:11311
echo "ROS_MASTER_URI set to $ROS_MASTER_URI"

export ROS_IP=12.0.5.1
echo "ROS_IP set to $ROS_IP"

export TURTLEBOT_STACKS=hexagons
echo "TURTLEBOT_STACKS set to $TURTLEBOT_STACKS"
export TURTLEBOT_3D_SENSOR=astra
echo "TURTLEBOT_3D_SENSOR set to $TURTLEBOT_3D_SENSOR"

exec "$@" #Passes arguments. Need this for ROS remote launching to work.
