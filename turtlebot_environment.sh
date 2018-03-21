#!/bin/bash


shell=`basename $SHELL`
echo "Activating ROS with shell: $SHELL"
source /opt/ros/indigo/setup.$shell

export ROS_HOSTNAME=$HOSTNAME
echo "Set ROS_HOSTNAME to: $ROS_HOSTNAME"

echo "Activating development environment..."
source ~/tiago_public_ws/devel/setup.$shell

export TURTLEBOT_BASE=kobuki
echo "TURTLEBOT_BASE set to $TURTLEBOT_BASE"

export ROS_MASTER_URI=http://192.168.221.:11311
echo "ROS_MASTER_URI set to $ROS_MASTER_URI"

echo "TURTLEBOT_BASE set to $TURTLEBOT_BASE"
export TURTLEBOT_STACKS=hexagons
echo "TURTLEBOT_STACKS set to $TURTLEBOT_STACKS"
export TURTLEBOT_3D_SENSOR=astra
echo "TURTLEBOT_3D_SENSOR set to $TURTLEBOT_3D_SENSOR"

exec "$@" #Passes arguments. Need this for ROS remote launching to work.
