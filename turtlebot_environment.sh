#!/bin/bash
echo "Setting TurtleBot environment..."

shell=`basename $SHELL`
echo "Activating ROS with shell: $SHELL"
source /opt/ros/indigokinetic/setup.$shell

if [ $# -gt 0 ]; then
	# provided a IP, use it as ROS_MASTER_URI
	export ROS_MASTER_URI=http://$1:11311/
	export ROS_HOSTNAME=$1
	export ROS_IP=$1
else
	echo "No hostname provided. Using $HOSTNAME."
	export ROS_MASTER_URI=http://$HOSTNAME:11311/
	export ROS_HOSTNAME=$HOSTNAME
fi

echo "ROS_MASTER_URI set to $ROS_MASTER_URI"
echo "Set ROS_HOSTNAME to: $ROS_HOSTNAME"
echo "Set ROS_IP to: $ROS_IP"

echo "Activating development environment..."
source ~/tiago_public_ws/devel/setup.$shell

export TURTLEBOT_BASE=kobuki
echo "TURTLEBOT_BASE set to $TURTLEBOT_BASE"

export TURTLEBOT_STACKS=hexagons
echo "TURTLEBOT_STACKS set to $TURTLEBOT_STACKS"

export TURTLEBOT_3D_SENSOR=astra
echo "TURTLEBOT_3D_SENSOR set to $TURTLEBOT_3D_SENSOR"

exec "$@" #Passes arguments. Need this for ROS remote launching to work.
