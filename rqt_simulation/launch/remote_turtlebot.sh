 #!/bin/bash 
ROBOT_NAME=$1
roslaunch rqt_simulation turtlebot_base_astra.launch robot_name:=$ROBOT_NAME
