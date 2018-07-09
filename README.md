# Package `rqt_simulation_gui` {#rqt_simulation_gui}

In this package a Graphical User Interface (GUI) for the [LTL-planner](https://github.com/MengGuo/P_MAS_TG) is provided.

## Installation

At the moment the supported robots are [Turtlebot](http://wiki.ros.org/turtlebot/Tutorials/indigo) and [TIAGo](http://wiki.ros.org/Robots/TIAGo). Install at least one of them to use the GUI.
Simulations are performed in Gazebo. Install the Gazebo distribution.

If the software for one robot has been installed the rqt_simulation_gui package can be build by:

Go to your catkin workspace:

        $ cd ![catkin_ws]

Source your workspace:  

        $ source devel/setup.bash

Build your workspace:

        $ catkin build

## Start the GUI

Go to your catkin workspace, source it and start a roscore:

        $ cd <catkin_ws>
        $ source devel/setup.bash
        $ roscore

In as second terminal window also go to the workspace, source it and start rqt:

        $ cd <catkin_ws>
        $ source devel/setup.bash
        $ rqt

Click on the top left corner on Plugins and choose simulation.

## Setup

### Worlds

Choose a world in the GUI.
Worlds can be added by:
1. Appending the list in the file [gui_config.yaml](rqt_simulation/config/gui_config.yaml)

2. Add a folder with <world_name> [here](rqt_simulation/scenario). The folder contains an image from the map (map.png) and the correspoding parameterfile (map.yaml). You must follow the naming conventions of the files to load them successfully in the GUI.

3. If you use the world for simulations, add the Gazebo world file [here](rqt_simulation/worlds/gazebo). Again the file must be named as <world_name>.worlds

### Choose FTS

After choosing your FTS the FTS is saved in the file [env_GUI.yaml](rqt_simulation/config/FTS/env_GUI.yaml) and can be loaded afterwards.

### Agent specifications

Select and agent type in the combo box.
A new agent type is added by appending the robot model in the file [gui_config.yaml](rqt_simulation/config/gui_config.yaml). Do not forget to specify if it is a ground robot or an aerial vehicle.
The corresponding launch files need  to be added in [this](rqt_simulation/launch), where all robots for simulation are launched over [robot.launch](rqt_simulation/launch/robot.launch) and for experiments over [robot_exp.launch](rqt_simulation/launch/robot_exp.launch).

The robot name must match the Qualisys name of the robot model if you want to use the motion capture system.

It is not possible multiple robots at the same initial position.

### Task specifications

Write hard and soft task in the line.

### Synthesize plan

After synthesizing the plan, the robot and task specifications are added to the file [gui_config.yaml](rqt_simulation/config/gui_config.yaml) and can be reloaded afterwards.

## Setup experiment

Assume a roscore has already been started successfully.
Go to your catkin workspace and source the turtlebot environment with the file [turtlebot_environment.sh](turtlebot_environment.sh) and start rqt:

        $ cd <catkin_ws>
        $ source src/rqt_simulation_gui/turtlebot_environment.sh
        $ rqt

Login to the on-board computer of a Turtlebot:

        $ ssh <username>@<IP_addresse>

The username and IP_addresse are written on the Turtelbot. The Turtlebot and ROS environment should be already sourced by the login.
Start the Kobuki base and astra camera driver:

        $ roslaunch rqt_simulation turtlebot_base_astra.launch robot_name:=<robot_name> use_qualisys:=<Bool>

The argument <robot_name> must match the chosen robot name in the GUI. Set the argument <Bool> to True if you want to use the Qualisys for localization. The argument <Bool> is set to False by default. Robot model name on the Qualisys computer, the robot name in the GUI and the argument <robot_name> must match.

For starting Qualisys start the Qualisys node in a new terminal window (not on the Turtlebot):

        $ cd <catkin_ws>
        $ source src/rqt_simulation_gui/turtlebot_environment.sh
        $ roslaunch qualisys qualisys.launch

If everything is set up press the button "Setup experiment" in the GUI.

## Data logging

By pressing the button "Record data" the robot trajectories and temporary task are saved [here](rqt_simulation/logging).
