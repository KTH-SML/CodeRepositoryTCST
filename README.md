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

        $ cd ![catkin_ws]
        $ source devel/setup.bash
        $ roscore

In as second terminal window also go to the workspace, source it and start rqt:

        $ cd ![catkin_ws]
        $ source devel/setup.bash
        $ roscore

Click on the top left corner on Plugins and choose simulation.
