# Hybrid Controller 
(`hybrid_controller` and `formula_parser` packages)
This is an implementation of the method described in https://arxiv.org/pdf/1709.07723.pdf (L. Lindemann, D. Dimarogonas; "Decentralized Robust Control of Coupled Multi-Agent Systems under Local Signal Temporal Logic Tasks")

## Prerequisites
Ubuntu 14.04 and ROS Indigo are recommended.

### Armadillo
Armadillo is needed for linear algebra and vector operations. It was not extensively used in the project and could be rid of in the future. The library is dependent on several packages which need to be downloaded first.

        $ sudo apt-get install cmake libopenblas-dev liblapack-dev

Download the code from http://arma.sourceforge.net/download.html. Then, navigate to where it was saved and unzip.

        $ cd <the_unzipped_folder>
        $ cmake .
        $ make
        $ sudo make install

The detailed instuction, as well as installation instructions for other operating systems, can be found under https://solarianprogrammer.com/2017/03/24/getting-started-armadillo-cpp-linear-algebra-windows-mac-linux/ . The version of the library installed should be the same as the one in the repo. Alternatively, a more recent version can be installed, but then it must also be upated in the repo.

## Installation

Clone this repository to your workspace. Also, download the `qualisys` package from https://github.com/KTH-SML/qualisys and clone the repositories containing the packages `plotter` (for plotting the results) and `robot` (for simulation). Build the workspace:

        $ cd catkin_ws
        $ source devel/setup.bash
        $ catkin_make

On each Nexus computer, there should be a ROS package `robot` containing a launch file `robot.launch`:
```
<launch>
    <node name="rossserial<x>" pkg="rosserial_python" type="serial_node.py">
        <remap from="/cmdvel" to="/cmdvel<x>"/>
    </node>
</launch>
```
The robot is paired with its controller through <x>. The workspace on the Nexus must be built as well.


## Deployment

Make sure the parameter ROS_MASTER_URI is set to the IP of the computer which the roscore is going to be run on; ROS_IP should be set to the local address. It is convenient to set it inside the `.bashrc` file:

    $ nano ~/.bashrc
    
and inside:

    $ export ROS_MASTER_URI=http://<master_ip_address>:11311
    $ export ROS_IP=<local_ip>

Start the roscore:

        $ roscore
        
If in simulation mode, start dummy robot nodes:

        $ roslaunch hybrid_controller robot.launch
        
If in physical experiment mode, connect to the Nexus robots' on-board computers

        $ ssh nexus@12.0.4.x
        
where x is 1, 2 or 3 (there are three robots with static IP addresses; should the IPs change, consult Pedro Roque). The password can be obtained from Pedro Roque. To start the nodes, on each Nexus do:

        $ roslaunch robot robot.launch
        
Start one instance of the `qualisys` node (on your computer, not on Nexus) and an instance of the `qualisys_odom` for each robot, with the `model` argument matching the name of the rigid body defined in the Qualisys Track Manager. 

Start the hybrid controller nodes for each robot with appropriately remapped topics. The configurations files containing the STL formulas should be loaded into the ROS Parameter Server. See the hybrid_controller/launch/test.launch for reference.

### Recording messages and plotting
The experiments can be recorded by calling

    $ roslaunch hybrid_controller record.launch
    
and played back with

    $ roslaunch hybrid_controller playback.launch
    
which wrap rosbag's functions. The `plotter` nodes can be started by

    $ roslaunch hybrid_controller plotter.launch
    
to generate robustness plots for each agent. They should be started before the rosbag is played back in order to avoid missing messages at the beginning. Upon exiting the `plotter` node, the resultant plots can be found in the main directory of the `plotter` package.

### Parameters
A number of parameters are defined inside the launch files for the controllers. To understand the meaning of the parameters, read https://arxiv.org/pdf/1709.07723.pdf (variable names match).