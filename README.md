# IHA-4506 Advanced Robotics 2020
We'll use [[4]](https://github.com/modulabs/arm-control) for implementing and demonstrating the course assignments.  

## Prerequisit
It is recommended to use the following requirements to avoid any unforseen error during the implementation of the codes.

- Ubuntu 18 or newer
- ROS Melodic or newer

## VMWare (Optional)
If you are using windows or run an older version of ubuntu on your PC, you can use VMWare Workstation Player to run ubuntu 18 which is free for non-commercial use. In that case, follow the instructions below to install and prepare you virtual machine:

- depending on your current version of operating system, download one of the installers from [VMWare Installers](https://www.vmware.com/fi/products/workstation-player/workstation-player-evaluation.html).
- on windows, just run the installer GUI and follow the instructions
- on Ubuntu install it from command lines:
```
$ cd "PATH/TO/DOWNLOADED/FOLDER"  #change directory to where you have downloaded the .bundle (installer) file
$ sudo sh VMware-Player-15.5.6-16341506.x86_64.bundle  #change .bundle file name if necessary
```
- Download an ubuntu .iso image from [Ubuntu 18 ISO file](https://releases.ubuntu.com/bionic/ubuntu-18.04.5-desktop-amd64.iso)
- open VMWare Workstation Player and create an new environment from the ISO file and set the hardware properties as you wish.

## ROS melodic
To avoid any possible incompatibility issue, please install ROS melodic from [ROS Melodic installation guide](http://wiki.ros.org/melodic/Installation/Ubuntu)

## Installation
The github repository provided here is the modified version of [4] that some of its bugs are fixed and we'll maintain it during the course implementation.

### Install gazebo-ros-pkgs and gazebo-ros-control

    $ sudo apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control

### Install effort-controllers to use torque-control interface

    $ sudo apt-get install ros-melodic-effort-controllers

### Download and build 

    $ mkdir -p catkin_ws/src
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/AdvancedRobotics-iha4506/ElfinSimulation.git
    $ cd ~/catkin_ws/
    $ catkin_make

### Ros Noetic (Ubuntu 20.04)
Running the examples in ROS Noetic is also possible. The master branch does not work directly with noetic, some slight modifications are required in the launch files.

Under the 'elfin/elfin_gazebo/launch' folder, find the line in all launch files

`<param name="robot_description" command="$(find xacro)/xacro.py --inorder '$(find elfin_description)/urdf/elfin3.urdf.xacro'" />`

and modify `xacro.py` to `xacro`, as 

`<param name="robot_description" command="$(find xacro)/xacro --inorder '$(find elfin_description)/urdf/elfin3.urdf.xacro'" />`

Additionally, in the files `elfin3_empty_world.launch` and `elfin3_no_fric_no_joint_limit_world.launch`, find the line

`<node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher" ns="/elfin"/>`

and modify it as

`<node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" ns="/elfin"/>`


### Run examples
Motion controllers in joint space

    $ roslaunch elfin_gazebo elfin3_empty_world.launch controller:=gravity_comp_controller
    or
    $ roslaunch elfin_gazebo elfin3_empty_world.launch controller:=time_delay_controller
    or
    $ roslaunch elfin_gazebo elfin3_empty_world.launch controller:=computed_torque_controller
    or
    $ roslaunch elfin_gazebo elfin3_empty_world.launch controller:=computed_torque_clik_controller

If you want to use motion and force controller in task space, then you may choose this controllers as follows:

    $ roslaunch elfin_gazebo elfin3_experiment1_world.launch controller:=adaptive_impedance_controller
    or
    $ roslaunch elfin_gazebo elfin3_experiment2_world.launch controller:=adaptive_impedance_controller

If you want to plot data in rqt graph, use rqt_plot.launch file. Customize perspective files to plot data you need.

    $ roslaunch rqt_plot.launch controller:=gravity_comp_controller



## Notes
- If you are using VMWare and gazebo keeps crashing during run time, try the following:

```
$ echo "export SVGA_VGPU10=0" >> ~/.bashrc
``` 
- before running any of the sample codes, you also need to issue the following command from your /catkin_ws directory so the built nodes and launch files could be found in terminal:


```
$ source devel/setup.bash
```
## References
1. [ros_Control](http://wiki.ros.org/ros_control)
2. [Write a new ros-controller](https://github.com/ros-controls/ros_control/wiki/controller_interface)
3. [Elfin Robot](http://wiki.ros.org/Robots/Elfin)
4. [Elfin Simulation package](https://github.com/modulabs/arm-control)
5. [ROS](http://wiki.ros.org/)

