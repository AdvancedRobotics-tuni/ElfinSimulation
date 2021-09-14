# AUT-720 Advanced Robotics 2021-2022
This repository provides an example of a very simple reactive controller. The main focus is to learn some ros functionalities that helps students with future implementations by:
- cloning and starting to implement your own controller
- learning how to communicate between different nodes using ros subscriber/publisher
- how to coordinate robot actions in case you have different controllers implemented (switching between task space and joint space for example and etc.)

## Prerequisit
For this excercise students must have already done setting up their ubuntu/ros and are familiar with launching the elfin simulation.

## Example intro
This example utilizes ros control services and ros subscriber/publisher to make a network of different nodes and coordinate the actions of the robot.
- two nodes are cloned where the first one is gravity_comp_controller that controls the motion in joint space, and also the second controller is cloned from computed_torque_clik_controller
that works in task space.
- in addition to these separate nodes, a controller switcher node is implemented that is able to:
1. load, stop and start controllers using ros control services
2. communicate with arm controller using ros topic messages

### Clone the repository
in order to run the examples, clone this repository in some other directory than before and run catkin_make again:
```
$ mkdir -p /another_ws/src
$ cd another_ws/src
$ git clone https://github.com/AdvancedRobotics-tuni/ElfinSimulation.git
$ cd ElfinSimulation
$ git checkout reactive_control_example
$ cd ../..
$ catkin_make
$ source devel/setup.bash

```

### How to run
1. first run the simulation and the first controller using the command:
```
$ roslaunch elfin_gazebo elfin3_empty_world.launch controller:=cloned_clik_controller_1
```
2. go to the directory of switcher node:
```
$ cd ~/another_ws/src/ElfinSimulation/arm_controllers/custom_script

```
2. make the code executable:
```
$ chmod +x controller_switcher.py

```
3. run the code:

```
$ ./controller_switcher.py

```

The program then asks for a number from the user that corresponds to the cloned controllers:
* 1 is to switch to cloned_clik_controller_1 and 2 is for switching to cloned_gravity. however you can later change these according to your need.
* before switching to any controller, the code loads the second controller first!
* the code is continuesly listenning to a ros topic called /rqt_command_listener. you can use any method (use rqt topic publisher for example) to send messages over that topic that is being published directly to set target ee pose of elfin robot (only for cloned_clik_controller_1 controller).

## About the repository
This repository is a clone of course github's master branch. To be able to run the example, the followings are changed:

### Two new arm controllers are cloned
read the instructions on [how to add new controller](https://github.com/AdvancedRobotics-tuni/ElfinSimulation/tree/master/arm_controllers#readme) to clone the followings:

* gravity_comp_controller -> cloned_gravity.
* computed_torque_clik_controller ->  cloned_clik_controller_1.

### Custom yaml file
in addition to steps above, you'll need to create a new config file that include both controller's initial parameters:
find switch_controllers.yaml in config folder of elfin gazebo and see how it is implemented. It is worth mentioning that the PID gains that are set in this config file are not optimal and they are only set here to have some stable behaviour.

### Custom launch file
a new launch file is added to load custom config file and run only the first controller at the launch time.

to run simulation:

```
$ roslaunch elfin_gazebo controller_switcher_example.launch controller:=cloned_clik_controller_1
```

### Controller switcher script
A python script is provided in ElfinSimulation/Arm_Controllers/custom_scripts that provide some functions to load, start and stop the controllers and also to communicate with controller nodes through ros topics. The same ros functionalities are also available in C++ so if you are more familiar with C++, you can implement your own service clients there using the following instructions:

* [ros subscriber and publisher in C++](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)
* [ros service and client in C++](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29)





