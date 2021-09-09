# Adding new controllers

Depending on the implementation, a separate controller to be used for the arm controllers can be added as a plugin to Ros. As an example, creation of a copy of gravity_comp_controller will be explained.

Firstly, create a copy of the `gravity_comp_controller.cpp` and rename the copy as `gravity_comp_alternative_controller.cpp`.

## Modify the class names in the source code
Firstly, modification of the class name in the new controller is necessary. Originally, the class name is set to `GravityCompController`. Find each mention of these and modify it as `GravityCompAlternativeController`. For instance, the beginning description
`namespace arm_controllers{

	class GravityCompController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
	{
		public:
		~GravityCompController() 
		{
			command_sub_.shutdown();
		}`

will be modified as 
`namespace arm_controllers{

	class GravityCompAlternativeController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
	{
		public:
		~GravityCompAlternativeController() 
		{
			command_sub_.shutdown();
		}`

and the callback reference for the subscription 

```
command_sub_ = n.subscribe<std_msgs::Float64MultiArray>("command", 1, &GravityCompController::commandCB, this);
```

as

```
command_sub_ = n.subscribe<std_msgs::Float64MultiArray>("command", 1, &GravityCompAlternativeController::commandCB, this);
```

Finally, this controller class will be exported as class with the line 
```
PLUGINLIB_EXPORT_CLASS(arm_controllers::GravityCompController, controller_interface::ControllerBase)

```
and modify it similar to before as 
Finally, this controller class will be exported as class with the line 
```
PLUGINLIB_EXPORT_CLASS(arm_controllers::GravityCompAlternativeController, controller_interface::ControllerBase)

```

## Modify other files for compiling the new controller
### Plugin defition file: `controller_plugins.xml`
The plugins for defining the exported plugins to Ros is defined in the file `controller_plugins.xml`. The original controller is given as
```
  <class name="arm_controllers/AdaptiveImpedanceController" type="arm_controllers::AdaptiveImpedanceController" base_class_type="controller_interface::ControllerBase">
    <description>
	    Adaptive Impedance Controller
    </description>
  </class>
```

Create another class entry in this file with the code block;
```
  <class name="arm_controllers/GravityCompAlternativeController" type="arm_controllers::GravityCompAlternativeController" base_class_type="controller_interface::ControllerBase">
    <description>
	    Adaptive Impedance Alternative Controller 
    </description>
  </class>
```
### Compilation file: `CMakeLists.txt`
The source code needs to be added to CMake files in order to be compiled. Find the `add_library` function with other controllers added and add the new source code name into the list as,
```
add_library(${PROJECT_NAME}
  src/time_delay_controller.cpp
  src/gravity_comp_controller.cpp
  src/gravity_comp_alternative_controller.cpp
  src/computed_torque_controller.cpp
  src/computed_torque_clik_controller.cpp
  src/adaptive_variable_impedance_controller.cpp
  src/passivity_controller.cpp
)
```

### Configuration file (YAML)
The `yaml` configuration files are located in `catkin_ws/elfin/elfin_gazebo/config`. These configuration files store the information about the plugin definition and the parameters that the code can read (it does not auto load every parameter written into the file, the source code needs to have commands to directly access the specific parameter). Go to the folder and find `gravity_comp_controller.yaml` and create a copy of it. Rename the copy as `gravity_comp_alternative_controller.yaml` and find the plugin definition lines 
```
    gravity_comp_controller:
      type: arm_controllers/GravityCompController

```
and modify as 
```
    gravity_comp_alternative_controller:
      type: arm_controllers/GravityCompAlternativeController
```


## Compile and run

    $ cd ~/catkin_ws/
    $ catkin_make
    $ source devel/setup.bash

After the compilation, you can run the simulation environment with the new controller as 

    $ roslaunch elfin_gazebo elfin3_empty_world.launch controller:=gravity_comp_alternative_controller
