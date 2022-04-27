# A* Algorithm on Turtlebot 

1. Download the `my_simulations` ROS package into your `catkin_ws/src` folder.
2. Run the following commands

	```sh
	cd ~/<your catkin workspace>/
	cd src/my_simulations/scripts
	chmod +x ros.py
	cd ~/<your catkin workspace>/
	catkin_make
	source ~/<your catkin workspace>/devel/setup.bash
	roslaunch my_simulations astar.launch
	```

## Launch File 

Default arguments in the launch file are as follows:

```xml
  <arg name="model" default="burger" doc="model type [burger, waffle, waffle_pi]"/>
  <arg name="x_pos" default="-4"/>
  <arg name="y_pos" default="-4"/>
  <arg name="z_pos" default="0"/>
  <arg name="theta" default="0"/>
  <arg name="x_pos_f" default="4"/>
  <arg name="y_pos_f" default="4"/>
  <arg name="clearance" default="5"/>
  <arg name="rpm1" default="60"/>
  <arg name="rpm2" default="30"/>
```

To input arguments, use the following structure:

```sh
	roslaunch my_simulations astar.launch x_pos:=-4 y_pos:=-4 theta:=0 x_pos_f:=4 y_pos_f:=4 clearance:=5 rpm1:=60 rpm2:=30
```


