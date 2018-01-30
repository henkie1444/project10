# project10

ROS Indigo Ubuntu 14.04. 

Install ROS http://wiki.ros.org/indigo/Installation/Ubuntu

Install MoveIt!
```
sudo apt-get install ros-indigo-moveit-full
```

Install ROS Industrial
```
sudo apt-get install ros-indigo-industrial-core
```

Create catkin workspace. In the terminal.
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
```

Build the workspace.
```
cd ~/catkin_ws/
catkin_make
```

Source the workspace.
```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Check whether the workspace is in the right path.
```
echo $ROS_PACKAGE_PATH
```

Install UR5 Packages.
We want to install from source, so we can modify source files easily to our needs:
```
cd ~/catkin_ws/src
git clone https://github.com/ros-industrial/universal_robot.git
cd ..
catkin_make
```

Create a package for INF3480 with required dependencies
```
cd ~/catkin_ws/src
catkin_create_pkg ur5_inf3480 moveit_ros_move_group moveit_planners_ompl moveit_ros_visualization joint_state_publisher robot_state_publisher xacro ur_description moveit_msgs moveit_core moveit_ros_planning_interface
```

Create the launch directory
```
cd ~/catkin_ws/src/ur5_inf3480
mkdir launch
cd launch
```

Create the launch file for running the UR5 simulation named “ur5_launch_inf3480.launch”
```
<!-- ROS Demo INF3480 -->
<launch>
    <!-- Launch our own script -->
    <!-- <node name="inf3480_move_robot" pkg="ur5_inf3480" type="inf3480_move_robot" respawn="false" output="screen"></node> -->
</launch>
```

Build the workspace.
```
cd ~/catkin_ws
catkin_make
```

Run the launch file
```
roslaunch ur5_inf3480 ur5_launch_inf3480.launch
```

Add our own nodes. Create the src directory
```
cd ~/catkin_ws/src/ur5_inf3480
mkdir src
cd src
```

Create a new file called move_robot.cpp and coordinates_pub.cpp. (Don't forget to also type .cpp.)
Create these files in the src folder of the ur5_inf3480 package.

Paste the following code in the move_robot.cpp file.
```
code
```

Paste the following code in the coordinates_pub.cpp file.
```
code
```

Paste the following code in the tfbinnen.cpp file.
```
code
```

Modify the CMakeLists.txt to the following CMakeLists.txt file. The CMakeLists.txt file is in the ur5_inf3480 package.
```
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(inf3480_move_robot src/inf3480_move_robot.cpp)

target_link_libraries(inf3480_move_robot
  ${catkin_LIBRARIES}
)
```

Edit the launch file as follows. The launch file is in the launch folder in the package ur5_inf3480.
```
<node name="inf3480_move_robot" pkg="ur5_inf3480" type="inf3480_move_robot" respawn="false" output="screen"></node>
```
