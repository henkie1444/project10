# project10


This document explains first the standard steps that need to be taken.

The rest of the document shows the robot control part of our project.


The vision part is located on the follwing GitHub page: https://github.com/project10ros/Vision-indigo





ROS Indigo Ubuntu 14.04. 

Install ROS: http://wiki.ros.org/indigo/Installation/Ubuntu

ROS tutorial: http://wiki.ros.org/ROS/Tutorials


Install MoveIt!:
```
sudo apt-get install ros-indigo-moveit-full
```
MoveIt! tutorial: http://docs.ros.org/indigo/api/moveit_tutorials/html/


Install ROS Industrial:
```
sudo apt-get install ros-indigo-industrial-core
```

Install Universal robot:
```
sudo apt-get install ros-indigo-universal-robot
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
<launch>
    <!-- If sim=false, then robot_ip is required -->
    <arg name="sim" default="true" />
    <arg name="robot_ip" unless="$(arg sim)" />
    <!-- By default, we are not in debug mode -->
    <arg name="debug" default="false" />

    <!-- Limited joint angles are used. Prevents complex robot configurations and makes the planner more efficient -->
    <arg name="limited" default="true" />

    <!-- Load UR5 URDF file - robot description file -->
    <include file="$(find ur5_moveit_config)/launch/planning_context.launch">
      <arg name="load_robot_description" value="true" />
      <arg name="limited" value="$(arg limited)" />
    </include>

    <!-- If sim mode, run the simulator -->
    <group if="$(arg sim)">
      <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
        <param name="/use_gui" value="true"/>
        <rosparam param="/source_list">[/move_group/fake_controller_joint_states]</rosparam>
      </node>
    </group>

    <!-- If using real robot, initialise connection to the real robot -->
    <group unless="$(arg sim)">
<!-- change ur5_moveit_config to ur_modern_driver -->
      <include file="$(find ur_modern_driver)/launch/ur5_bringup.launch">
        <arg name="robot_ip" value="$(arg robot_ip)" />
      </include>
    </group>

    <!-- Given the published joint states, publish tf for the robot links -->
    <!-- <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" respawn="true" output="screen" /> -->

    <!-- Launch the move group for motion planning -->
    <group if="$(arg sim)">
        <include file="$(find ur5_moveit_config)/launch/move_group.launch">
            <arg name="limited" value="$(arg limited)" />
            <arg name="allow_trajectory_execution" value="true"/>  
            <arg name="fake_execution" value="true"/>
            <arg name="info" value="true"/>
            <arg name="debug" value="$(arg debug)"/>
        </include>
    </group>

    <group unless="$(arg sim)">
        <include file="$(find ur5_moveit_config)/launch/move_group.launch">
            <arg name="limited" default="true" />
            <arg name="publish_monitored_planning_scene" value="true" />
        </include>
    </group>

    <!-- Launch the RViz visualizer -->
    <include file="$(find ur5_moveit_config)/launch/moveit_rviz.launch">
        <arg name="config" value="true" />
    </include>

    <!-- Optionally, you can launch a database to record all the activities -->
    <!-- <include file="$(find ur5_moveit_config)/launch/default_warehouse_db.launch" /> -->

    <!-- Launch our own script -->
    <node name="coordinates_pub" pkg="ur5_inf3480" type="coordinates_pub" respawn="false" output="screen"></node>
    <node name="inf3480_move_robot" pkg="ur5_inf3480" type="inf3480_move_robot" respawn="false" output="screen"></node>
    <node name="tfbinnen" pkg="ur5_inf3480" type="tfbinnen" respawn="false" output="screen"></node> -->
</launch>
```

Build the workspace.
```
cd ~/catkin_ws
catkin_make
```

Add our own nodes. Create the src directory
```
cd ~/catkin_ws/src/ur5_inf3480
mkdir src
cd src
```

Create a new file called move_robot.cpp and coordinates_pub.cpp. (Don't forget to also type .cpp.)
Create these files in the src folder of the ur5_inf3480 package.

Paste the following code in the inf3480_move_robot.cpp file.
```
//this script receives the message and moves the UR5 instantly
#include <iostream>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <sstream>
#include <ur5_inf3480/Coor.h>
#include <nodelet/nodelet.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include "std_msgs/String.h"

//a class is made so that we can assign different variables in the main loop for the void that moves the UR5
class MoveRobot {
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;		
	moveit_msgs::CollisionObject collision_object1;
	std::vector<moveit_msgs::CollisionObject> collision_objects;
	public:
		double x;
		double y;
		double z;
		double roll;
		double pitch;
		double yaw;
		int a;
		int counter;
		void move(double x, double y, double z, double roll, double pitch, double yaw);	//this is the void that moves the UR5
		void stop();
		void object();
		void moveobject();
		void PublishCoordinatesCallback(const ur5_inf3480::Coor msg);			//this void is a callback that receives the Coor message
};

void MoveRobot::PublishCoordinatesCallback(const ur5_inf3480::Coor msg) {
	x = msg.x;										//assign the Coor.msg variables to public variables in the script
	y = msg.y;
	z = msg.z;
	roll = msg.roll;
	pitch = msg.pitch;
	yaw = msg.yaw;
	a = msg.a;
	counter = msg.counter;
}

void MoveRobot::stop() {
	moveit::planning_interface::MoveGroup group("manipulator");
	group.stop();										//stop the UR5
}

void MoveRobot::object() {									//this void adds a table to RViz
	ROS_INFO("OBJECT ADDING");
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;		//create a planningsceneinterface
	moveit_msgs::CollisionObject collision_object1;						//create a message for collision with came collision_object1
	collision_object1.header.frame_id = "/world";						//the object is related to the world
	collision_object1.id = "table";								//the object is called table

	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 4;
	primitive.dimensions[1] = 4;
	primitive.dimensions[2] = 0.1;

	geometry_msgs::Pose box_pose;
	box_pose.orientation.w = 1.0;
	box_pose.position.x = 0.0;
	box_pose.position.y = 0.0;
	box_pose.position.z =  -0.15;

	geometry_msgs::Pose cube_pose;

	collision_object1.primitives.push_back(primitive);
	collision_object1.primitive_poses.push_back(box_pose);
	collision_object1.operation = collision_object1.ADD;

	std::vector<moveit_msgs::CollisionObject> collision_objects;
	collision_objects.push_back(collision_object1);
	ROS_INFO("Add an object into the world");
	planning_scene_interface.addCollisionObjects(collision_objects);
}

void MoveRobot::moveobject() {
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;		//create a planningsceneinterface
	moveit_msgs::CollisionObject collision_object1;						//create a message for collision with came collision_object1
	collision_object1.header.frame_id = "/cube";						//the object is related to the cube
	collision_object1.id = "cube";								//it is called cube
	//the cube is the moving object that we import from vision. Here we want to attach an object.

	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 0.1;
	primitive.dimensions[1] = 0.1;
	primitive.dimensions[2] = 0.1;

	geometry_msgs::Pose cube_pose;

	collision_object1.primitives.push_back(primitive);
	collision_object1.primitive_poses.push_back(cube_pose);
	collision_object1.operation = collision_object1.ADD;

	std::vector<moveit_msgs::CollisionObject> collision_objects;
	collision_objects.push_back(collision_object1);
	planning_scene_interface.addCollisionObjects(collision_objects);

	//This is the code which should add an object to the moving frame that we import from vision
	//This is still a work in progress
	//moveit_msgs::CollisionObject move_object;
  	//move_object.id = "cube";
  	//move_object.header.frame_id = "/cube";						//"/visp_auto_tracker1/object_position"
  	//move_object.primitive_poses.push_back(cube_pose);
	//move_object.operation = move_object.MOVE;
	//collision_object_publisher.publish(move_object);
	//MOVE OBJECT//
}

void MoveRobot::move(double x, double y, double z, double roll, double pitch, double yaw) {	//move the UR5
	geometry_msgs::Pose pose;								//create a position message for MoveIt
	moveit::planning_interface::MoveGroup group("manipulator");				//create a MoveGroup for MoveIt
	moveit::planning_interface::MoveGroup::Plan my_plan;					//create a plan for MoveIt
	pose.position.x = x;									//assign the variables from the message
	pose.position.y = y;
	pose.position.z = z;
	pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
	group.setPoseTarget(pose, "");								//create the combination of coordinates
	bool success = group.plan(my_plan);
	while(!success) {									//if planning is not succes yet. Keep on looping until it is
		success = group.plan(my_plan);
	}
	if(success) {										//when the planning succeeded. Execute the movement
		group.asyncMove();
	}	
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "coordinates");							//setup ROS and create a nodehandle called nh
	ros::NodeHandle nh;
	ros::Rate loop_rate(30);
	MoveRobot mr;										//call the class and then subscribe to the topic
	ros::Subscriber sub = nh.subscribe<ur5_inf3480::Coor>("coordinates", 1000, &MoveRobot::PublishCoordinatesCallback, &mr);
	double v = 0.2;
	double w = 10;									
	double x = mr.x;									//assign the messagevariables to the public variables
	double y = mr.y;
	double z = mr.z;
	double roll = mr.roll;
	double pitch = mr.pitch;
	double yaw = mr.yaw;
	int a = mr.a;
	int b;
	int c;
	int d;
	int counter = mr.counter;
	while (x<v||x>w && y<v||y>w && z<v||z>w && roll<v||roll>w && pitch<v||pitch>w && yaw<v||yaw>w && a<v||a>w) { 
		x = mr.x;									//then you wait until they aren't (work in progress)
		y = mr.x;
		z = mr.x;
		roll = mr.roll;
		pitch = mr.pitch;
		yaw = mr.yaw;
		a = mr.a;
		loop_rate.sleep();
		ros::spinOnce();
	}
	loop_rate.sleep();									//the coordinates are higher than 0.2
	ros::spinOnce();
	ros::AsyncSpinner spinner(1);
	spinner.start();
	moveit::planning_interface::MoveGroup group("manipulator");				//create the MoveGroup for MoveIt
	group.allowLooking(true);
	group.allowReplanning(true);
	b = 0;
	c = 0;
	d = 0;
	mr.object();
	ros::Duration(7.0).sleep();
	while (1) {
		//The followinging two lines execute the moveobject void which should be the void that attaches an object to the moving frame from vision
		//This piece of code is a work in progress
//		mr.moveobject();
//		collision_object_publisher.publish(move_object);
		if (a == 0 && b == 0) {
			mr.move(x, y, z, roll, pitch, yaw);					//call the move void with the variables from the message
			b = 1;
		}
		else if (a == 1 && c == 0) {
			mr.stop();
			c = 1;
		}
		else if (a == 2 && d == 0) {
			x = mr.x;				
			y = mr.x;
			z = mr.x;
			roll = mr.roll;
			pitch = mr.pitch;
			yaw = mr.yaw;
			loop_rate.sleep();
			ros::spinOnce();
			mr.move(x, y, z, roll, pitch, yaw);
			d = 1;
		}
		else {
		}
		a = mr.a;
		loop_rate.sleep();
		ros::spinOnce();
	}
	return 0;
}
```

Paste the following code in the coordinates_pub.cpp file.
```
#include "ros/ros.h"
#include <sstream>
#include <ur5_inf3480/Coor.h>

int teller1 = 550; //620
int teller2 = 700; //800

int main(int argc, char **argv)
{
	ros::init(argc, argv, "coordinates");
	ros::NodeHandle nh;
	ros::Publisher PublishCoordinates = nh.advertise<ur5_inf3480::Coor>("/coordinates", 1000);
	ros::Rate loop_rate(30);

	int counter = 0;
	srand (time(NULL));
	while (ros::ok()) {					//this while loop creates three messages according to a counter
		double x = 0.3;	
		double y = 0.3;		
		double z = 0.3;
		double roll = 0.5;
		double pitch = 0.2;			
		double yaw = 0.5;
		int a = 0;
		ur5_inf3480::Coor msg;				//create the message
		msg.counter=counter;				//assign all coordinates to the message
		msg.x=x;
		msg.y=y;
		msg.z=z;
		msg.roll=roll;
		msg.pitch=pitch;
		msg.yaw=yaw;
		msg.a=a;
		PublishCoordinates.publish(msg);		//publish the message
		if (counter >= teller1 && counter < teller2) {
			if (counter == teller1) {
				ROS_INFO("counter is teller1");
			}
			double x = 1;
			double y = 0.25;
			double z = 0.3;
			double roll = 0.5;
			double pitch = 0.5;
			double yaw = 0.5;
			int a = 1;
			ur5_inf3480::Coor msg;			//create the message
			msg.counter=counter;			//assign all coordinates to the message
			msg.x=x;
			msg.y=y;
			msg.z=z;
			msg.roll=roll;
			msg.pitch=pitch;
			msg.yaw=yaw;
			msg.a=a;
			PublishCoordinates.publish(msg);	//publish the message
		}
		else if (counter >= teller2) {
			if (counter == teller2) {
				ROS_INFO("counter is teller2");
			}
			double x = 0.3;			
			double y = 0.3;			
			double z = 0.3;
			double roll = 0.5;
			double pitch = 0.2;			
			double yaw = 0.5;
			int a = 2;
			ur5_inf3480::Coor msg;			//create the message
			msg.counter=counter;			//assign all coordinates to the message
			msg.x=x;
			msg.y=y;
			msg.z=z;
			msg.roll=roll;
			msg.pitch=pitch;
			msg.yaw=yaw;
			msg.a=a;
			PublishCoordinates.publish(msg);	//publish the message
		}
		ros::spinOnce();
		loop_rate.sleep();
		counter++;
	}
	return 0;
}
```

Paste the following code in the tfbinnen.cpp file.
```
#include "ros/ros.h"
#include <sstream>
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>
#include <vector>

std::vector<geometry_msgs::PoseStamped::ConstPtr> pose;				//Define the message we subscibe to

double x_current = 0;								//Define variables
double y_current = 0;
double z_current = 0;

std::string cube;

void tf_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {		//This void callback gets the information from the message from vision
 	x_current = msg->pose.position.x;
	y_current = msg->pose.position.y;
	z_current = msg->pose.position.z;

	pose.push_back(msg);							//push back the message

	static tf::TransformBroadcaster br;					//create a broadcaster that broadcasts the received coordinates of the cube
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(x_current, y_current, z_current) );	//here the coordinates are assigned to the tf.
	transform.setRotation( tf::Quaternion(0, 0, 0, 1) );

	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/front_camera", "cube"));	//send the coordinates to the tf
}										//RViz is now aware of the coordinates of the qr code and places it next to the robot

int main(int argc, char **argv) {
	sleep(10.0);
	ros::init(argc, argv, "subscriberTF");					//initialize ros
	ros::NodeHandle nh;							//create a nodehandle
	ros::Subscriber subscribetf = nh.subscribe("/visp_auto_tracker1/object_position", 1000, tf_callback);	//subscribe to the message to call tf_callback
	ros::spin();
	return(0);
}
```

Modify the CMakeLists.txt so that it looks like this one. Do not copy paste this code. MAKE SURE THAT YOU KEEP YOUR ORIGINAL CMakeLists.txt The CMakeLists.txt file is in the ur5_inf3480 package.
```
cmake_minimum_required(VERSION 2.8.3)
project(ur5_inf3480)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  joint_state_publisher
  moveit_core
  moveit_msgs
  moveit_planners_ompl
  moveit_ros_move_group
  moveit_ros_planning_interface
  moveit_ros_visualization
  robot_state_publisher
  ur_description
  xacro
  roscpp
  rospy
  std_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    Coor.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
#   moveit_msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ur5_inf3480
#  CATKIN_DEPENDS joint_state_publisher moveit_core moveit_msgs moveit_planners_ompl moveit_ros_move_group moveit_ros_planning_interface moveit_ros_visualization robot_state_publisher ur_description xacro
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/ur5_inf3480.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

add_executable(coordinates_pub src/coordinates_pub.cpp)
target_link_libraries(coordinates_pub ${catkin_LIBRARIES})
add_dependencies(coordinates_pub ur5_inf3480 coordinates_pub.cpp)

add_executable(inf3480_move_robot src/inf3480_move_robot.cpp)
target_link_libraries(inf3480_move_robot ${catkin_LIBRARIES})

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ur5_inf3480.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
```

Edit the launch file as follows. The launch file is in the launch folder in the package ur5_inf3480.
```
<launch>
    <!-- If sim=false, then robot_ip is required -->
    <arg name="sim" default="true" />
    <arg name="robot_ip" unless="$(arg sim)" />
    <!-- By default, we are not in debug mode -->
    <arg name="debug" default="false" />

    <!-- Limited joint angles are used. Prevents complex robot configurations and makes the planner more efficient -->
    <arg name="limited" default="true" />

    <!-- Load UR5 URDF file - robot description file -->
    <include file="$(find ur5_moveit_config)/launch/planning_context.launch">
      <arg name="load_robot_description" value="true" />
      <arg name="limited" value="$(arg limited)" />
    </include>

    <!-- If sim mode, run the simulator -->
    <group if="$(arg sim)">
      <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
        <param name="/use_gui" value="true"/>
        <rosparam param="/source_list">[/move_group/fake_controller_joint_states]</rosparam>
      </node>
    </group>

    <!-- If using real robot, initialise connection to the real robot -->
    <group unless="$(arg sim)">
<!-- change ur5_moveit_config to ur_modern_driver -->
      <include file="$(find ur_modern_driver)/launch/ur5_bringup.launch">
        <arg name="robot_ip" value="$(arg robot_ip)" />
      </include>
    </group>

    <!-- Given the published joint states, publish tf for the robot links -->
    <!-- <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" respawn="true" output="screen" /> -->

    <!-- Launch the move group for motion planning -->
    <group if="$(arg sim)">
        <include file="$(find ur5_moveit_config)/launch/move_group.launch">
            <arg name="limited" value="$(arg limited)" />
            <arg name="allow_trajectory_execution" value="true"/>  
            <arg name="fake_execution" value="true"/>
            <arg name="info" value="true"/>
            <arg name="debug" value="$(arg debug)"/>
        </include>
    </group>

    <group unless="$(arg sim)">
        <include file="$(find ur5_moveit_config)/launch/move_group.launch">
            <arg name="limited" default="true" />
            <arg name="publish_monitored_planning_scene" value="true" />
        </include>
    </group>

    <!-- Launch the RViz visualizer -->
    <include file="$(find ur5_moveit_config)/launch/moveit_rviz.launch">
        <arg name="config" value="true" />
    </include>

    <!-- Optionally, you can launch a database to record all the activities -->
    <!-- <include file="$(find ur5_moveit_config)/launch/default_warehouse_db.launch" /> -->

    <!-- Launch our own script -->
    <node name="coordinates_pub" pkg="ur5_inf3480" type="coordinates_pub" respawn="false" output="screen"></node>
    <node name="inf3480_move_robot" pkg="ur5_inf3480" type="inf3480_move_robot" respawn="false" output="screen"></node>
    <node name="tfbinnen" pkg="ur5_inf3480" type="tfbinnen" respawn="false" output="screen"></node> -->
</launch>
```
Everything is set up now. To run the code run the following line.
Make sure that you are in your workspace.
So
```
cd
cd ~/catkin_ws
roslaunch ur5_inf3480 ur5_launch_inf3480.launch
```
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////










////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Master - Slave connection
Set-up master slave connection between two laptops:

Master:
1. Get IP address:
```
hostname -I
```

```
Result will be like: 145.93.128.85
```
	
2. Export on IP:
```
ROS_IP=145.93.128.85	#Given IP is example.
```

Slave:
1. Connect to master:
```
export ROS_MASTER_URI=”Master_IP” :11311
```
2. Get IP address:
```
hostname -I
```

3. Export on IP:
```
ROS_IP=145.93.128.85	#Given IP is example.
```

Now the master can check the connection by asking the information from the node that is being send:
```
rostopic echo /node
```

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
When adding poses or frames send to the master from the slave, The configuration of Rviz must be changed in order to keep the correct settings every time when starting up Rviz.
1.The poses and frames are added in Rviz by using the add button and selecting the parts.
2.When the necessary poses or frames are added and visible the configuration of Rviz can be saved under he current or a new name.
3.When a new name is chosen the file that launches Rviz must be changed.
4.For our project the launch file in “universal robot” → “ur5_moveit_config” → “launch” → “moveit_rviz.launch”.
5. Code is shown below, where find ur5_moveit_config/launch is edited to include our new setting file named test.rviz.
6. These setting files are in the same location as this launch file.

```
<launch> 

  <arg name="debug" default="false" /> 
  <arg unless="$(arg debug)" name="launch_prefix" value="" /> 
  <arg     if="$(arg debug)" name="launch_prefix" value="gdb --ex run --args" /> 

  <arg name="config" default="false" /> 
  <arg unless="$(arg config)" name="command_args" value="" /> 
  <arg     if="$(arg config)" name="command_args" value="-d $(find ur5_moveit_config)/launch/test.rviz" />	 
  
  <node name="$(anon rviz)" launch-prefix="$(arg launch_prefix)" pkg="rviz" type="rviz" respawn="false" 
	args="$(arg command_args)" output="screen"> 
    <rosparam command="load" file="$(find ur5_moveit_config)/config/kinematics.yaml"/> 
  </node> 

</launch>
```
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
