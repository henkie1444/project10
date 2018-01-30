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

class MoveRobot {
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;		
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
		void stop();									//this is the void that stops the UR5
		void PublishCoordinatesCallback(const ur5_inf3480::Coor msg);			//this is the callback that receives the Coor message
		void object();									//this is void that adds all the objects
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
	ROS_INFO("Hallo");
	moveit::planning_interface::MoveGroup group("manipulator");
	group.stop();										//stop the UR5
}

void MoveRobot::object() {
	moveit_msgs::CollisionObject collision_object1, collision_object2;			//create two objects that can collide
	collision_object1.header.frame_id = "base_link";					//objects are attached to these links
	collision_object2.header.frame_id = "wrist_3_link";
	collision_object1.id = "box1";								//the names
	collision_object2.id = "cube";
	shape_msgs::SolidPrimitive primitive, primitive2;

	primitive.type = primitive.BOX;								//define the size of object1
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 4;
	primitive.dimensions[1] = 4;
	primitive.dimensions[2] = 0.1;

	primitive2.type = primitive.BOX;							//define the size of object2
	primitive2.dimensions.resize(3);
	primitive2.dimensions[0] = 0.1;
	primitive2.dimensions[1] = 0.1;
	primitive2.dimensions[2] = 0.1;	

	geometry_msgs::Pose box_pose;								//the position of object1 (the table)
	box_pose.orientation.w = 1.0;
	box_pose.position.x = 0.0;
	box_pose.position.y = 0.0;
	box_pose.position.z =  -0.15;

	geometry_msgs::Pose cube_pose;								//add the position of object1
	collision_object1.primitives.push_back(primitive);
	collision_object1.primitive_poses.push_back(box_pose);
	collision_object1.operation = collision_object1.ADD;

	collision_object2.primitives.push_back(primitive2);					//add the position of object2
	collision_object2.primitive_poses.push_back(cube_pose);
	collision_object2.operation = collision_object2.ADD;

	std::vector<moveit_msgs::CollisionObject> collision_objects;
	collision_objects.push_back(collision_object1);
	collision_objects.push_back(collision_object2);
	planning_scene_interface.addCollisionObjects(collision_objects);

	//MOVE OBJECT//
	//moveit_msgs::CollisionObject move_object;
  	//move_object.id = "cube";
  	//move_object.header.frame_id = "wrist_3_link";	//TF-frame????
					//"/visp_auto_tracker1/object_position"
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
		x = mr.x;									//keep on checking the variables
		y = mr.x;									//at first the publisher doesn't publish yet so wait for this
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
	ros::Duration(5.0).sleep();
	//in this while loop the UR5 will move and stop. It will do this according to the variables it receives from the publisher
	//the publisher is in the coordinates_pub.cpp code in this src folder
	while (1) {										//every time a different variable is being published
		mr.object();
		if (a == 0 && b == 0) {
			mr.move(x, y, z, roll, pitch, yaw);					//call the move void with the variables from the message
			b = 1;
		}
		else if (a == 1 && c == 0) {
			mr.stop();								//after a while stop the UR5
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
			mr.move(x, y, z, roll, pitch, yaw);					//move the UR5 again with new coordinates
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

int teller1 = 550;
int teller2 = 700;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "coordinates");		//setup ROS
	ros::NodeHandle nh;
	ros::Publisher PublishCoordinates = nh.advertise<ur5_inf3480::Coor>("/coordinates", 1000); //create a publisher
	ros::Rate loop_rate(30);

	int counter = 0;
	srand (time(NULL));
	while (ros::ok()) {
		double x = 0.3;				// define the variables where the UR5 needs to move to
		double y = 0.3;
		double z = 0.3;
		double roll = 0.5;
		double pitch = 0.2;			
		double yaw = 0.5;
		int a = 0;
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
			double x = 0.4;
			double y = 0.3;
			double z = 0.4;
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
code
```

Modify the CMakeLists.txt to the following CMakeLists.txt file. The CMakeLists.txt file is in the ur5_inf3480 package.
```
cmake_minimum_required(VERSION 2.8.3)
project(ur5_inf3480)

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

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    Coor.msg
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
#   moveit_msgs
 )

catkin_package(
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

add_executable(coordinates_pub src/coordinates_pub.cpp)
target_link_libraries(coordinates_pub ${catkin_LIBRARIES})
add_dependencies(coordinates_pub ur5_inf3480 coordinates_pub.cpp)

add_executable(inf3480_move_robot src/inf3480_move_robot.cpp)
target_link_libraries(inf3480_move_robot ${catkin_LIBRARIES})

add_executable(tfbinnen src/tfbinnen.cpp)
target_link_libraries(tfbinnen ${catkin_LIBRARIES})
```

Edit the launch file as follows. The launch file is in the launch folder in the package ur5_inf3480.
```
<node name="inf3480_move_robot" pkg="ur5_inf3480" type="inf3480_move_robot" respawn="false" output="screen"></node>
```
Everything is set up now. To run the code run the following line in the terminal.
```
roslaunch ur5_inf3480 ur5_launch_inf3480.launch
```
