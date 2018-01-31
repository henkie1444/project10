//this script receives the message and moves the UR5 instantly
//roslaunch ur5_inf3480 ur5_launch_inf3480.launch sim:=false robot_ip:=192.168.1.146
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
//#include <moveit/core/transforms/transforms.h>	// Eigen transforms

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
		// Collision object
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
	ROS_INFO("Hallo");
	moveit::planning_interface::MoveGroup group("manipulator");				//this void is still a work in progress
	group.stop();
}

void MoveRobot::object() {
	
	//OBJECTS//	
	ROS_INFO("OBJECT ADDING");
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	//moveit_msgs::CollisionObject collision_object1;
	//std::vector<moveit_msgs::CollisionObject> box1;
	//std::vector<moveit_msgs::CollisionObject> collision_objects;


	//MOVE OBJECT//
	//ros::NodeHandle node_handle;	//???????????
	//ros::Publisher collision_object_publisher = node_handle.advertise<moveit_msgs::CollisionObject>("collision_object", 1);
	//MOVE OBJECT//



	moveit_msgs::CollisionObject collision_object1;
	//moveit_msgs::CollisionObject collision_object1, collision_object2;

	collision_object1.header.frame_id = "/world";//group.getPlanningFrame();
	//collision_object2.header.frame_id = "/cube";//group.getPlanningFrame();
						//"/visp_auto_tracker1/object_position"
						//"wrist_3_link"
	
/////////////////////////////////////////TRY WITH TF-FRAME ID FROM VISION VISP TF????????????????????????
	
	
	
	collision_object1.id = "table";
	//collision_object2.id = "cube";
	
	shape_msgs::SolidPrimitive primitive;
	//shape_msgs::SolidPrimitive primitive, primitive2;

	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 4;
	primitive.dimensions[1] = 4;
	primitive.dimensions[2] = 0.1;

	//Object two	//
	//primitive2.type = primitive.BOX;
	//primitive2.dimensions.resize(3);
	//primitive2.dimensions[0] = 0.1;	//0.1
	//primitive2.dimensions[1] = 0.1;	//0.1
	//primitive2.dimensions[2] = 0.1;	//0.1	


	geometry_msgs::Pose box_pose;
	box_pose.orientation.w = 1.0;
	box_pose.position.x = 0.0;
	box_pose.position.y = 0.0;
	box_pose.position.z =  -0.15;

	//Position of object two
	geometry_msgs::Pose cube_pose;
	//NOT NEEDED????
	//cube_pose.orientation.w = 1.0;	//
	//cube_pose.position.x = 0.00;		//
	//cube_pose.position.y = 0.50;		//
	//cube_pose.position.z = -0.50;		//-0.05
	//NOT NEEDED????

	collision_object1.primitives.push_back(primitive);
	collision_object1.primitive_poses.push_back(box_pose);
	collision_object1.operation = collision_object1.ADD;

	//Object two
	//collision_object2.primitives.push_back(primitive2);
	//collision_object2.primitive_poses.push_back(cube_pose);
	//collision_object2.operation = collision_object2.ADD;
	//


	std::vector<moveit_msgs::CollisionObject> collision_objects;
  
	collision_objects.push_back(collision_object1);
	//Object two
	//collision_objects.push_back(collision_object2);
  
	ROS_INFO("Add an object into the world");
	planning_scene_interface.addCollisionObjects(collision_objects);

//OBJECTS//
}

void MoveRobot::moveobject() {
	
	ROS_INFO("MOVING OBJECT ADDING");
	
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	
	moveit_msgs::CollisionObject collision_object1;
	
	collision_object1.header.frame_id = "/cube";
	
	collision_object1.id = "cube";

	
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


	//MOVE OBJECT//
	
	//moveit_msgs::CollisionObject move_object;
  	//move_object.id = "cube";
  	//move_object.header.frame_id = "/cube";	//TF-frame????
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
//		group.asyncExecute(my_plan);
//		group.move();
		group.asyncMove();
		ROS_INFO("asyncExecute");
	}	
	ROS_INFO("READY");
	
}





int main(int argc, char **argv) {
	ROS_INFO("1");
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
	ROS_INFO("2");
	while (x<v||x>w && y<v||y>w && z<v||z>w && roll<v||roll>w && pitch<v||pitch>w && yaw<v||yaw>w && a<v||a>w) { 
		ROS_INFO("3");
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
	ROS_INFO("4");
	loop_rate.sleep();									//the coordinates are higher than 0.2
	ros::spinOnce();
	ros::AsyncSpinner spinner(1);
	spinner.start();
	moveit::planning_interface::MoveGroup group("manipulator");				//create the MoveGroup for MoveIt
	ros::Duration(2.0).sleep();
	group.allowLooking(true);
	group.allowReplanning(true);
	b = 0;
	c = 0;
	d = 0;
	ROS_INFO("5");
	mr.object();

	//mr.moveobject();	///////////////////////////////////////////////////////////////////



	ros::Duration(5.0).sleep();




/*	//MOVE OBJECT//
	ros::NodeHandle node_handle;	//???????????
	ros::Publisher collision_object_publisher = node_handle.advertise<moveit_msgs::CollisionObject>("collision_object", 1);
	//MOVE OBJECT//
  //while(collision_object_publisher.getNumSubscribers() < 1)
 // {
     // ros::WallDuration sleep_t(0.5);
     // sleep_t.sleep();
  //}
	moveit_msgs::CollisionObject collision_object;

	collision_object.header.frame_id = "/cube";

	collision_object.id = "cube";

//	shape_msgs::SolidPrimitive primitive, primitive2;
	shape_msgs::SolidPrimitive primitive;

	//Object two	//
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 0.1;	//0.1
	primitive.dimensions[1] = 0.1;	//0.1
	primitive.dimensions[2] = 0.1;	//0.1	

	//Position of object two
	geometry_msgs::Pose cube_pose;

	//Object two
	collision_object.primitives.push_back(primitive);
	collision_object.primitive_poses.push_back(cube_pose);
	collision_object.operation = collision_object.ADD;
	//

	//Object two
	//collision_objects.push_back(collision_object2);



	//MOVE OBJECT//
	
	moveit_msgs::CollisionObject move_object;
  	move_object.id = "cube";
  	move_object.header.frame_id = "/cube";	//TF-frame????
					//"/visp_auto_tracker1/object_position"
  	move_object.primitive_poses.push_back(cube_pose);
  	
	
	move_object.operation = move_object.MOVE;


	collision_object_publisher.publish(move_object);
	//MOVE OBJECT//
*/


	while (1) {
		if (a == 0 && b == 0) {
			ROS_INFO("6");
			mr.move(x, y, z, roll, pitch, yaw);					//call the move void with the variables from the message
			b = 1;
			ROS_INFO("7");
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
			ROS_INFO("8");
			mr.move(x, y, z, roll, pitch, yaw);
			ROS_INFO("9");
			d = 1;
		}
		else {
		}
		a = mr.a;
//		collision_object_publisher.publish(move_object);
		loop_rate.sleep();
		ros::spinOnce();
	}
	return 0;
}
