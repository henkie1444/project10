#include "ros/ros.h"
#include <sstream>
#include "geometry_msgs/PoseStamped.h"

#include <tf/transform_broadcaster.h>

#include <vector>

std::vector<geometry_msgs::PoseStamped::ConstPtr> pose;

double x_current = 0;
double y_current = 0;
double z_current = 0;

//header;
std::string cube;



void tf_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	//ROS_INFO("CALLBACK");
	
	//ROS_INFO_STREAM(x);

	//ROS_INFO_STREAM("Received pose: " << msg);
 	 // Use the msg object here to access the pose elements,
	
	//header = msg->header.frame_id;

 	x_current = msg->pose.position.x;
	y_current = msg->pose.position.y;
	z_current = msg->pose.position.z;
	
	//ROS_INFO_STREAM(header);

	//ROS_INFO_STREAM(x_current);
	//ROS_INFO_STREAM(y_current);
	//ROS_INFO_STREAM(z_current);
	pose.push_back(msg);



	 static tf::TransformBroadcaster br;
 	 tf::Transform transform;
 	 transform.setOrigin( tf::Vector3(x_current, y_current, z_current) );
	 transform.setRotation( tf::Quaternion(0, 0, 0, 1) );

	 br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/front_camera", "cube"));
}
	

int main(int argc, char **argv) {
	
	sleep(10.0);

	ROS_INFO("HAAAAAAAAAAAAAAAAAAAAAAALLO");
	ros::init(argc, argv, "subscriberTF");

	ros::NodeHandle nh;
	
	//tf::TransformBroadcaster br;
 	//tf::Transform transform;
	//transform.setOrigin( tf::Vector3(1.0, 0.0, 0.0) );
   	//transform.setRotation( tf::Quaternion(0, 0, 0, 0) );
    	//br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/front_camera", "carrot1"));



	ros::Subscriber subscribetf = nh.subscribe("/visp_auto_tracker1/object_position", 1000, tf_callback);
	ros::spin();

	//ROS_INFO("DOOOOOOOOOOOOOOEEEEEEEEEEEII");
	return(0);
}
