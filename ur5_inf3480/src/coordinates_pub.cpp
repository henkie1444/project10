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
	while (ros::ok()) {					//this while loop creates two messages
		double x = 0.3;			//positive x is from us away
		double y = 0.3;				//positive y is to the left
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
			double x = 0.3;			//positive x is from us away
			double y = 0.3;				//positive y is to the left
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
