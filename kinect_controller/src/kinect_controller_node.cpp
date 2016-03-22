#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"

double v=0,w=0;

void twistCallback(const geometry_msgs::TwistConstPtr &msg ){
	// Unpacking the message (Fixed body frame)
	v = msg->linear.x;
	w = msg->angular.z;
}


int main(int argc,char **argv ){
	// Initialize ROS machine
	ros::init(argc,argv,"kine_controller");
	// Node
	ros::NodeHandle node;
	// Publisher
	ros::Publisher pub_motorLeft  = node.advertise<std_msgs::Float32>("/vrep/vehicle/motorLeftSpeed",1);
        ros::Publisher pub_motorRight = node.advertise<std_msgs::Float32>("/vrep/vehicle/motorRightSpeed",1);
	// Subscriber
	ros::Subscriber sub_twist = node.subscribe("/amr/cmd_vel",1,twistCallback);

	// Loop rate
	ros::Rate loop_rate(60);

	// ROS Messages
	std_msgs::Float32 left_speed_msg, right_speed_msg;
	
	// Parameters
	double radius = 0.1;

	while(ros::ok()){	
		// Computing wheels speed
		double v_right = v + 0.175*w;  
		double v_left  = v - 0.175*w;
		
		// Assembling Message
		right_speed_msg.data = v_right/radius;  
		left_speed_msg.data  = v_left/radius;

		// Publish
		pub_motorLeft.publish(left_speed_msg);
		pub_motorRight.publish(right_speed_msg);

		// ROS routines
		ros::spinOnce();
		loop_rate.sleep();
	}

}