#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

std_msgs::Float32 wR;
std_msgs::Float32 wL;
double carWidth = 0.35;

void callback(const geometry_msgs::TwistConstPtr& vel)
{               
  wR.data = (vel->linear.x)/carWidth + (vel->angular.z)/2;
  wL.data = (vel->linear.x)/carWidth - (vel->angular.z)/2;
       
}

int main( int argc, char **argv)
{
  // ROS init
  ros::init(argc,argv, "Kinect_Controller");
  
  // Node
  ros::NodeHandle node;
  
  // Publishers
  ros::Publisher pubRMotor = node.advertise<std_msgs::Float32>("/vrep/vehicle/motorRightSpeed" , 1);   
  ros::Publisher pubLMotor = node.advertise<std_msgs::Float32>("/vrep/vehicle/motorLeftSpeed" , 1);
  
  // Subscribers
  ros::Subscriber s = node.subscribe("/sonarController",1, callback);
  
  ros::Rate loopRate(20);
  
  while(ros::ok()) {
    // Publishing
    pubRMotor.publish(wR);
    pubLMotor.publish(wL);
    
    // Spin once then sleep
    ros::spinOnce();
    loopRate.sleep();
  }
   
} 