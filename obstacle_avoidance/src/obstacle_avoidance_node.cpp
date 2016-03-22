#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

geometry_msgs::Twist glTwist;
/*
* 0 - Nada
* 1 - +-
* 2 - PQP
*/
std_msgs::Int32 glAlert;

void think() {
  glAlert.data = 0;
  glTwist.linear.x = 1.5;
  glTwist.angular.z = 0;
}

void cbkFrontSonar(const std_msgs::Float32ConstPtr& sonar)
{
  ROS_INFO("front: [%.2f]", sonar->data);
}

void cbkLeftSonar(const std_msgs::Float32ConstPtr& sonar)
{
  ROS_INFO("left: [%.2f]", sonar->data);
}

void cbkRightSonar(const std_msgs::Float32ConstPtr& sonar)
{
  ROS_INFO("right: [%.2f]", sonar->data);
}

int main( int argc, char **argv)
{
  // ROS init
  ros::init(argc,argv, "obstacle_avoidance");
  
  // Node
  ros::NodeHandle node;
  
  // Publishers
  ros::Publisher pubTwist = node.advertise<geometry_msgs::Twist>("/amr/twist", 1);
  ros::Publisher pubAlert = node.advertise<std_msgs::Int32>("/amr/obstacle/alert", 1);
  
  // Subscribers
  ros::Subscriber subSonarLeft = node.subscribe("/vrep/vehicle/leftSonar", 1, cbkLeftSonar);
  ros::Subscriber subSonarRight = node.subscribe("/vrep/vehicle/rightSonar", 1, cbkRightSonar);
  ros::Subscriber subSonarFront = node.subscribe("/vrep/vehicle/frontSonar", 1, cbkFrontSonar);
  
  ros::Rate loopRate(20);
  
  while(ros::ok()) {
    // Think
    think();
    
    // Publishing
    pubTwist.publish(glTwist);
    pubAlert.publish(glAlert);
    
    // Spin once then sleep
    ros::spinOnce();
    loopRate.sleep();
  }
   
} 