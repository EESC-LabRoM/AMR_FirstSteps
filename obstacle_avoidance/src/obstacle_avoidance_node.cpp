#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

#define SONAR_MAX 0.65

geometry_msgs::Twist glTwist;
/* 0 - Nada * 1 - +- * 2 - PQP */
std_msgs::Int32 glAlert;
double glLSonar = 0, glRSonar = 0, glFSonar = 0;
double glDirection = 1;

geometry_msgs::Twist think_if() {
  // local variables
  double v = 1.5;
  double w = 0;
  geometry_msgs::Twist returnTwist;
  
  // logic
  if(glFSonar == 0) {
    v = 1.5;
    w = 0;
  }
  if(glFSonar > 0 && glFSonar < 0.3) {
    v = 0.3;
  }
  if(glFSonar >= 0.3 && glFSonar < 0.3) {
    v = 1;
  }
  if(glRSonar == 0 && glLSonar == 0) {
    w = 0.25 * glDirection;
  }
  
  if(glLSonar >= 0.3 && glLSonar < 0.6) {
    v = 1.2;
    w += -0.5;
    glDirection = -1;
  }
  if(glLSonar > 0 && glLSonar < 0.3) {
    v = 1;
    w += -1.5;
    glDirection = -1;
  }
  
  if(glRSonar >= 0.3 && glRSonar < 0.6) {
    w += 0.5;
    glDirection = 1;
  }
  if(glRSonar > 0 && glRSonar < 0.3) {
    w += 1.5;
    glDirection = 1;
  }
  
  // return
  glAlert.data = 0;
  returnTwist.linear.x = v;
  returnTwist.angular.z = w;
  return returnTwist;
}

geometry_msgs::Twist think_math() {
  // local variables
  double v = 1.5;
  double w = 0;
  geometry_msgs::Twist returnTwist;
  double kw = 1.5, kv = 1.2;
  
  // ===== logic =====
  // ----- local variables -----
  double d_side = glRSonar - glLSonar;
  // ----- alert -----
  if(std::abs(d_side) < 0.3 && glFSonar < 0.3) {
    glAlert.data = 2;
  }
  else if(std::abs(d_side) < 0.5 && glFSonar < 0.5) {
    glAlert.data = 1;
  } else {
    glAlert.data = 0;
  }
  // ----- v -----
  v = kv * (1 - glFSonar/SONAR_MAX);
  if(glFSonar != 0 && glFSonar < 0.22) {
    v *= -1;
  }
  // ----- w -----
  if(d_side != 0 && std::abs(d_side) < 0.3) {
    w = kw * ((d_side) / SONAR_MAX);
  }
  
  // return
  returnTwist.linear.x = v;
  returnTwist.angular.z = w;
  return returnTwist;
}

void think() {
  glTwist = think_math();
}

void cbkFrontSonar(const std_msgs::Float32ConstPtr& sonar) { 
  ROS_INFO("front: [%.2f]", sonar->data);
  glFSonar = sonar->data;
}

void cbkLeftSonar(const std_msgs::Float32ConstPtr& sonar) {
  ROS_INFO("left: [%.2f]", sonar->data);
  glLSonar = sonar->data;
}

void cbkRightSonar(const std_msgs::Float32ConstPtr& sonar) {
  ROS_INFO("right: [%.2f]", sonar->data);
  glRSonar = sonar->data;
}

int main( int argc, char **argv) {
  // ROS init
  ros::init(argc,argv, "obstacle_avoidance");
  
  // Node
  ros::NodeHandle node;
  
  // Publishers
  ros::Publisher pubTwist = node.advertise<geometry_msgs::Twist>("/amr/obstacle/twist", 1);
  // ros::Publisher pubTwist = node.advertise<geometry_msgs::Twist>("/amr/twist", 1);
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

