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
double glPercentage = 0;

geometry_msgs::Twist think_math() {
  // local variables
  double v = -0.1;
  double w = 0;
  geometry_msgs::Twist returnTwist;
  double kw = 2, kv = 1;
 glAlert.data = 0;
    // ===== logic =====
  if (glRSonar > 0 && glLSonar > 0) // detected something on my right
  {
        double d_side = glRSonar - glLSonar;
        w = kw * (d_side);
        if (std::abs(d_side) > 0.3){               
            glAlert.data = 2;
        } else {
            glAlert.data = 1;
        }
  } else if (glRSonar > 0 ){
        w =  kw * (SONAR_MAX - glRSonar);
        if (std::abs(glRSonar) < 0.3){
            glAlert.data = 2;
        } else {
             w *= 2;
            glAlert.data = 1;
        }
        
 } else if (glLSonar > 0 ){
        w = - 1.5 * kw * (SONAR_MAX - glLSonar);
        if (std::abs(glLSonar) < 0.3){
            glAlert.data = 2;
        } else {
            w *= 2;
            glAlert.data = 1;
        }
  }
    

  
  if (glFSonar > 0 && glFSonar < 0.3){
    v = v - 0.5;
    glAlert.data = 2;
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

void cbkPercentage(const std_msgs::Float32ConstPtr& msg) {
  glPercentage = msg->data;
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
  ros::Subscriber subPercentage = node.subscribe("/amr/percentageDone", 1, cbkPercentage);
  
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

