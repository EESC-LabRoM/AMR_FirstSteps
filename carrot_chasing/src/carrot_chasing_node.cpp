#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "std_msgs/Float32.h"
#include <cmath>

#define MAX_V 1.5
#define MAX_W 10
#define MAX_CROSS_ERROR_DISTANCE 1
#define TRAJECTORY_WIDTH 0.1

double x, y, yaw;
double WiX = 4,WiY = 1.5, WfX = -3, WfY = 1.5;
ros::Publisher pub_twist;
ros::Publisher pub_percentage;
    
double w = 0, v = 0.5, sigma = 1, K = 3; 

double normalizeAngle(double angle)
{
    if (std::abs(angle)>M_PI)
        angle -= 2*M_PI*angle / std::abs(angle);   
    return angle;
}
double calcDistance(double x1, double x2, double y1, double y2)
{
    return sqrt(pow(x1-x2,2) + pow(y1-y2,2));
}

void carrotChasing()
{
    ROS_INFO("WI: %f %f",WiX,WiY);
    ROS_INFO("WF: %f %f",WfX,WfY);
    double distance, Ru, theta, thetaU, beta, R, xc, yc, psiD, u, headingDiff, crossErrorDistance, xq, yq, fullDistance;    
    geometry_msgs::Twist Twist_msg;  
    std_msgs::Float32 percentage;
    
    //2
    Ru = sqrt(pow(WiX-x,2) + pow(WiY-y,2));
    theta = atan2(WfY-WiY,WfX-WiX);
    //3
    thetaU = atan2((y - WiY),x - WiX);
    beta  = theta - thetaU;
    //4
    crossErrorDistance = Ru*sin(beta);
    R = sqrt(pow(Ru,2) - pow(crossErrorDistance,2));
    //5
    xc = (R+sigma)*cos(theta) + WiX;
    yc = (R+sigma)*sin(theta) + WiY;
    //6
    psiD = atan2(yc - y, xc - x);
    headingDiff = normalizeAngle(psiD- yaw);    

    u = K*headingDiff;
    if (std::abs(u) > MAX_W)
    {
        u = MAX_W*std::abs(u)/u;
    }
    v = MAX_V;
    /*    
    if (crossErrorDistance > MAX_CROSS_ERROR_DISTANCE) 
    {
        v = MAX_V;        
    }else if (crossErrorDistance > TRAJECTORY_WIDTH) 
    {
        v = 0.5;
    }
    else{
        v = MAX_V;
    }*/
    
    xq = (R)*cos(theta) + WiX;
    yq = (R)*sin(theta) + WiY;
    fullDistance = calcDistance(WiX,WfX,WiY,WfY);
    //Publishing angular and linear velocities
    Twist_msg.linear.x = v;
    Twist_msg.angular.z =  u;     
    percentage.data = 1 - calcDistance(xq,WfX,yq,WfY)/fullDistance; 
    pub_twist.publish(Twist_msg);   
    pub_percentage.publish(percentage);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
    carrotChasing();
}

void wiCallback(const geometry_msgs::PointConstPtr &msg)
{
    WiX = msg->x;
    WiY = msg->y;
}

void wfCallback(const geometry_msgs::PointConstPtr &msg)
{
    WfX = msg->x;
    WfY = msg->y;
}

int main(int argc, char** argv)
{      
	ros::init(argc,argv,"carrot_chasing");
	ros::NodeHandle node;
    
    //Subscriber
    ros::Subscriber sub_odom = node.subscribe("/vrep/vehicle/odometry",1,odomCallback);
    ros::Subscriber sub_wi = node.subscribe("/amr/waypoints_i",1,wiCallback);
    ros::Subscriber sub_wf = node.subscribe("/amr/waypoints_f",1,wfCallback);
    
	//Publisher
	pub_twist = node.advertise<geometry_msgs::Twist>("/amr/carrot/twist",1); 
	pub_percentage = node.advertise<std_msgs::Float32>("/amr/percentageDone",1); 
    //pub_twist = node.advertise<geometry_msgs::Twist>("/amr/twist",1); 
            
    ros::spin();
	return 0;
}