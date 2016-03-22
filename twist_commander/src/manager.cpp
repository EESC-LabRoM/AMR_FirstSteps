#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>

#define OKAY 0
#define WARN 1
#define EMERGENCY 2

// Publisher object
ros::Publisher pub_twist;
// Twist messages
geometry_msgs::Twist obstacle_twist, carrot_twist;
// Obstacle warn gain factor. The greater this gain, the more the robot believes in the obstacle detector
double k_warn = 0.7;   

void emergencyCallback(const std_msgs::Int32::ConstPtr &msg)
{

    geometry_msgs::Twist twist;
    
    ROS_INFO("emergency status: [%d]", msg->data);

    switch (msg->data){
            // No obstacle detected in the sensor grid
            case(OKAY): twist = carrot_twist;
                        break;
            // Detected an obstacle far from robot
            case(WARN): twist.linear.x = ((1-k_warn) * carrot_twist.linear.x + k_warn * obstacle_twist.linear.x);
                        twist.angular.z =((1-k_warn) * carrot_twist.angular.z + k_warn * obstacle_twist.angular.z);
                        break;
            // Detected obstacle very close
            case(EMERGENCY): twist = obstacle_twist;
                            break;
    } 
    
    pub_twist.publish(twist);
}

void carrotCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    carrot_twist = *msg;
}
void obstacleCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    obstacle_twist = *msg;
}

int main(int argc, char **argv)
{
    // Start ROS within this node
    ros::init(argc,argv,"manager");
    // Start ROS Node Handle
    ros::NodeHandle node;
    // Start subscribers
    ros::Subscriber sub_obstacle = node.subscribe("/amr/obstacle/twist", 1, obstacleCallback);
    ros::Subscriber sub_carrot   = node.subscribe("/amr/carrot/twist"  , 1,   carrotCallback);
    ros::Subscriber sub_emergency   = node.subscribe("/amr/obstacle/alert"  , 1,   emergencyCallback);
    // Start publishers
    pub_twist     = node.advertise<geometry_msgs::Twist>("/amr/cmd_vel",1);
    
    ros::spin();

}