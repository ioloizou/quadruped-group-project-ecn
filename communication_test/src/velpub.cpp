#include "ros/ros.h"
#include <geometry_msgs/Twist.h>


class VelPubNode : public ros::NodeHandle
{
public:
    VelPubNode() : ros::NodeHandle("~")
    {
        param<double>("lin_x", lin_x, 1.0); //parameter name, variable to store parameter, default value. 
        param<double>("lin_y", lin_y, 0.0);
        param<double>("lin_z", lin_z, 0.0);
        
        param<double>("ang_x", ang_x, 0.0);
        param<double>("ang_y", ang_y, 0.0);
        param<double>("ang_z", ang_z, 1.0);

        publisher_ = advertise<geometry_msgs::Twist>("/robot_1/cmd_vel", 10);
        timer_ = createTimer(ros::Duration(0.05), &VelPubNode::publisher_callback, this);    
    }
private:
    
    ros::Publisher publisher_;
    ros::Timer timer_;

    double lin_x;
    double lin_y;
    double lin_z;

    double ang_x;
    double ang_y;
    double ang_z;

    void publisher_callback(const ros::TimerEvent& event){
        
        geometry_msgs::Twist cmd;
        cmd.linear.x = lin_x;
        cmd.linear.y = lin_y;
        cmd.linear.z = lin_z;

        cmd.angular.x = ang_x;
        cmd.angular.y = ang_y;
        cmd.angular.z = ang_z;

        broadcast(cmd);
        publisher_.publish(cmd);
    }
    
    void broadcast(const geometry_msgs::Twist msg){
        ROS_INFO("Publishing Twist:");
        ROS_INFO("linear x = %f", msg.linear.x);
        ROS_INFO("linear y = %f", msg.linear.y);
        ROS_INFO("linear z = %f", msg.linear.z);

        ROS_INFO("angular x = %f", msg.angular.x);
        ROS_INFO("angular y = %f", msg.angular.y);
        ROS_INFO("angular z = %f", msg.angular.z);
        ROS_INFO("--------------------------------");
    }

};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "vel_pub_node");
    VelPubNode vel_pub_node;
    ros::spin();
    return 0;
}