#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <quad_msgs/GRFArray.h>
#include <quad_msgs/RobotPlan.h>
#include <quad_msgs/RobotState.h>


class LocalPlanSub : public ros::NodeHandle
{
public:
    LocalPlanSub() : ros::NodeHandle("~")
    {
        subscriber_ = subscribe<quad_msgs::RobotPlan>("/robot_1/local_plan", 10, &LocalPlanSub::subscriber_callback, this);   
    }
private:
    
    ros::Subscriber subscriber_;

    void subscriber_callback(const quad_msgs::RobotPlan::ConstPtr& msg){
        broadcast(*msg);        
    }
    
    void broadcast(const quad_msgs::RobotPlan& msg){
        ROS_INFO("Retrived Local Plan:");        
        ROS_INFO("This plan contains: %i States and GRF Vectors", msg.grfs.size());        
        
        for (int i=0; i<msg.grfs.size(); i++){
            
            ROS_INFO("INDEX: %i", i);
            ROS_INFO("-Body State:");
            ROS_INFO("  Position x: %f", msg.states[i].body.pose.position.x);
            ROS_INFO("  Position y: %f", msg.states[i].body.pose.position.x);

            ROS_INFO("  Velocity x: %f", msg.states[i].body.twist.linear.x);
            ROS_INFO("  Velocity y: %f", msg.states[i].body.twist.linear.y);

            
            ROS_INFO("-GRF Contact point(Only first GRF):");
            ROS_INFO("  x = %f", msg.grfs[i].points[0].x);
            ROS_INFO("  y = %f", msg.grfs[i].points[0].y);
            ROS_INFO("  z = %f", msg.grfs[i].points[0].z);

        }

        ROS_INFO("-------------------------------------------------------------");
        
    }

};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "local_plan_sub");
    LocalPlanSub localplansub_node;
    ros::spin();
    return 0;
}