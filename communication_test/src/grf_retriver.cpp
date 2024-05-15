#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <quad_msgs/GRFArray.h>


class GRFRetriver : public ros::NodeHandle
{
public:
    GRFRetriver() : ros::NodeHandle("~")
    {
        subscriber_ = subscribe<quad_msgs::GRFArray>("/robot_1/state/grfs", 10, &GRFRetriver::subscriber_callback, this);   
    }
private:
    
    ros::Subscriber subscriber_;

    void subscriber_callback(const quad_msgs::GRFArray::ConstPtr& msg){
        broadcast(*msg);        
    }
    
    void broadcast(const quad_msgs::GRFArray& msg){
        ROS_INFO("Retrived GRFs:");        
        for (int i=0; i<msg.points.size(); i++){
            
            ROS_INFO("Data for GRFArray at index: %i", i);
            
            ROS_INFO("-POINT:");
            ROS_INFO("  x = %f", msg.points[i].x);
            ROS_INFO("  y = %f", msg.points[i].y);
            ROS_INFO("  z = %f", msg.points[i].z);

            ROS_INFO("-CONTACT STATE:");
            ROS_INFO("  contact = %f", msg.contact_states[i]);

            ROS_INFO("-VECTOR:");
            ROS_INFO("  x = %f", msg.vectors[i].x);
            ROS_INFO("  y = %f", msg.vectors[i].y);
            ROS_INFO("  z = %f", msg.vectors[i].z);
            
            ROS_INFO("-TRAJ. INDEX:");
            ROS_INFO("  index = %f", msg.traj_index);
        }

        ROS_INFO("-------------------------------------------------------------");
        
    }

};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "grf_retriver_node");
    GRFRetriver grf_retiver_node;
    ros::spin();
    return 0;
}