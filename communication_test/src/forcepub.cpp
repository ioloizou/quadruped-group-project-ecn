#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Wrench.h>

class ForcePubNode : public ros::NodeHandle
{
public:
    ForcePubNode() : ros::NodeHandle("~")
    {
        param<double>("f_x", f_x, 0.0); //parameter name, variable to store parameter, default value. 
        param<double>("f_y", f_y, 10.0);
        param<double>("f_z", f_z, 0.0);
        
        param<double>("m_x", m_x, 0.0);
        param<double>("m_y", m_y, 0.0);
        param<double>("m_z", m_z, 0.0);

        publisher_ = advertise<geometry_msgs::Wrench>("/apply_force/trunk", 10);
        
        loop_timer_ = createTimer(ros::Duration(3), &ForcePubNode::loop_callback, this);

        // timer_ = createTimer(ros::Duration(0.05), &ForcePubNode::publisher_callback, this);    
    }
private:
    
    ros::Publisher publisher_;
    ros::Timer loop_timer_;

    double f_x;
    double f_y;
    double f_z;

    double m_x;
    double m_y;
    double m_z;

    void apply_force(bool opposite){
        geometry_msgs::Wrench cmd;
        if (opposite == true){
            f_x = -f_x;
            f_y = -f_y;
            f_z = -f_z;
        }
        
        cmd.force.x = f_x;
        cmd.force.y = f_y;
        cmd.force.z = f_z;

        cmd.torque.x = m_x;
        cmd.torque.y = m_y;
        cmd.torque.z = m_z;

        broadcast(cmd);
        publisher_.publish(cmd);
    }

    void reset_force(){
        geometry_msgs::Wrench cmd;
        cmd.force.x = 0.0;
        cmd.force.y = 0.0;
        cmd.force.z = 0.0;

        cmd.torque.x = 0.0;
        cmd.torque.y = 0.0;
        cmd.torque.z = 0.0;

        broadcast(cmd);
        publisher_.publish(cmd);
    }

    void loop_callback(const ros::TimerEvent& event) {
        apply_force(true);
        ros::Duration(0.6).sleep();
        reset_force();
    }

    void broadcast(const geometry_msgs::Wrench msg){
        ROS_INFO("Publishing Twist:");
        ROS_INFO("force x = %f", msg.force.x);
        ROS_INFO("force y = %f", msg.force.y);
        ROS_INFO("force z = %f", msg.force.z);

        ROS_INFO("torque x = %f", msg.torque.x);
        ROS_INFO("torque y = %f", msg.torque.y);
        ROS_INFO("torque z = %f", msg.torque.z);
        ROS_INFO("--------------------------------");
    }

};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "force_pub_node");
    ForcePubNode force_pub_node;
   
    ros::spin(); 
    return 0;
}