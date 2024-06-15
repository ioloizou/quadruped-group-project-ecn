#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Wrench.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "communication_node");
    ros::NodeHandle n;                                  

    //declare publisher
    ros::Publisher test_publisher = n.advertise<geometry_msgs::Wrench>("/robot1/apply_force/trunk",10); //(topic name, buffer size (qos))

    ros::Rate loop_rate(10);    //rate to publish the message (here 10Hz)
    int count = 0;

    while (ros::ok()){
        //build msg
        std_msgs::String msg;
        std::stringstream ss;

        ss << "hello world" << count;
        msg.data = ss.str();

        //publish and output
        ROS_INFO("%s", msg.data.c_str());
        test_publisher.publish(msg);
    
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}

