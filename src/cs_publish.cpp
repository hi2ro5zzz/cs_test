#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cs_publisher");
    ros::NodeHandle nh;
    
    ros::Publisher cs_pub = nh.advertise<geometry_msgs::Vector3>("cs_msg",1);

    ros::Rate loop_rate(100);

    geometry_msgs::Vector3 force;

    force.x = 3;
    force.y = 3;
    force.z = 3;

    while(ros::ok())
    {
        cs_pub.publish(force);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

    
