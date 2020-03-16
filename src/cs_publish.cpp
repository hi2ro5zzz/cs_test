#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cs_publisher");
    ros::NodeHandle nh;
    
    ros::Publisher cs_pub = nh.advertise<geometry_msgs::Wrench>("cs_msg",1);

    ros::Rate loop_rate(100);

    geometry_msgs::Wrench force;

    // プラグインへ力の情報を出力
    force.force.x = 3;
    force.force.y = 2;
    force.force.z = 1;

    while(ros::ok())
    {
        cs_pub.publish(force);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

    
