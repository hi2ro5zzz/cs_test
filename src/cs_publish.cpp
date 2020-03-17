#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cs_publisher");
    ros::NodeHandle nh;
    
    ros::Publisher cs_pub = nh.advertise<geometry_msgs::Wrench>("cs_msg",1);

    ros::Rate loop_rate(1000);

    geometry_msgs::Wrench force;
    int i = 0;

    while(ros::ok())
    {
        // プラグインへ力の情報を出力
        force.force.x = sin(i*3.14/60);
        force.force.y = cos(i*3.14/60);
        force.force.z = 1;

        cs_pub.publish(force);
        i++;

        ros::spinOnce();
        loop_rate.sleep();

    }

    force.force.x = 0;
    force.force.y = 0;
    force.force.z = 0;
    cs_pub.publish(force);
}

    
