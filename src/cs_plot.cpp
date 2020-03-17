#include <iostream>

#include <ros/ros.h>
#include <gazebo_msgs/ContactsState.h>
#include <geometry_msgs/Wrench.h>

gazebo_msgs::ContactsState s1,s2;


ros::Publisher cs_pub1;
ros::Publisher cs_pub2;

void Sensor1CB(const gazebo_msgs::ContactsState::ConstPtr msg)
{
    if(msg->states.size()>0)
    {
        s1.states = msg->states;

        // Gazeboから得られるデータを格納
        geometry_msgs::Wrench force1;
        force1.force.x = s1.states[0].wrenches[0].force.x;
        force1.force.y = s1.states[0].wrenches[0].force.y;
        force1.force.z = s1.states[0].wrenches[0].force.z;
        cs_pub1.publish(force1);
    }
    
}

void Sensor2CB(const gazebo_msgs::ContactsState::ConstPtr msg)
{
    if(msg->states.size()>0)
    {
        s2.states = msg->states;

        // Gazeboから得られるデータを格納
        geometry_msgs::Wrench force2;
        force2.force.x = s2.states[0].wrenches[0].force.x;
        force2.force.y = s2.states[0].wrenches[0].force.y;
        force2.force.z = s2.states[0].wrenches[0].force.z;
        cs_pub2.publish(force2);
    }
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "cs_plot");
    ros::NodeHandle nh;
    
    s1.states.push_back(*(new gazebo_msgs::ContactState()));
    s2.states.push_back(*(new gazebo_msgs::ContactState()));

    // Gazeboから取得し，格納したデータをパブリッシュ
    cs_pub1 = nh.advertise<geometry_msgs::Wrench>("force1",1);
    cs_pub2 = nh.advertise<geometry_msgs::Wrench>("force2",1);
    
    // Gazeboからデータをサブスクライブ
    ros::Subscriber sensor1_sub = nh.subscribe("/state1",1,Sensor1CB);
    ros::Subscriber sensor2_sub = nh.subscribe("/state2",1,Sensor2CB);

    ros::Rate loop_rate(1000);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

    
