#include <iostream>

#include <ros/ros.h>
#include <gazebo_msgs/ContactsState.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>

gazebo_msgs::ContactsState s1,s2;

ros::Publisher forcepub1;
ros::Publisher forcepub2;

ros::Publisher cs_pub1;
ros::Publisher cs_pub2;

void Sensor1CB(const gazebo_msgs::ContactsState::ConstPtr msg)
{
    if(msg->states.size()>0)
    {
        s1.states = msg->states;
        // std_msgs::Float64 lol;
        // lol.data = s1.states[0].wrenches[0].force.x;
        // lol.data = s1.states[0].wrenches[0].force.z;
        // forcepub1.publish(lol);

        geometory_msgs::Vector3 force1;
        force1.x = s1.states[0].wrenches[0].force.x;
        force1.y = s1.states[0].wrenches[0].force.y;
        force1.z = s1.states[0].wrenches[0].force.z;
        cs_pub1.publish(force1);
    }
    
}

void Sensor2CB(const gazebo_msgs::ContactsState::ConstPtr msg)
{
    if(msg->states.size()>0)
    {
        s2.states = msg->states;
        // std_msgs::Float64 lol;
        // lol.data = s2.states[0].wrenches[0].force.x;
        // lol.data = s2.states[0].wrenches[0].force.z;
        // forcepub2.publish(lol);

        geometory_msgs::Vector3 force2;
        force2.x = s1.states[0].wrenches[0].force.x;
        force2.y = s1.states[0].wrenches[0].force.y;
        force2.z = s1.states[0].wrenches[0].force.z;
        cs_pub2.publish(force2);
    }
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "cs_plot");
    ros::NodeHandle nh;
    
    s1.states.push_back(*(new gazebo_msgs::ContactState()));
    s2.states.push_back(*(new gazebo_msgs::ContactState()));

    // ros::Publisher cb_pub = nh.advertise<std_msgs::String>("cb_msg",1);
    // ros::Publisher start_pub = nh.advertise<std_msgs::Int32>("start_msg",1);
    forcepub1 = nh.advertise<std_msgs::Float64>("xforce1",1);
    forcepub2 = nh.advertise<std_msgs::Float64>("xforce2",1);

    cs_pub1 = nh.advertise<geometry_msgs::Vector3>("force1",1);
    cs_pub2 = nh.advertise<geometry_msgs::Vector3>("force2",1);

    // forcepub1 = nh.advertise<std_msgs::Float64>("zforce1",1);
    // forcepub2 = nh.advertise<std_msgs::Float64>("zforce2",1);
    
    ros::Subscriber sensor1_sub = nh.subscribe("/state1",1,Sensor1CB);
    ros::Subscriber sensor2_sub = nh.subscribe("/state2",1,Sensor2CB);

    ros::Rate loop_rate(100);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

    
