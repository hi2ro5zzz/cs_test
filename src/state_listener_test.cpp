#include <iostream>

#include <ros/ros.h>
#include <gazebo_msgs/ContactsState.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Vector3.h>
#include <gazebo_msgs/LinkStates.h>

using namespace std;

gazebo_msgs::ContactsState s1,s2;
gazebo_msgs::LinkStates ls;


// ros::Publisher cs_pub1;
// ros::Publisher cs_pub2;
// ros::Publisher capacitance_pub;
ros::Publisher statepub;

// geometry_msgs::Wrench force1;
// geometry_msgs::Vector3 pos1;

// void Sensor1CB(const gazebo_msgs::ContactsState::ConstPtr msg)
// {
//     if(msg->states.size()>0)
//     {
//         s1.states = msg->states;

//         // Gazeboから得られるデータを格納

//         force1.force.x = s1.states[0].wrenches[0].force.x;
//         force1.force.y = s1.states[0].wrenches[0].force.y;
//         force1.force.z = s1.states[0].wrenches[0].force.z;

//         pos1.x = s1.states[0].contact_positions[0].x;
//         pos1.y = s1.states[0].contact_positions[0].y;
//         pos1.z = s1.states[0].contact_positions[0].z;

//         cs_pub1.publish(force1);
//     }
    
// }

// void Sensor2CB(const gazebo_msgs::ContactsState::ConstPtr msg)
// {
//     if(msg->states.size()>0)
//     {
//         s2.states = msg->states;

//         // Gazeboから得られるデータを格納
//         geometry_msgs::Wrench force2;
//         geometry_msgs::Vector3 pos2;

//         force2.force.x = s2.states[0].wrenches[0].force.x;
//         force2.force.y = s2.states[0].wrenches[0].force.y;
//         force2.force.z = s2.states[0].wrenches[0].force.z;

//         pos2.x = s2.states[0].contact_positions[0].x;
//         pos2.y = s2.states[0].contact_positions[0].y;
//         pos2.z = s2.states[0].contact_positions[0].z;

//         cs_pub2.publish(force2);
//     }
// }

void LinkStateCB(const gazebo_msgs::LinkStates::ConstPtr msg)
{
    // ls.name = msg->name;
    ls.pose = msg->pose;
    ls.twist = msg->twist;
}


int main(int argc, char** argv)
{
    // ros::init(argc, argv, "cs_plot");
    ros::init(argc, argv, "state_listener_test");
    ros::NodeHandle nh;
    
    // s1.states.push_back(*(new gazebo_msgs::ContactState()));
    // s2.states.push_back(*(new gazebo_msgs::ContactState()));

    // Gazeboから取得し，格納したデータをパブリッシュ
    // cs_pub1 = nh.advertise<geometry_msgs::Wrench>("force1",1);
    // cs_pub2 = nh.advertise<geometry_msgs::Wrench>("force2",1);
    statepub = nh.advertise<gazebo_msgs::LinkStates>("listening_state",1);

    // capacitance_pub = nh.advertise<geometry_msgs::Vector3>("capacitance",1);
    
    // Gazeboからデータをサブスクライブ
    // ros::Subscriber sensor1_sub = nh.subscribe("/state1",1,Sensor1CB);
    // ros::Subscriber sensor2_sub = nh.subscribe("/state2",1,Sensor2CB);
    ros::Subscriber link_state_sub = nh.subscribe("/gazebo/link_states",1,LinkStateCB);

    ros::Rate loop_rate(1000);

    // ofstream ofs("result.csv");

    int zone = 0;

    while(ros::ok())
    {
        statepub.publish(ls);
        ros::spinOnce();
        loop_rate.sleep();
    }
}