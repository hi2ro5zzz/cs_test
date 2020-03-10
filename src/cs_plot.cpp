#include <iostream>

#include <ros/ros.h>
#include <gazebo_msgs/ContactsState.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

gazebo_msgs::ContactsState s1,s2;

// bool cbFlag = false;
ros::Publisher forcepub1;
ros::Publisher forcepub2;

void Sensor1CB(const gazebo_msgs::ContactsState::ConstPtr msg)
{
    if(msg->states.size()>0)
    {
        s1.states = msg->states;
        std_msgs::Float64 lol;
        lol.data = s1.states[0].wrenches[0].force.x;
        forcepub1.publish(lol);
    }
    
}

void Sensor2CB(const gazebo_msgs::ContactsState::ConstPtr msg)
{
    if(msg->states.size()>0)
    {
        s2.states = msg->states;
        std_msgs::Float64 lol;
        lol.data = s2.states[0].wrenches[0].force.x;
        forcepub2.publish(lol);
    }
}

// void flagCB(const std_msgs::String::ConstPtr& msg)
// {
//     cbFlag = true;
// }




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
    
    ros::Subscriber sensor1_sub = nh.subscribe("/state1",1,Sensor1CB);
    ros::Subscriber sensor2_sub = nh.subscribe("/state2",1,Sensor2CB);

    // ros::Subscriber flag_sub = nh.subscribe("flag_msg",1,flagCB);

    ros::Rate loop_rate(100);

    // bool flag = true;
    // int start = 2;

    // double f1ave,f2ave;
    // int count = 0;
    // std_msgs::String cbmsg;

        // ROS_INFO("waiting cbflag...");
        // start_pub.publish(start);
        
        // std::vector<double> f1,f2;
        while(ros::ok()){
        // while(!cbFlag)
        // {
        //     cbFlag = false;
        //     ros::spinOnce();
        //     loop_rate.sleep();
        // }

        // ROS_INFO("receive cbflag...");
        // ROS_INFO("start checking force...");

        // while(cbFlag)
        // {
            // f1.push_back(s1.states[0].total_wrench.force.y);
            // f2.push_back(s2.states[0].total_wrench.force.y);
            // f1ave = (f1[count-3] + f1[count-2] + f1[count-1]) / 3;
            // f2ave = (f2[count-3] + f2[count-2] + f2[count-1]) / 3;
        
            // if (f1ave < -0.5 && f2ave > 0.5)
            // {
            //     ROS_INFO("Sensor CB");
            //     cbmsg.data = "0";
            //     cb_pub.publish(cbmsg);
            //     cbFlag = false;
            // }
    
        // count++;
        ros::spinOnce();
        loop_rate.sleep();
        }
}

    
