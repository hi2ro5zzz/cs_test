#include <iostream>
#include <fstream>
#include <time.h>

#include <ros/ros.h>
#include <gazebo_msgs/ContactsState.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Vector3.h>

using namespace std;

gazebo_msgs::ContactsState s1,s2;


ros::Publisher cs_pub1;
ros::Publisher cs_pub2;
ros::Publisher capacitance_pub;

geometry_msgs::Wrench force1;
geometry_msgs::Vector3 pos1;

void Sensor1CB(const gazebo_msgs::ContactsState::ConstPtr msg)
{
    if(msg->states.size()>0)
    {
        s1.states = msg->states;

        // Gazeboから得られるデータを格納

        force1.force.x = s1.states[0].wrenches[0].force.x;
        force1.force.y = s1.states[0].wrenches[0].force.y;
        force1.force.z = s1.states[0].wrenches[0].force.z;

        pos1.x = s1.states[0].contact_positions[0].x;
        pos1.y = s1.states[0].contact_positions[0].y;
        pos1.z = s1.states[0].contact_positions[0].z;

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
        geometry_msgs::Vector3 pos2;

        force2.force.x = s2.states[0].wrenches[0].force.x;
        force2.force.y = s2.states[0].wrenches[0].force.y;
        force2.force.z = s2.states[0].wrenches[0].force.z;

        pos2.x = s2.states[0].contact_positions[0].x;
        pos2.y = s2.states[0].contact_positions[0].y;
        pos2.z = s2.states[0].contact_positions[0].z;

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

    capacitance_pub = nh.advertise<geometry_msgs::Vector3>("capacitance",1);
    
    // Gazeboからデータをサブスクライブ
    ros::Subscriber sensor1_sub = nh.subscribe("/state1",1,Sensor1CB);
    ros::Subscriber sensor2_sub = nh.subscribe("/state2",1,Sensor2CB);

    ros::Rate loop_rate(1000);

    // struct tm now;
    // {
    //     const time_t t = time(NULL);
    //     localtime_r(&t,&now);
    // }

    // char fname[32] = {0};
    // strftime(fname,sizeof(fname),"%Y%m%d%H%M.csv",&now);

    ofstream ofs("result.csv");

    int zone = 0;

    while(ros::ok())
    {
        geometry_msgs::Vector3 capacitance;
      // calculate capacitanceacitanceacitance
      if( pos1.x < 0.5 && pos1.x > -0.5 && 
          pos1.y < 0.5 && pos1.y > -0.5 )
               {
                  zone = 1;
                  capacitance.x = 0;
                  capacitance.y = 0;
                  capacitance.z = 0.0092 * std::pow( force1.force.z , 2 ) + 0.0393 * force1.force.z + 1.3318;
               }
      else if( pos1.x < 1.0 && pos1.x >  0.5 && 
               pos1.y < 0.5 && pos1.y > -0.5 )
               {
                  zone = 2;
                  capacitance.x = 0;
                  capacitance.y = 0;
                  capacitance.z = 0.0106 * std::pow( force1.force.z , 2 ) + 0.0361 * force1.force.z + 1.3321;
               }
      else if( pos1.x < 1.5 && pos1.x >  1.0 && 
               pos1.y < 0.5 && pos1.y > -0.5 )
               {
                  zone = 3;
                  capacitance.x = 0;
                  capacitance.y = 0;
                  capacitance.z = 0.0068 * std::pow( force1.force.z , 2 ) + 0.0478 * force1.force.z + 1.3308;
               }
      else if( pos1.x < -0.5 && pos1.x > -1.0 && 
               pos1.y <  0.5 && pos1.y > -0.5 )
               {
                  zone = 4;
                  capacitance.x = 0;
                  capacitance.y = 0;
                  capacitance.z = 0.0082 * std::pow( force1.force.z , 2 ) + 0.0436 * force1.force.z + 1.3316;
               }
      else if( pos1.x < -1.0 && pos1.x > -1.5 && 
               pos1.y <  0.5 && pos1.y > -0.5 )
               {
                  zone = 5;
                  capacitance.x = 0;
                  capacitance.y = 0;
                  capacitance.z = 0.0041 * std::pow( force1.force.z , 2 ) + 0.0538 * force1.force.z + 1.3298;
               }
      else if( pos1.x < 0.5 && pos1.x > -0.5 && 
               pos1.y < 1.0 && pos1.y >  0.5 )
               {
                  zone = 6;
                  capacitance.x = 0;
                  capacitance.y = 0;
                  capacitance.z = 0.0100 * std::pow( force1.force.z , 2 ) + 0.0419 * force1.force.z + 1.3324;
               }
      else if( pos1.x < 0.5 && pos1.x > -0.5 && 
               pos1.y < 1.5 && pos1.y >  1.0 )
               {
                  zone = 7;
                  capacitance.x = 0;
                  capacitance.y = 0;
                  capacitance.z = 0.0059 * std::pow( force1.force.z , 2 ) + 0.0533 * force1.force.z + 1.3320;
               }
      else if( pos1.x <  0.5 && pos1.x > -0.5 && 
               pos1.y < -0.5 && pos1.y > -1.0 )
               {
                  zone = 8;
                  capacitance.x = 0;
                  capacitance.y = 0;
                  capacitance.z = 0.0082 * std::pow( force1.force.z , 2 ) + 0.0448 * force1.force.z + 1.3322;
               }
      else if( pos1.x <  0.5 && pos1.x > -0.5 && 
               pos1.y < -1.0 && pos1.y > -1.5 )
               {
                  zone = 9;
                  capacitance.x = 0;
                  capacitance.y = 0;
                  capacitance.z = 0.0036 * std::pow( force1.force.z , 2 ) + 0.0550 * force1.force.z + 1.3306;
               }
      else
               {
                  zone = 0;
                  capacitance.x = 0;
                  capacitance.y = 0;
                  capacitance.z = 0.0;
               }

        ofs << zone << "," << pos1.x << "," << pos1.y << "," << force1.force.z << "," << capacitance.z << "," << endl; 

        capacitance_pub.publish(capacitance);
        ros::spinOnce();
        loop_rate.sleep();
    }
}