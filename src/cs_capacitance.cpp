#include <iostream>
#include <fstream>
#include <time.h>
#include <cmath>

#include <ros/ros.h>

#include <gazebo_msgs/ContactsState.h>
#include <gazebo_msgs/LinkStates.h>

#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Vector3.h>

#include "cs_test/cpstate.h"


using namespace std;

gazebo_msgs::ContactsState s1,s2;
gazebo_msgs::LinkStates ls;

ros::Publisher cpstate_pub;

geometry_msgs::Wrench force1;
geometry_msgs::Vector3 contactpos1,contactpos2,linkpos1;
cs_test::cpstate cpstate;

void Sensor1CB(const gazebo_msgs::ContactsState::ConstPtr msg)
{
    if(msg->states.size()>0)
    {
        s1.states = msg->states;

        // Gazeboから得られるデータを格納

        force1.force.x = s1.states[0].wrenches[0].force.x;
        force1.force.y = s1.states[0].wrenches[0].force.y;
        force1.force.z = s1.states[0].wrenches[0].force.z;

        contactpos1.x = s1.states[0].contact_positions[0].x;
        contactpos1.y = s1.states[0].contact_positions[0].y;
        contactpos1.z = s1.states[0].contact_positions[0].z;
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

        contactpos2.x = s2.states[0].contact_positions[0].x;
        contactpos2.y = s2.states[0].contact_positions[0].y;
        contactpos2.z = s2.states[0].contact_positions[0].z;
    }
}

void LinkStateCB(const gazebo_msgs::LinkStates::ConstPtr msg)
{
    ls.name = msg->name;
    ls.pose = msg->pose;
    
    for(int i=0;i<ls.name.size();i++)
    {
       if(ls.name[i]=="j2s6s200::o_sensor1")
       {
         linkpos1.x = ls.pose[i].position.x;
         linkpos1.y = ls.pose[i].position.y;
         linkpos1.z = ls.pose[i].position.z;
       }
    }  
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "cs_state");
    ros::NodeHandle nh;
    
    s1.states.push_back(*(new gazebo_msgs::ContactState()));
    s2.states.push_back(*(new gazebo_msgs::ContactState()));

    cpstate_pub = nh.advertise<cs_test::cpstate>("cpstate",1);
    
    // Gazeboからデータをサブスクライブ
    ros::Subscriber s1_sub = nh.subscribe("contact_sensor_state_o1",1,Sensor1CB);
    ros::Subscriber link_state_sub = nh.subscribe("/gazebo/link_states",1,LinkStateCB);

    ros::Rate loop_rate(1000);

    // struct tm now;
    // {
    //     const time_t t = time(NULL);
    //     localtime_r(&t,&now);
    // }

    // char fname[32] = {0};
    // strftime(fname,sizeof(fname),"%Y%m%d%H%M.csv",&now);

    ofstream ofs("result.csv");

    while(ros::ok())
    {
        cpstate.relativeposition.x = contactpos1.x - linkpos1.x;
        cpstate.relativeposition.y = contactpos1.y - linkpos1.y;
        cpstate.relativeposition.z = contactpos1.z - linkpos1.z;

        double r = sqrt(std::pow(cpstate.relativeposition.x , 2) + std::pow(cpstate.relativeposition.y , 2));
        double theta = atan2(cpstate.relativeposition.y, cpstate.relativeposition.x);
        
      // calculate capacitanceacitanceacitance
      if( r <= 0.08E-3 )
               {
                  cpstate.zone = 1;
                  cpstate.capacitance.x = 0;
                  cpstate.capacitance.y = 0;
                  if(force1.force.z < 4)
                  {
                     cpstate.capacitance.z = 0.0092 * std::pow( force1.force.z , 2 ) + 0.0393 * force1.force.z + 1.3318;
                     cpstate.calforce.force.z = 13.98 * cpstate.capacitance.z - 18.529;
                  }
                  else
                  {
                     cpstate.capacitance.z = - 0.051 * std::pow( force1.force.z , 2 ) + 0.6068 * force1.force.z + 0.0369;
                  }  
               }
      else if( (r > 0.08E-3 && r <= 0.24E-3) &&
               (theta > -M_PI/4 && theta <= M_PI/4) )
               {
                  cpstate.zone = 2;
                  cpstate.capacitance.x = 0;
                  cpstate.capacitance.y = 0;
                  if(force1.force.z < 4)
                  {
                     cpstate.capacitance.z = 0.0106 * std::pow( force1.force.z , 2 ) + 0.0361 * force1.force.z + 1.3321;
                     cpstate.calforce.force.z = 13.98 * cpstate.capacitance.z - 18.529;
                  }
                  else
                  {
                     cpstate.capacitance.z = 0.0337 * force1.force.z + 1.5125;
                  }  
               }
      else if( (r > 0.24E-3 && r <= 0.40E-3) &&
               (theta > -M_PI/4 && theta <= M_PI/4) )
               {
                  cpstate.zone = 3;
                  cpstate.capacitance.x = 0;
                  cpstate.capacitance.y = 0;
                  if(force1.force.z < 3.5)
                  {
                     cpstate.capacitance.z = 0.0096 * std::pow( force1.force.z , 2 ) + 0.0408 * force1.force.z + 1.3315;
                     cpstate.calforce.force.z = 13.98 * cpstate.capacitance.z - 18.529;
                  }
                  else
                  {
                     cpstate.capacitance.z = 0.0156 * force1.force.z + 1.5426;
                  }   
               }
      else if( (r > 0.08E-3 && r <= 0.24E-3) &&
               ((theta > 3*M_PI/4 && theta <= M_PI) || (theta > -M_PI && theta <= -3*M_PI/4)) )
               {
                  cpstate.zone = 4;
                  cpstate.capacitance.x = 0;
                  cpstate.capacitance.y = 0;
                  if(force1.force.z < 3.5)
                  {
                     cpstate.capacitance.z = 0.0086 * std::pow( force1.force.z , 2 ) + 0.0425 * force1.force.z + 1.3317;
                     cpstate.calforce.force.z = 13.98 * cpstate.capacitance.z - 18.529;
                  }
                  else
                  {
                     cpstate.capacitance.z = - 0.0045 * std::pow( force1.force.z , 2 ) + 0.0682 * force1.force.z + 1.4196;
                  }  
               }
      else if( (r > 0.24E-3 && r <= 0.40E-3) &&
               ((theta > 3*M_PI/4 && theta <= M_PI) || (theta > -M_PI && theta <= -3*M_PI/4)) )
               {
                  cpstate.zone = 5;
                  cpstate.capacitance.x = 0;
                  cpstate.capacitance.y = 0;
                  if(force1.force.z < 3.3)
                  {
                     cpstate.capacitance.z = 0.0087 * std::pow( force1.force.z , 2 ) + 0.0426 * force1.force.z + 1.3309;
                     cpstate.calforce.force.z = 13.98 * cpstate.capacitance.z - 18.529;
                  }
                  else
                  {
                     cpstate.capacitance.z = - 0.003 * std::pow( force1.force.z , 2 ) + 0.0422 * force1.force.z + 1.4635;
                  }                  
               }
      else if( (r > 0.08E-3 && r <= 0.24E-3) &&
               (theta > M_PI/4 && theta <= 3*M_PI/4) )
               {
                  cpstate.zone = 7;
                  cpstate.capacitance.x = 0;
                  cpstate.capacitance.y = 0;
                  if(force1.force.z < 4)
                  {
                     cpstate.capacitance.z = 0.0100 * std::pow( force1.force.z , 2 ) + 0.0419 * force1.force.z + 1.3324;
                     cpstate.calforce.force.z = 13.98 * cpstate.capacitance.z - 18.529;
                  }
                  else
                  {
                     cpstate.capacitance.z = 0.0387 * force1.force.z + 1.5077;
                  }                  
               }
      else if( (r > 0.24E-3 && r <= 0.40E-3) &&
               (theta > M_PI/4 && theta <= 3*M_PI/4) )
               {
                  cpstate.zone = 8;
                  cpstate.capacitance.x = 0;
                  cpstate.capacitance.y = 0;
                  if(force1.force.z < 3.5)
                  {
                     cpstate.capacitance.z = 0.0089 * std::pow( force1.force.z , 2 ) + 0.0463 * force1.force.z + 1.3327;
                     cpstate.calforce.force.z = 13.98 * cpstate.capacitance.z - 18.529;
                  }
                  else
                  {
                     cpstate.capacitance.z = 0.0168 * force1.force.z + 1.5440;
                  }                  
               }
      else if( (r > 0.08E-3 && r <= 0.24E-3) &&
               (theta > -3*M_PI/4 && theta <= -M_PI/4) )
               {
                  cpstate.zone = 9;
                  cpstate.capacitance.x = 0;
                  cpstate.capacitance.y = 0;
                  if(force1.force.z < 3.5)
                  {
                     cpstate.capacitance.z = 0.0084 * std::pow( force1.force.z , 2 ) + 0.0441 * force1.force.z + 1.3323;
                     cpstate.calforce.force.z = 13.98 * cpstate.capacitance.z - 18.529;
                  }
                  else
                  {
                     cpstate.capacitance.z = - 0.0075 * std::pow( force1.force.z , 2 ) + 0.0946 * force1.force.z + 1.3675;
                  }                  
               }
      else if( (r > 0.24E-3 && r <= 0.40E-3) &&
               (theta > -3*M_PI/4 && theta <= -M_PI/4) )
               {
                  cpstate.zone = 10;
                  cpstate.capacitance.x = 0;
                  cpstate.capacitance.y = 0;
                  if(force1.force.z < 3.3)
                  {
                     cpstate.capacitance.z = 0.0080 * std::pow( force1.force.z , 2 ) + 0.0443 * force1.force.z + 1.3318;
                     cpstate.calforce.force.z = 13.98 * cpstate.capacitance.z - 18.529;
                  }
                  else
                  {
                     cpstate.capacitance.z = - 0.0021 * std::pow( force1.force.z , 2 ) + 0.0348 * force1.force.z + 1.4769;
                  }                  
               }
      else
               {
                  cpstate.zone = 0;
                  cpstate.capacitance.x = 0;
                  cpstate.capacitance.y = 0;
                  cpstate.capacitance.z = 0.0;
               }


      ofs << cpstate.zone << "," << cpstate.relativeposition.x << "," << cpstate.relativeposition.y << "," << force1.force.z << "," << cpstate.capacitance.z << "," << endl; 

      cpstate_pub.publish(cpstate);
      ros::spinOnce();
      loop_rate.sleep();
    }
}