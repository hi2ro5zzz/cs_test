#include <iostream>
#include <fstream>
#include <time.h>
#include <cmath>

#include <ros/ros.h>

#include <gazebo_msgs/ContactsState.h>
#include <gazebo_msgs/LinkStates.h>

#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Vector3.h>


using namespace std;

gazebo_msgs::ContactsState s1,s2;
gazebo_msgs::LinkStates ls;


ros::Publisher cs_pub1;
ros::Publisher cs_pub2;
ros::Publisher capacitance_pub;
ros::Publisher relativepos_pub;

geometry_msgs::Wrench force1,calforce1;
geometry_msgs::Vector3 contactpos1,contactpos2,linkpos1,relativepos1;

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

        contactpos2.x = s2.states[0].contact_positions[0].x;
        contactpos2.y = s2.states[0].contact_positions[0].y;
        contactpos2.z = s2.states[0].contact_positions[0].z;

        cs_pub2.publish(force2);
    }
}

void LinkStateCB(const gazebo_msgs::LinkStates::ConstPtr msg)
{
    ls.name = msg->name;
    ls.pose = msg->pose;
    
    for(int i=0;i<size(ls.name);i++)
    {
       if(ls.name[i]=="sensor1")
       {
         linkpos1.x = ls.pose[i].point.x;
         linkpos1.y = ls.pose[i].point.y;
         linkpos1.z = ls.pose[i].point.z;
       }
    }
    
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "cs_state");
    ros::NodeHandle nh;
    
    s1.states.push_back(*(new gazebo_msgs::ContactState()));
    s2.states.push_back(*(new gazebo_msgs::ContactState()));

    // Gazeboから取得し，格納したデータをパブリッシュ
    cs_pub1 = nh.advertise<geometry_msgs::Wrench>("force1",1);
    cs_pub2 = nh.advertise<geometry_msgs::Wrench>("force2",1);
    relativepos_pub = nh.advertise<geometry_msgs::Vector3>("relative_position",1);

    capacitance_pub = nh.advertise<geometry_msgs::Vector3>("capacitance",1);
    
    // Gazeboからデータをサブスクライブ
    ros::Subscriber sensor1_sub = nh.subscribe("/state1",1,Sensor1CB);
    ros::Subscriber sensor2_sub = nh.subscribe("/state2",1,Sensor2CB);
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

    int zone = 0;

    relativepos1.x = contactpos1.x - linkpos1.x;
    relativepos1.y = contactpos1.y - linkpos1.y;
    relativepos1.z = contactpos1.z - linkpos1.z;

    double r = sqrt(std::pow(relativepos1.x , 2) + std::pow(relativepos1.y , 2));
    double theta = atan2(relativepos1.y, relativepos1.x);

    while(ros::ok())
    {
        geometry_msgs::Vector3 capacitance;
        
      // calculate capacitanceacitanceacitance
      if( r <= 0.08E-3 )
               {
                  zone = 1;
                  capacitance.x = 0;
                  capacitance.y = 0;
                  if(force1.force.z < 4)
                  {
                     capacitance.z = 0.0092 * std::pow( force1.force.z , 2 ) + 0.0393 * force1.force.z + 1.3318;
                     calforce1.z = 13.98*capacitance.z - 18.529;
                  }
                  else
                  {
                     capacitance.z = - 0.051 * std::pow( force1.force.z , 2 ) + 0.6068 * force1.force.z + 0.0369;
                  }
                  
               
               }
      else if( (r > 0.08E-3 && r <= 0.24E-3) &&
               (theta > -M_PI/4 && theta <= M_PI/4) )
               {
                  zone = 2;
                  capacitance.x = 0;
                  capacitance.y = 0;
                  if(force1.force.z < 4)
                  {
                     capacitance.z = 0.0106 * std::pow( force1.force.z , 2 ) + 0.0361 * force1.force.z + 1.3321;
                     calforce1.z = 13.98*capacitance.z - 18.529;
                  }
                  else
                  {
                     capacitance.z = 0.0337 * force1.force.z + 1.5125;
                  }
                  
               }
      else if( (r > 0.24E-3 && r <= 0.40E-3) &&
               (theta > -M_PI/4 && theta <= M_PI/4) )
               {
                  zone = 3;
                  capacitance.x = 0;
                  capacitance.y = 0;
                  if(force1.force.z < 3.5)
                  {
                     capacitance.z = 0.0096 * std::pow( force1.force.z , 2 ) + 0.0408 * force1.force.z + 1.3315;
                     calforce1.z = 13.98*capacitance.z - 18.529;
                  }
                  else
                  {
                     capacitance.z = 0.0156 * force1.force.z + 1.5426;
                  }
                  
               }
      else if( (r > 0.08E-3 && r <= 0.24E-3) &&
               ((theta > 3*M_PI/4 && theta <= M_PI) || (theta > -M*PI && theta <= -3*M_PI/4)) )
               {
                  zone = 4;
                  capacitance.x = 0;
                  capacitance.y = 0;
                  if(force1.force.z < 3.5)
                  {
                     capacitance.z = 0.0086 * std::pow( force1.force.z , 2 ) + 0.0425 * force1.force.z + 1.3317;
                     calforce1.z = 13.98*capacitance.z - 18.529;
                  }
                  else
                  {
                     capacitance.z = - 0.0045 * std::pow( force1.force.z , 2 ) + 0.0682 * force1.force.z + 1.4196;
                  }
                  
               }
      else if( (r > 0.24E-3 && r <= 0.40E-3) &&
               ((theta > 3*M_PI/4 && theta <= M_PI) || (theta > -M*PI && theta <= -3*M_PI/4)) )
               {
                  zone = 5;
                  capacitance.x = 0;
                  capacitance.y = 0;
                  if(force1.force.z < 3.3)
                  {
                     capacitance.z = 0.0087 * std::pow( force1.force.z , 2 ) + 0.0426 * force1.force.z + 1.3309;
                     calforce1.z = 13.98*capacitance.z - 18.529;
                  }
                  else
                  {
                     capacitance.z = - 0.003 * std::pow( force1.force.z , 2 ) + 0.0422 * force1.force.z + 1.4635;
                  }
                  
               }
      else if( (r > 0.08E-3 && r <= 0.24E-3) &&
               (theta > M_PI/4 && theta <= 3*M_PI/4) )
               {
                  zone = 7;
                  capacitance.x = 0;
                  capacitance.y = 0;
                  if(force1.force.z < 4)
                  {
                     capacitance.z = 0.0100 * std::pow( force1.force.z , 2 ) + 0.0419 * force1.force.z + 1.3324;
                     calforce1.z = 13.98*capacitance.z - 18.529;
                  }
                  else
                  {
                     capacitance.z = 0.0387 * force1.force.z + 1.5077;
                  }
                  
               }
      else if( (r > 0.24E-3 && r <= 0.40E-3) &&
               (theta > M_PI/4 && theta <= 3*M_PI/4) )
               {
                  zone = 8;
                  capacitance.x = 0;
                  capacitance.y = 0;
                  if(force1.force.z < 3.5)
                  {
                     capacitance.z = 0.0089 * std::pow( force1.force.z , 2 ) + 0.0463 * force1.force.z + 1.3327;
                     calforce1.z = 13.98*capacitance.z - 18.529;
                  }
                  else
                  {
                     capacitance.z = 0.0168 * force1.force.z + 1.5440;
                  }
                  
               }
      else if( (r > 0.08E-3 && r <= 0.24E-3) &&
               (theta > -3*M_PI/4 && theta <= -M_PI/4) )
               {
                  zone = 9;
                  capacitance.x = 0;
                  capacitance.y = 0;
                  if(force1.force.z < 3.5)
                  {
                     capacitance.z = 0.0084 * std::pow( force1.force.z , 2 ) + 0.0441 * force1.force.z + 1.3323;
                     calforce1.z = 13.98*capacitance.z - 18.529;
                  }
                  else
                  {
                     capacitance.z = - 0.0075 * std::pow( force1.force.z , 2 ) + 0.0946 * force1.force.z + 1.3675;
                  }
                  
               }
      else if( (r > 0.24E-3 && r <= 0.40E-3) &&
               (theta > -3*M_PI/4 && theta <= -M_PI/4) )
               {
                  zone = 10;
                  capacitance.x = 0;
                  capacitance.y = 0;
                  if(force1.force.z < 3.3)
                  {
                     capacitance.z = 0.0080 * std::pow( force1.force.z , 2 ) + 0.0443 * force1.force.z + 1.3318;
                     calforce1.z = 13.98*capacitance.z - 18.529;
                  }
                  else
                  {
                     capacitance.z = - 0.0021 * std::pow( force1.force.z , 2 ) + 0.0348 * force1.force.z + 1.4769;
                  }
                  
               }
      else
               {
                  zone = 0;
                  capacitance.x = 0;
                  capacitance.y = 0;
                  capacitance.z = 0.0;
               }


        ofs << zone << "," << relativepos1.x << "," << relativepos1.y << "," << force1.force.z << "," << capacitance.z << "," << endl; 

        capacitance_pub.publish(capacitance);
        relativepos_pub.publish(relativepos);
        ros::spinOnce();
        loop_rate.sleep();
    }
}