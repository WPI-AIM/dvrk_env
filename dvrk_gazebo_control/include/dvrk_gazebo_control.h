#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <boost/bind.hpp>
#include <gazebo_msgs/LinkState.h>
#include <gazebo_msgs/LinkStates.h>
//#include <opencv2/opencv.hpp>
//#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sstream>
#include "std_msgs/String.h"

class dvrk_gazebo_control{
private:
  std::vector<ros::Publisher> ecmPub, psm1Pub, psm2Pub, psm3Pub;
  ros::Subscriber link_states, image_sub;
  ros::Publisher plot_x, plot_y, plot_z;
  std::vector<ros::Publisher> cartPub;
public:
  dvrk_gazebo_control(ros::NodeHandle n){
    ecmPub.resize(4);
    ecmPub[0]=n.advertise<std_msgs::Float64>("/dvrk/ecm/yaw_joint/SetPositionTarget",10);
    ecmPub[1]=n.advertise<std_msgs::Float64>("/dvrk/ecm/pitch_front_joint/SetPositionTarget",10);
    ecmPub[2]=n.advertise<std_msgs::Float64>("/dvrk/ecm/main_insertion_joint/SetPositionTarget",10);
    ecmPub[3]=n.advertise<std_msgs::Float64>("/dvrk/ecm/tool_joint/SetPosition",1000);


    psm1Pub.resize(5);
    psm1Pub[0]=n.advertise<std_msgs::Float64>("/dvrk/PSM1/outer_yaw_joint/SetPositionTarget",10);
    psm1Pub[1]=n.advertise<std_msgs::Float64>("/dvrk/PSM1/outer_pitch_joint_1/SetPositionTarget",10);
    psm1Pub[2]=n.advertise<std_msgs::Float64>("/dvrk/PSM1/outer_insertion_joint/SetPositionTarget",10);
    psm1Pub[3]=n.advertise<std_msgs::Float64>("/dvrk/PSM1/outer_roll_joint/SetPosition",10);
    psm1Pub[4]=n.advertise<std_msgs::Float64>("/dvrk/PSM1/rev_joint/SetPositionTarget",10);

    psm2Pub.resize(5);
    psm2Pub[0]=n.advertise<std_msgs::Float64>("/dvrk/PSM2/outer_yaw_joint/SetPositionTarget",10);
    psm2Pub[1]=n.advertise<std_msgs::Float64>("/dvrk/PSM2/outer_pitch_joint_1/SetPositionTarget",10);
    psm2Pub[2]=n.advertise<std_msgs::Float64>("/dvrk/PSM2/outer_insertion_joint/SetPositionTarget",10);
    psm2Pub[3]=n.advertise<std_msgs::Float64>("/dvrk/PSM2/outer_roll_joint/SetPosition",10);
    psm2Pub[4]=n.advertise<std_msgs::Float64>("/dvrk/PSM2/rev_joint/SetPositionTarget",10);

     
    psm3Pub.resize(5);
    psm3Pub[0]=n.advertise<std_msgs::Float64>("/dvrk/PSM3/outer_yaw_joint/SetPositionTarget",1000);
    psm3Pub[1]=n.advertise<std_msgs::Float64>("/dvrk/PSM3/outer_pitch_joint_1/SetPositionTarget",1000);
    psm3Pub[2]=n.advertise<std_msgs::Float64>("/dvrk/PSM3/outer_insertion_joint/SetPositionTarget",1000);
    psm3Pub[3]=n.advertise<std_msgs::Float64>("/dvrk/PSM3/outer_roll_joint/SetPosition",1000);
    psm3Pub[4]=n.advertise<std_msgs::Float64>("/dvrk/PSM3/rev_joint/SetPositionTarget",1000);

    plot_x=n.advertise<std_msgs::Float64>("/plotx",1000);
    plot_y=n.advertise<std_msgs::Float64>("/ploty",1000);
    plot_z=n.advertise<std_msgs::Float64>("/plotz",1000);
    cartPub.resize(19);
    char topic[100];
    for (int i=1;i<4;i++)
    {
      for (int j=0;j<5;j++)
      {
        //sprintf(topic, "/dvrk/SUJ/SUJ_psm1%d_J%d/SetPosition", i,j);
        sprintf(topic, "/dvrk/SUJ/SUJ_PSM%d_J%d/SetPositionTarget", i,j);

        cartPub[5*(i-1)+j] = n.advertise<std_msgs::Float64>(topic,1000);
      }
    }
    for (int j=0;j<4;j++)
    {
      sprintf(topic, "/dvrk/SUJ/SUJ_ECM_J%d/SetPositionTarget",j);

      cartPub[15+j] = n.advertise<std_msgs::Float64>(topic,1000);
    }
    //link_states=n.subscribe("/gazebo/link_states", 100, &dvrk_gazebo_control::getECMEndEffector, this);
  }

  void showImage(const sensor_msgs::ImageConstPtr& img);
  void getECMEndEffector(const gazebo_msgs::LinkStatesPtr &msg);
  void PublishCartStates();
  void PublishECMStates();
  void PublishPSM1States();
  void PublishPSM2States();
  void PublishPSM3States();

};
