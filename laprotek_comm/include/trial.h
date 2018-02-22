#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <vector>
// #include <boost/bind.hpp>
// #include <boost/function.hpp>
// #include "Conversion.h"
// #include "LaprotekRosBridge.h"

class trial{
private:
  double LT[4][4];
  double RT[4][4];
  double *LJ;
  double *RJ;
public:
  void setTransforms(double L[][4], double R[][4]);
  void setJoints(double *L, double *R);
  void getRosPoses( geometry_msgs::PoseStamped &leftPose, geometry_msgs::PoseStamped &rightPose);
  void getRosJoints( sensor_msgs::JointState &joints);
  void print();
};

void trial::setTransforms(double L[][4], double R[][4])
{
  for (int i=0; i<4;i++)
    for (int j=0;j<4;j++)
    {
      LT[i][j]=L[i][j];
      RT[i][j]=R[i][j];
      // std::cout<<LT[i][j]<<"\t"<<RT[i][j]<<"\n";
    }
}

void trial::setJoints(double *L, double *R)
{
  LJ=L;
  RJ=R;
}

void trial::getRosPoses(geometry_msgs::PoseStamped &leftPose, geometry_msgs::PoseStamped &rightPose)
{
  ros::Time time=ros::Time::now();
  leftPose.header.stamp=time;
  rightPose.header.stamp=time;
  leftPose.pose.position.x=LT[0][3];
  leftPose.pose.position.y=LT[1][3];
  leftPose.pose.position.z=LT[2][3];
  rightPose.pose.position.x=RT[0][3];
  rightPose.pose.position.y=RT[1][3];
  rightPose.pose.position.z=RT[2][3];
}

void trial::getRosJoints(sensor_msgs::JointState &joints)
{
  joints.name.resize(0);
  ros::Time time=ros::Time::now();
  joints.header.stamp=ros::Time::now();
  joints.name.push_back("left_shoulder_axis");
  joints.name.push_back("left_twist_axis");
  joints.name.push_back("left_forarm_axis");
  joints.name.push_back("left_outer_axis");
  joints.name.push_back("left_inner_axis");
  joints.name.push_back("left_grip_axis");
  joints.name.push_back("left_thumb_axis");
  joints.name.push_back("right_shoulder_axis");
  joints.name.push_back("right_twist_axis");
  joints.name.push_back("right_forarm_axis");
  joints.name.push_back("right_outer_axis");
  joints.name.push_back("right_inner_axis");
  joints.name.push_back("right_grip_axis");
  joints.name.push_back("right_thumb_axis");
  int n=joints.name.size();
  joints.position.resize(n);
  joints.position[0]=LJ[0];
  joints.position[1]=LJ[1];
  joints.position[2]=LJ[2];
  joints.position[3]=LJ[3];
  joints.position[4]=LJ[4];
  joints.position[5]=LJ[5];
  joints.position[6]=LJ[6];
  joints.position[7]=LJ[0];
  joints.position[8]=LJ[1];
  joints.position[9]=LJ[2];
  joints.position[10]=LJ[3];
  joints.position[11]=LJ[4];
  joints.position[12]=LJ[5];
  joints.position[13]=LJ[6];
}

void trial::print()
{
  std::cout << LJ[0] << LJ[1]<< LJ[2]<< LJ[3]<< LJ[4]<< LJ[5]<< LJ[6]<< '\n';
}
