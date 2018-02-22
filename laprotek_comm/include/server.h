#include <iostream>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <unistd.h>
// #include <time.h>
#include <stdint.h>
// #include <fstream>
#include <iomanip>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <signal.h>
// #include "tf/LinearMath/Matrix3x3.h"
// #include "tf/Quaternion.h"
#include <tf/transform_datatypes.h>
#include <string.h>

// #define IP "192.168.1.205"
#define IP "127.0.0.1"
#define DATATYPE 0

class Server{
private:
  int client, server;
  int portNum;
  struct sockaddr_in server_addr;
  int bytes_sent, bytes_recv;
  int in_bufsize;
  int out_bufsize;
  char out_buf[200];
  char in_buf[200];

  double LT[4][4];
  double RT[4][4];
  double LJ[7];
  double RJ[7];
  double MotionScale;
  int clutch;
  int frozen;
public:
  Server(int port)
  {
    portNum=port;
  }
  void initSocket()
  {
    socklen_t size;
    client = socket(AF_INET, SOCK_STREAM, 0);

    if (client < 0)
    {
        std::cout << "\nError establishing socket..." << std::endl;
        exit(1);
    }

    std::cout << "\n=> Socket server has been created..." << std::endl;


    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr =inet_addr(IP);
    server_addr.sin_port = htons(portNum);
  }
  void setPortNumber(int port);
  int ConnectToClient();
  void closeConnection();
  int SendData();
  void ReceiveData();

  void PackData();
  void UnpackData();

  void getRosPoses(geometry_msgs::PoseStamped &leftPose, geometry_msgs::PoseStamped &rightPose);
  void getRosJoints(sensor_msgs::JointState &joints);

  void DebugPrint();
  // void siginthandler(int param);

};

void Server::setPortNumber(int port)
{
  portNum=port;
}

int Server::ConnectToClient()
{
  if ((bind(client, (struct sockaddr*)&server_addr,sizeof(server_addr))) < 0)
  {
      std::cout << "=> Error binding connection, the socket has already been established..." << std::endl;
      return 0;
  }
  else
  {
    std::cout<< "=> CLient Binded to the Socket" << std::endl;
    std::cout << "=> Looking for clients..." << std::endl;

    listen(client, 1);
    socklen_t size;
    size = sizeof(server_addr);
    server = accept(client,(struct sockaddr *)&server_addr,&size);

    std::cout << "=>Found Client, waiting for server to accept..." << '\n';
    if (server<0)
    {
      std::cout << "Error on accepting" << '\n';
    }
    else
    {
      std::cout << "=> Connection with Client established" << std::endl;
    }
    return 1;
  }
}

int Server::SendData()
{
  bytes_sent=send(server,out_buf,200, 0);
  if (bytes_sent<0)
    return 0;
  return 1;
}

void Server::ReceiveData()
{
  bytes_recv=recv(server,in_buf,200, 0);
}

void Server::PackData()
{
  snprintf(out_buf, 2, "%d", DATATYPE);
}

void Server::UnpackData()
{
  char message_substring[7];
  char message_substring_scale[2];
  char message_substring_clutch[1];
  int iterator=0;
  int i, j, hand;

  if (DATATYPE==0)
    for (hand=0;hand<=1;hand++)
    {
      for (i=0; i<4; i++)
      {
        for (j=0; j<3; j++)
        {
          memcpy(message_substring,in_buf+iterator,8);
          if (hand==0)
            LT[j][i]=atof(message_substring)/100000;
          else
            RT[j][i]=atof(message_substring)/100000;
          iterator=iterator+8;
        }
      }
    }
  else
    for (int hand=0;hand<=1;hand++)
      for (i=0;i<7;i++)
      {
        memcpy(message_substring,in_buf+iterator,8);
        if (hand==0)
          LJ[i]=atof(message_substring)/100000;
        else
          RJ[i]=atof(message_substring)/100000;
        iterator=iterator+8;
      }
  memcpy(message_substring_scale,in_buf+iterator,2);
  // printf("%d, %s\n",iterator,message_substring_scale);
  MotionScale=atof(message_substring_scale)/10;
  iterator=iterator+2;
  memcpy(message_substring_clutch,in_buf+iterator,1);
  clutch=atoi(message_substring_clutch);
  iterator=iterator+1;
  memcpy(message_substring_clutch,in_buf+iterator,1);
  frozen=atoi(message_substring_clutch);
  iterator=iterator+1;
}

void Server::closeConnection()
{
  std::cout << "\n\n=> Connection terminated with IP " << inet_ntoa(server_addr.sin_addr);
  close(server);
  close(client);
  std::cout << "\nGoodbye..." << std::endl;
}

void Server::getRosPoses(geometry_msgs::PoseStamped &leftPose, geometry_msgs::PoseStamped &rightPose)
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
  tf::Matrix3x3 LeftRotation(LT[0][0], LT[0][1], LT[0][2], LT[1][0], LT[1][1], LT[1][2], LT[2][0], LT[2][1], LT[2][2]);
  tf::Quaternion LeftQuaternion;
  LeftRotation.getRotation(LeftQuaternion);
  LeftQuaternion.normalize();
  leftPose.pose.orientation.x=LeftQuaternion.getX();
  leftPose.pose.orientation.y=LeftQuaternion.getY();
  leftPose.pose.orientation.z=LeftQuaternion.getZ();
  leftPose.pose.orientation.w=LeftQuaternion.getW();

  tf::Matrix3x3 RightRotation(RT[0][0], RT[0][1], RT[0][2], RT[1][0], RT[1][1], RT[1][2], RT[2][0], RT[2][1], RT[2][2]);
  tf::Quaternion RightQuaternion;
  RightRotation.getRotation(RightQuaternion);
  RightQuaternion.normalize();
  rightPose.pose.orientation.x=RightQuaternion.getX();
  rightPose.pose.orientation.y=RightQuaternion.getY();
  rightPose.pose.orientation.z=RightQuaternion.getZ();
  rightPose.pose.orientation.w=RightQuaternion.getW();
  std::cout << RightQuaternion.getX() << '\t' << RightQuaternion.getY() << '\t' << RightQuaternion.getZ() << '\t' << RightQuaternion.getW() << '\n';
}

void Server::getRosJoints(sensor_msgs::JointState &joints)
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
  joints.position[1]=LJ[1]-0.33;
  joints.position[2]=LJ[2]-0.88;
  joints.position[3]=LJ[3];
  joints.position[4]=LJ[4];
  joints.position[5]=LJ[5];
  joints.position[6]=LJ[6];
  joints.position[7]=RJ[0];
  joints.position[8]=RJ[1]-0.33;
  joints.position[9]=RJ[2]-0.88;
  joints.position[10]=RJ[3];
  joints.position[11]=RJ[4];
  joints.position[12]=RJ[5];
  joints.position[13]=RJ[6];
}

void Server::DebugPrint()
{
  std::cout << bytes_sent << '\t' << bytes_recv << '\n';
}
