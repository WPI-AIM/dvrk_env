#include "dvrk_gazebo_control.h"



void dvrk_gazebo_control::getECMEndEffector(const gazebo_msgs::LinkStatesPtr &msg)
{
  gazebo_msgs::LinkState ecm_roll;
  for (int i=0;i<msg->pose.size();i++)
  {
    if (!msg->name[i].compare("dvrk::ecm::camera_link"))
    // if (!msg->name[i].compare("dvrk::PSM1::one_tool_wrist_link"))
    {
      ecm_roll.pose = msg->pose[i];
    }
  }
  std::cout << ecm_roll.pose.position.x << '\t' << ecm_roll.pose.position.y << '\t' << ecm_roll.pose.position.z<< '\n';
  // std_msgs::Float64 msg2;
  // msg2.data=ecm_roll.pose.position.x;
  // plot_x.publish(msg2);
  // msg2.data=ecm_roll.pose.position.y;
  // plot_y.publish(msg2);
  // msg2.data=ecm_roll.pose.position.z;
  // plot_z.publish(msg2);
  // msg2.data=0;
  // ecmPub[0].publish(msg2);
  // ecmPub[3].publish(msg2);
  //
  // static double pitch_angle;
  // static double direction;
  // msg2.data=pitch_angle;
  // if (pitch_angle>0.7)
  //   direction=1;
  // if (pitch_angle<-0.2)
  //   direction=0;
  // if (direction==0)
  //   pitch_angle=pitch_angle+0.001;
  // else
  //   pitch_angle=pitch_angle-0.001;
  // ecmPub[1].publish(msg2);
  // msg2.data=0;
  // ecmPub[2].publish(msg2);
  // msg2.data=0;

  PublishCartStates();
  PublishECMStates();
  PublishPSM1States();
  PublishPSM2States();
  PublishPSM3States();

}

void dvrk_gazebo_control::PublishECMStates()
{
  std::vector<std_msgs::Float64> msg;
  msg.resize(4);
  msg[0].data=0.0;
  msg[1].data=0.0;
  msg[2].data=0.0;
  msg[3].data=0.0;

  ecmPub[0].publish(msg[0]);
  ecmPub[1].publish(msg[1]);
  ecmPub[2].publish(msg[2]);
  ecmPub[3].publish(msg[3]);

}

void dvrk_gazebo_control::PublishPSM1States()
{
  std::vector<std_msgs::Float64> msg;
  msg.resize(5);
  msg[0].data=0.0;
  msg[1].data=0.0;
  msg[2].data=0.0;
  msg[3].data=0.0;
  msg[4].data=0.0;

  psm1Pub[0].publish(msg[0]);
  psm1Pub[1].publish(msg[1]);
  psm1Pub[2].publish(msg[2]);
  psm1Pub[3].publish(msg[3]);
  psm1Pub[4].publish(msg[4]);

}
void dvrk_gazebo_control::PublishPSM2States()
{
  std::vector<std_msgs::Float64> msg;
  msg.resize(5);
  msg[0].data=0.0;
  msg[1].data=0.0;
  msg[2].data=0.0;
  msg[3].data=0.0;
  msg[4].data=0.0;

  psm2Pub[0].publish(msg[0]);
  psm2Pub[1].publish(msg[1]);
  psm2Pub[2].publish(msg[2]);
  psm2Pub[3].publish(msg[3]);
  psm2Pub[4].publish(msg[4]);

}

void dvrk_gazebo_control::PublishPSM3States()
{
  std::vector<std_msgs::Float64> msg;
  msg.resize(5);
  msg[0].data=0.0;
  msg[1].data=0.0;
  msg[2].data=0.0;
  msg[3].data=0.0;
  msg[4].data=0.0;

  psm3Pub[0].publish(msg[0]);
  psm3Pub[1].publish(msg[1]);
  psm3Pub[2].publish(msg[2]);
  psm3Pub[3].publish(msg[3]);
  psm3Pub[4].publish(msg[4]);

}

void dvrk_gazebo_control::PublishCartStates()
{
  std::vector<std_msgs::Float64> msg;
  msg.resize(19);

  msg[0].data=0.0;
  msg[1].data=0.0;
  msg[2].data=0.0;
  msg[3].data=0.0;
  msg[4].data=0.0;

  msg[5].data=0.0;
  msg[6].data=0.0;
  msg[7].data=0.0;
  msg[8].data=0.0;
  msg[9].data=0.0;

  msg[10].data=0.0;
  msg[11].data=0.0;
  msg[12].data=0.0;
  msg[13].data=0.0;
  msg[14].data=0.0;

  msg[15].data=0.2;
  msg[16].data=0.0;
  msg[17].data=0.0;
  msg[18].data=0.0;

  for (int i=1;i<4;i++)
  {
    for (int j=0;j<5;j++)
    {
      cartPub[5*(i-1)+j].publish(msg[5*(i-1)+j]);
    }
  }
  for (int j=0;j<4;j++)
  {
    cartPub[15+j].publish(msg[15+j]);
  }
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "dvrk_gazebo_control_node");
  ros::NodeHandle n;
 
  std_msgs::String mes;
  std::stringstream ss;
  ss<<"it starteed";
  mes.data = ss.str();
  ROS_INFO("%s", mes.data.c_str());


  dvrk_gazebo_control obj(n);
  // int i, j;
  int count = 0;

  std_msgs::Float64 msg;
  // char link[100];
  ros::Duration(0.1).sleep();
  ros::Rate loop_rate(10);
  ROS_INFO("%s", mes.data.c_str());

  while(count<1500&&ros::ok())
  {
  obj.PublishCartStates();
  obj.PublishECMStates();
  obj.PublishPSM1States();
  obj.PublishPSM2States();
  obj.PublishPSM3States();
  loop_rate.sleep();
  //ROS_INFO("%s", mes.data.c_str());
  count=count+1;
  }

/*  while (ros::ok())
  {
    obj.PublishCartStates();
  obj.PublishECMStates();
  obj.PublishPSM1States();
  obj.PublishPSM2States();
  obj.PublishPSM3States();
  count=count+1;
  ROS_INFO("%s", mes.data.c_str());
   // ros::spin();
  }*/
}
