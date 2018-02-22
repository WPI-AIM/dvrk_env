#include "dvrk_gazebo_control_plugin.h"

namespace dvrk_plugins
{
void dvrkGazeboControlPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  parent_model=_model;
  sdf=_sdf;
  gazebo::physics::JointPtr joint;

  model_nh_ = ros::NodeHandle(parent_model->GetName());
  num_joints=parent_model->GetJoints().size();
  sub_position.resize(num_joints);
  sub_positionTarget.resize(num_joints);
  sub_Force.resize(num_joints);

  //Initializing clock subscriber to continually publish states
  sub_clock = model_nh_.subscribe<rosgraph_msgs::Clock>("/clock",1,&dvrkGazeboControlPlugin::clock_cb, this);

  //Initializing the publisher topic
  pub_states = model_nh_.advertise<sensor_msgs::JointState>("joint/states", 1000);

  for (int i=0;i<num_joints; i++)
  {
    joint=parent_model->GetJoints()[i];   //Get joint pointer handle

    std::string joint_name;
    getJointStrings(joint, joint_name);   //Get joint name which is compatible with ROS

    // Checking if the joint is a fixed joint (16448 is decimal for 4040 in hexadecimal which enum for fixed joint)
    if (joint->GetType()==16448)
      continue;

    //Checking if the joint is the active pitch joint of the kinematic chain for PSM
    if (joint_name.find("PSM")!=std::string::npos)
    {
      if ((joint_name.find("outer_pitch_joint")!=std::string::npos) && joint_name.find("pitch_joint_1")==std::string::npos)
        continue;
    }
    //Checking if the joint is the active pitch joint of the kinematic chain for ecm
    else if (joint_name.find("ecm")!=std::string::npos)
    {
      if ((joint_name.find("pitch_")!=std::string::npos) && joint_name.find("pitch_front")==std::string::npos)
      {
        continue;
      }
    }

    //Biniding subscriber callback functions for additional arguments to be passed in the functions
    boost::function<void (const std_msgs::Float64Ptr)>PositionFunc(boost::bind(&dvrkGazeboControlPlugin::SetPosition,this, _1,joint));
    boost::function<void (const std_msgs::Float64Ptr)>PositionTargetFunc(boost::bind(&dvrkGazeboControlPlugin::SetPositionTarget,this, _1,joint));
    boost::function<void (const std_msgs::Float64Ptr)>ForceFunc(boost::bind(&dvrkGazeboControlPlugin::SetForce,this, _1,joint));

    //Initializing the subscriber topic for setting Position, Position Target or Effort
    sub_position[i] = model_nh_.subscribe<std_msgs::Float64>("/"+joint_name+"/SetPosition",1,PositionFunc);
    sub_positionTarget[i] = model_nh_.subscribe<std_msgs::Float64>("/"+joint_name+"/SetPositionTarget",1,PositionTargetFunc);
    sub_Force[i] = model_nh_.subscribe<std_msgs::Float64>("/"+joint_name+"/SetEffort",1,ForceFunc);

  }
  this->PublishStates(); //Publish the read values from Gazebo to ROS
}

//callback function for clock.
void dvrkGazeboControlPlugin::clock_cb(const rosgraph_msgs::Clock msg)
{
  //Always publishes the state of all joints as long as the Gazebo simulation is running
  PublishStates();
}

//callback function to set position of the joint
void dvrkGazeboControlPlugin::SetPosition(const std_msgs::Float64Ptr& msg, gazebo::physics::JointPtr joint)
{
  joint->SetPosition(0,msg->data);
}

//callback function to set target position and use the pid controller in Gazebo. pid values are specified through the rosparam server
void dvrkGazeboControlPlugin::SetPositionTarget(const std_msgs::Float64Ptr& msg, gazebo::physics::JointPtr joint)
{
  joint_class joint_obj(joint);
  double p, i, d;
  joint_obj.getPID(p, i, d);

  if (p<0|| i<0 || d<0)
  {
    ROS_FATAL_STREAM("PID controller could not be initialized. Check if the parameters have been loaded properly on the ROS parameter server." << '\n');
  }
  gazebo::common::PID pid;
  pid=gazebo::common::PID(p,i,d);
  parent_model->GetJointController()->SetPositionPID(joint->GetScopedName(), pid);
  parent_model->GetJointController()->SetPositionTarget(joint->GetScopedName(), msg->data);

}

//callback function to set effort (torque for revolute, force for prismatic) of the joint
void dvrkGazeboControlPlugin::SetForce(const std_msgs::Float64Ptr& msg, gazebo::physics::JointPtr joint)
{
  joint->SetForce(0,msg->data);
}

//publish all joint state values for position, velocity and external effort applied at the joint
void dvrkGazeboControlPlugin::PublishStates()
{
  sensor_msgs::JointState msg;
  gazebo::physics::JointPtr joint;
  for (int n=0; n<num_joints; n++)
  {
    joint=parent_model->GetJoints()[n];
    std::string joint_namespace, joint_name;
    getJointStrings(joint, joint_name);
    if (joint->GetType()==16448)
      continue;

    if (joint_name.find("PSM")!=std::string::npos)
    {
      if ((joint_name.find("outer_pitch_joint")!=std::string::npos) && joint_name.find("pitch_joint_1")==std::string::npos)
        continue;
    }
    else if (joint_name.find("ecm")!=std::string::npos)
    {
      if ((joint_name.find("pitch_")!=std::string::npos) && joint_name.find("pitch_front")==std::string::npos)
      {
        continue;
      }
    }

    msg.header.stamp=ros::Time::now();
    msg.name.push_back(joint_name);
    msg.position.push_back(joint->GetAngle(0).Radian());
    msg.velocity.push_back(joint->GetVelocity(0));
    msg.effort.push_back(joint->GetForce(0));

  }
  pub_states.publish(msg);
}

//Convert the joint names from Gazebo to something which is compatible with ROS
void dvrkGazeboControlPlugin::getJointStrings(gazebo::physics::JointPtr jointPtr, std::string &str1)
{
  std::string joint_name_scoped=jointPtr->GetScopedName();
  size_t rep_pos = 0;
  while ((rep_pos = joint_name_scoped.find("::", rep_pos)) != std::string::npos) {
         joint_name_scoped.replace(rep_pos, 2, "/");
         rep_pos += 1;
    }
    str1=joint_name_scoped;
}

//Setting private variables of the joint class
void joint_class::setJointStrings()
{
  std::string joint_name_scoped=jointPtr->GetScopedName();
  size_t rep_pos = 0;
  while ((rep_pos = joint_name_scoped.find("::", rep_pos)) != std::string::npos) {
         joint_name_scoped.replace(rep_pos, 2, "/");
         rep_pos += 1;
    }
  joint_name=joint_name_scoped;
}

//Read ROS param servers to  get pid values for the postion controller for the particular joint. If no such values are available, they are set to -1.
void joint_class::setController()
{
  double p_def=-1, i_def=-1, d_def=-1;
  model_nh_.param(std::string("/"+joint_name+"_controller/p"), p, p_def);
  model_nh_.param(std::string("/"+joint_name+"_controller/i"), i, i_def);
  model_nh_.param(std::string("/"+joint_name+"_controller/d"), d, d_def);
}

//Function to access pid values from the private variables of joint class
void joint_class::getPID(double& K_p, double& K_i, double& K_d)
{
  K_p=p;
  K_d=d;
  K_i=i;
}

}
