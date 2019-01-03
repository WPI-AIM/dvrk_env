#include "control_plugin.h"

namespace dvrk_gazebo
{
void ControlPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  parent_model=_model;
  // std::cout << parent_model << '\n';
  sdf=_sdf;
  gazebo::physics::JointPtr joint;
  gazebo::physics::LinkPtr link;

  model_nh_ = ros::NodeHandle(parent_model->GetName());
  num_joints=parent_model->GetJoints().size();
  sub_position.resize(num_joints);
  sub_positionTarget.resize(num_joints);
  sub_Force.resize(num_joints);
  updateConnection.resize(num_joints);

  //Initializing the publisher topic
  pub_states = model_nh_.advertise<sensor_msgs::JointState>("joint/states", 1000);


  for (int i=0;i<num_joints; i++)
  {
    joint=parent_model->GetJoints()[i];   //Get joint pointer handle
    joint_class* joint_obj= new joint_class(joint, parent_model);


    std::string joint_name;

    joint_obj->name(joint_name); //Get joint name which is compatible with ROS

    // Checking if the joint is a fixed joint (16448 is decimal for 4040 in hexadecimal which enum for fixed joint)
    if (joint->GetType()==16448)
      continue;

    //Checking if the joint is the active pitch joint of the kinematic chain for PSM
    if (joint_name.find("psm")!=std::string::npos)
    {
      if ((joint_name.find("pitch_")!=std::string::npos) && joint_name.find("pitch_back")==std::string::npos && joint_name.find("tool_pitch_")==std::string::npos)
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

    joint_obj->setDefaultMode();

    //Biniding subscriber callback functions for additional arguments to be passed in the functions
    boost::function<void (const std_msgs::Float64Ptr)>PositionFunc(boost::bind(&joint_class::SetPosition,joint_obj, _1));
    boost::function<void (const std_msgs::Float64Ptr)>PositionTargetFunc(boost::bind(&joint_class::SetPositionTarget,joint_obj, _1));
    boost::function<void (const std_msgs::Float64Ptr)>ForceFunc(boost::bind(&ControlPlugin::SetForce,this, _1,joint));

    //Initializing the subscriber topic for setting Position, Position Target or Effort
    sub_position[i] = model_nh_.subscribe<std_msgs::Float64>("/"+joint_name+"/SetPosition",1,PositionFunc);
    sub_positionTarget[i] = model_nh_.subscribe<std_msgs::Float64>("/"+joint_name+"/SetPositionTarget",1,PositionTargetFunc);
    sub_Force[i] = model_nh_.subscribe<std_msgs::Float64>("/"+joint_name+"/SetEffort",1,ForceFunc);

    // joint_obj.setPositionTarget();
    this->updateConnection[i] = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&joint_class::update, joint_obj));
    this->updateStates = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&ControlPlugin::PublishStates,this));
  }
  // this->PublishStates(); //Publish the read values from Gazebo to ROS
}

void joint_class::update()
{
  if (mode==1)
  {
    if (dynamic_init==0)
      setPositionTarget();
    dynamic_init=1;
  }
  else
  {
    setPosition();
  }
}

//callback function to set position of the joint
void joint_class::SetPosition(const std_msgs::Float64Ptr& msg)
{
  if (isInLoop())
  {
    ROS_ERROR_STREAM(joint_name +" is inside a closed loop chain. Set position using /SetPositionTarget topic. Ignoring incoming message.\n");
    return;
  }
  joint_pos=msg->data;
  mode=0;
}

//callback function to set target position and use the pid controller in Gazebo. pid values are specified through the rosparam server
void joint_class::SetPositionTarget(const std_msgs::Float64Ptr& msg)
{
  joint_pos=msg->data;
  mode=1;
  dynamic_init=0;
  setPositionTarget();
}

//callback function to set effort (torque for revolute, force for prismatic) of the joint
void ControlPlugin::SetForce(const std_msgs::Float64Ptr& msg, gazebo::physics::JointPtr joint)
{
  joint->SetForce(0,msg->data);
}

//publish all joint state values for position, velocity and external effort applied at the joint
void ControlPlugin::PublishStates()
{
  sensor_msgs::JointState msg;
  gazebo::physics::JointPtr joint;
  for (int n=0; n<num_joints; n++)
  {
    joint=parent_model->GetJoints()[n];
    std::string joint_name;
    getJointStrings(joint, joint_name);
    if (joint->GetType()==16448)
      continue;

    if (joint_name.find("psm")!=std::string::npos)
    {
      if ((joint_name.find("outer_pitch_joint")!=std::string::npos) && joint_name.find("pitch_back")==std::string::npos && joint_name.find("tool_pitch_")==std::string::npos)
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
//    msg.position.push_back(joint->GetAngle(0).Radian());
    msg.position.push_back(joint->Position(0));
    msg.velocity.push_back(joint->GetVelocity(0));
    msg.effort.push_back(joint->GetForce(0));

  }
  pub_states.publish(msg);
}

//Convert the joint names from Gazebo to something which is compatible with ROS
void ControlPlugin::getJointStrings(gazebo::physics::JointPtr jointPtr, std::string &str1)
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
  model_nh_.param(std::string("/"+joint_name+"_controller/init_joint"), init_joint, 0.0);
}

//Function to access pid values from the private variables of joint class
void joint_class::getPID(double& K_p, double& K_i, double& K_d)
{
  K_p=p;
  K_d=d;
  K_i=i;
}

void joint_class::setPosition()
{
  jointPtr->SetPosition(0,joint_pos);
  // std::cout <<  << '\n';
}

void joint_class::setPositionTarget()
{
  double p, i, d;
  getPID(p, i, d);

  if (p<0|| i<0 || d<0)
  {
    ROS_FATAL_STREAM("PID controller for " + joint_name + " could not be initialized. Check if the parameters have been loaded properly on the ROS parameter server." << '\n');
  }
  gazebo::common::PID pid;
  pid=gazebo::common::PID(p,i,d);

  parent_model->GetJointController()->SetPositionPID(jointPtr->GetScopedName(), pid);

  parent_model->GetJointController()->SetPositionTarget(jointPtr->GetScopedName(), joint_pos);
}

void joint_class::name(std::string &name)
{
    name=joint_name;
}

bool joint_class::isInLoop()
{
  std::vector<gazebo::physics::LinkPtr> a;
  a.push_back(jointPtr->GetChild());
  return !(jointPtr->GetChild()->FindAllConnectedLinksHelper(jointPtr->GetParent(), a, true));
}

void joint_class::setDefaultMode()
{
  if (isInLoop())
    mode=1;
  else
    mode=0;
}
}
