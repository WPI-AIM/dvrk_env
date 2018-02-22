// #include "trial.h"
#include "server.h"


template<typename C>
class RosBridge{
  typedef void (C::*my_func_ptr)( geometry_msgs::PoseStamped&,  geometry_msgs::PoseStamped&);
  typedef void (C::*my_func_ptr2)( sensor_msgs::JointState&);

private:
   geometry_msgs::PoseStamped leftPose;
   geometry_msgs::PoseStamped rightPose;
   sensor_msgs::JointState Joints;
   my_func_ptr func;
   my_func_ptr2 func2;
   C *obj;

  ros::Publisher leftHandlePosePub;
  ros::Publisher rightHandlePosePub;
  ros::Publisher HandleJointsPub;

public:
  RosBridge(C *class_obj){obj=class_obj;}
  void setFuncPtr(my_func_ptr function);
  void setFuncPtr2(my_func_ptr2 function);
  // void setObject(C obj);
  void init();
  void setPoses();
  void setJoints();
  void print();
  void publishPoses();
  void publishJoints();
};

template<typename C>void RosBridge<C>::setFuncPtr(my_func_ptr function)
{
  func=function;
}

template<typename C>void RosBridge<C>::setFuncPtr2(my_func_ptr2 function)
{
  func2=function;
}

template<typename C>void RosBridge<C>::setPoses()
{
  (obj->*func)(leftPose, rightPose);
}

template<typename C>void RosBridge<C>::setJoints()
{
  (obj->*func2)(Joints);
}

template<typename C>void RosBridge<C>::print()
{
  std::cout << Joints.position[0] << '\t' << Joints.position[1] << '\t' << Joints.position[2] << '\t' << Joints.position[3] << '\t' << Joints.position[4] << '\t' << Joints.position[5] << '\t' << Joints.position[6] << '\t' << '\n';
  std::cout << Joints.position[7] << '\t' << Joints.position[8] << '\t' << Joints.position[9] << '\t' << Joints.position[10] << '\t' << Joints.position[11] << '\t' << Joints.position[12] << '\t' << Joints.position[13] << '\t' << '\n';
}

template<typename C>void RosBridge<C>::init()
{
  int argc;
  char** argv;
  // ros::M_string s;
  ros::init(argc, argv, "laprotek_comm_node");
  ros::NodeHandle n;
  leftHandlePosePub = n.advertise<geometry_msgs::PoseStamped>("/Laprotek/LeftHandle/Pose",1000);
  rightHandlePosePub = n.advertise<geometry_msgs::PoseStamped>("/Laprotek/RightHandle/Pose",1000);
  HandleJointsPub = n.advertise<sensor_msgs::JointState>("/joint_states",1000);
}

template<typename C>void RosBridge<C>::publishPoses()
{
  leftHandlePosePub.publish(leftPose);
  rightHandlePosePub.publish(rightPose);
}

template<typename C>void RosBridge<C>::publishJoints()
{
  HandleJointsPub.publish(Joints);

}
