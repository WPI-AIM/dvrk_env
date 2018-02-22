#include "RosBridge.h"

int main(int argc, char**argv)
{

  trial trial_obj;
  double LJ[7]={1,0,0,0,0,1,0}, RJ[7]={1,0,0,0,0,0,0};
  RosBridge<trial> Bridge(&trial_obj);
  double L[4][4];
  double R[4][4];
  Bridge.init();
  int iter=0;

while (ros::ok())
{

  iter=iter+1;
  for (int i=0; i<4; i++)
  {
    for (int j=0;j<4;j++)
    {
      if (i==j)
      {
        L[i][j]=1;
        R[i][j]=1;
      }
      else
      {
        L[i][j]=0;
        R[i][j]=0;
      }
    }
  }
  L[0][3]=3;
  L[1][3]=2;

  trial_obj.setTransforms(L,R);
  trial_obj.setJoints(LJ, RJ);
  // trial_obj.print();
  // trial_obj.getRosPoses(left,right);
  Bridge.setFuncPtr(&trial::getRosPoses);
  Bridge.setPoses();
  Bridge.setFuncPtr2(&trial::getRosJoints);
  Bridge.setJoints();
  // Bridge.print();

  Bridge.publishPoses();
  Bridge.publishJoints();
  // boost::function <void(double[4][4] , double[4][4])> f(boost::bind( &Laprotek_ROS_Bridge::getPoses, trial_obj ,_1,_2 )) ;
}
}
