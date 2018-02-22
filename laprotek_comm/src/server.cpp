#include "RosBridge.h"

// #define MAX_ITERATION 100000

void siginthandler(int param)
{
  exit(1);
}

int main(int argc, char **argv)
{

  Server server(1500);
  RosBridge<Server> Bridge(&server);
  // char *param;
  // param[0]='R';
  // std::cout << argc << '\t'<< argv[1] << '\n';
  if (argc==2 && !strcmp(argv[1], "ROS"))
  {
    Bridge.init();
    std::cout << "ROS Node started and initialised" << '\n';
  }

  while(1)
  {
    signal(SIGINT, siginthandler);
    server.initSocket();
    if (!server.ConnectToClient())
    {
      return -1;
    }
    while(1)
    {
      server.PackData();
      if (!server.SendData())
        break;
      server.ReceiveData();
      server.UnpackData();
      // server.DebugPrint();
    if (argc==2 && !strcmp(argv[1], "ROS"))
      {
        Bridge.setFuncPtr(&Server::getRosPoses);
        Bridge.setPoses();
        Bridge.setFuncPtr2(&Server::getRosJoints);
        Bridge.setJoints();
        // Bridge.print();

        Bridge.publishPoses();
        Bridge.publishJoints();
      }
    }
    server.closeConnection();
  }
}
