#include "ros/ros.h"
#include <serial/speedcmd.h>
#include <serial/shootcmd.h>
#include <serial/feedback.h>
using namespace serial;

int main(int argc, char** argv)
{
  ros::init(argc,argv,"SerialTest");
  ros::NodeHandle handle;
  ros::Publisher pub = handle.advertise<speedcmd>("speedcontroller", 10);
  ros::Rate loop_rate(10);
  speedcmd msg;

  msg.speed.push_back(0x0211);
  msg.speed.push_back(0x0211);
  msg.speed.push_back(0x0211);
  msg.speed.push_back(0x0211);
  msg.speed.push_back(0x0211);

  while(ros::ok())
  {
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
