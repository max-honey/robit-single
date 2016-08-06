#include "movectrl/movectrl_const.h"
#include <ros/ros.h>
#include <movectrl/MoveCmd.h>
#include <movectrl/SelfPos.h>
using namespace movectrl;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "movectrl_systest");
  ros::NodeHandle handle;
  ros::Rate loop_rate(20);

  ros::Publisher cmd_pub = handle.advertise<MoveCmd>("move_cmd", 10);
  ros::Publisher pos_pub = handle.advertise<SelfPos>("set_self_pos", 10);

  MoveCmd cmd;
  SelfPos pos;

  pos.x = pos.y = pos.ang = 0.0;
  cmd.x = cmd.y = 0.0;
  cmd.cmd = DoCircle;
  cmd.ang = 100.0;

  while(ros::ok())
  {
    cmd_pub.publish(cmd);
    pos_pub.publish(pos);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

