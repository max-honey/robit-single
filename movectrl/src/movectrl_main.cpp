#include "movectrl_common.h"
#include <ros/ros.h>
#include <serial/speedcmd.h>
#include <movectrl/MoveCmd.h>
#include <movectrl/GetState.h>
#include <movectrl/SelfPos.h>
#include <movectrl/SetBallCatcher.h>
#include <movectrl/SetSpeed.h>
using namespace movectrl;

MoveCtrl g_movectrl;
unsigned long g_ball_catcher = 0;
ros::Publisher g_cmd_publisher;

namespace movectrl
{
  const char* stateMsgStr[] =
  {
    "Stop",
    "Direct",
    "Rotate",
    "March",
    "Rush",
    "Track",
    "Circle",
    "Circle2"
  };
}

void cmdCallback(const MoveCmd::ConstPtr& msg)
{
  switch(msg->cmd)
  {
    case DoStop:
      g_movectrl.DoStop();
      break;
    case DirectTo:
      g_movectrl.DirectTo(round(msg->x), round(msg->y));
      break;
    case DirectToWithAng:
      g_movectrl.DirectToWithAng(round(msg->x), round(msg->y), round(msg->ang));
      break;
    case MarchTo:
      g_movectrl.MarchTo(round(msg->x), round(msg->y));
      break;
    case DoRotate:
      g_movectrl.DoRotate(round(msg->ang));
      break;
    case DoRush:
      g_movectrl.DoRush();
      break;
    case DoCircle:
      g_movectrl.DoCircle(round(msg->ang));
      break;
    case DoCircle2:
      g_movectrl.DoCircle2(round(msg->ang));
      break;
    case StartTrack:
      g_movectrl.StartTrack();
      break;
    case KeepTrack:
      g_movectrl.KeepTrack(round(msg->x), round(msg->y));
      break;
  }
}

bool getStateCallback(GetState::Request &req, GetState::Response &res)
{
  res.state.state = g_movectrl.GetState();
  res.state.state_name = g_movectrl.GetStateStr();
  return true;
}

bool setBallCatcherCallback(SetBallCatcher::Request &req, SetBallCatcher::Response &res)
{
  g_ball_catcher = req.speed;
  return true;
}

bool setSpeedCallback(SetSpeed::Request &req, SetSpeed::Response &res)
{
  if(req.move > 0)
    g_movectrl.moveSpeed = req.move;
  if(req.rotate > 0)
    g_movectrl.rotateSpeed = req.rotate;
  if(req.dribble > 0)
    g_movectrl.dribbleSpeed = req.dribble;
  return true;
}

void selfPosCallback(const SelfPos::ConstPtr &msg)
{
  int mtl, mtr, mtb, zhuan;
  serial::speedcmd cmd;

  g_movectrl.Run(round(msg->x), round(msg->y), round(msg->ang));
  g_movectrl.OutputMotorArg(&mtl, &mtr, &mtb, &zhuan);

  cmd.speed.push_back(mtl);
  cmd.speed.push_back(mtr);
  cmd.speed.push_back(mtb);
  cmd.speed.push_back(g_ball_catcher);
  cmd.speed.push_back(- g_ball_catcher);

  g_cmd_publisher.publish(cmd);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "movectrl");
  ros::NodeHandle handle;

  g_cmd_publisher = handle.advertise<serial::speedcmd>("speedcontroller", 10);

  ros::Subscriber cmd_sub = handle.subscribe("move_cmd", 100, cmdCallback);
  ros::Subscriber pos_sub = handle.subscribe("set_self_pos", 4, selfPosCallback);

  ros::ServiceServer get_state_srv = handle.advertiseService("get_state", getStateCallback);
  ros::ServiceServer set_bc_srv = handle.advertiseService("set_ball_catcher", setBallCatcherCallback);
  ros::ServiceServer set_speed_srv = handle.advertiseService("set_speed", setSpeedCallback);

  ros::spin();

  return 0;
}

