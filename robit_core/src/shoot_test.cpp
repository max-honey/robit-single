#include <ros/ros.h>
#include "movectrl/movectrl_const.h"
#include <movectrl/MoveCmd.h>
#include <movectrl/SelfPos.h>
#include <image_processor/Location.h>
#include <serial/shootcmd.h>
#include <cmath>

const float PI = 3.1415926535897932384626;

float g_ball_x, g_ball_y;
float g_ball_old_x, g_ball_old_y;
int g_near_time;
float g_shoot = 7000;
float g_interval = 60;
ros::Publisher g_cmd_pub;
ros::Publisher g_pos_pub;
ros::Publisher g_shot_pub;

inline float dist(float dx, float dy)
{
  return sqrt(dx * dx + dy * dy);
}

void ballCallback(const image_processor::Location::ConstPtr& msg)
{
  movectrl::MoveCmd cmd;
  movectrl::SelfPos spos;

  g_ball_x = - msg->distance * cos(msg->theta / 180.0 * PI);
  g_ball_y = - msg->distance * sin(msg->theta / 180.0 * PI);
  if(dist(g_ball_x - g_ball_old_x, g_ball_y - g_ball_old_y) > 50)
  {
    cmd.x = cmd.y = cmd.ang = 0;
    cmd.cmd = movectrl::StartTrack;
    g_cmd_pub.publish(cmd);
  }

  cmd.x = g_ball_x;
  cmd.y = g_ball_y;
  cmd.ang = 0;
  cmd.cmd = movectrl::KeepTrack;
  g_cmd_pub.publish(cmd);

  g_ball_old_x = g_ball_x;
  g_ball_old_y = g_ball_y;

  spos.x = 0;
  spos.y = 0;
  spos.ang = 0;
  g_pos_pub.publish(spos);

  if(msg->distance < 70)
  {
    g_near_time ++;
  }
  else if(msg->distance > 90)
  {
    g_near_time = 0;
  }
  if(g_near_time >= g_interval)
  {
    g_near_time = 0;
    serial::shootcmd stcmd;
    stcmd.power = g_shoot;
    g_shot_pub.publish(stcmd);
    ROS_INFO("Shoot!!");
  }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "catch_ball");
  ros::NodeHandle handle;

  if(ros::param::has("~shootpower"))
  {
    ros::param::get("~shootpower", g_shoot);
  }
  if(ros::param::has("~interval"))
  {
    ros::param::get("~interval", g_interval);
  }

  g_cmd_pub = handle.advertise<movectrl::MoveCmd>("move_cmd", 10);
  g_pos_pub = handle.advertise<movectrl::SelfPos>("set_self_pos", 10);
  g_shot_pub = handle.advertise<serial::shootcmd>("shootcmd", 10);
  ros::Subscriber ball_sub = handle.subscribe("location", 5, ballCallback);

  ros::spin();
  return 0;
}

