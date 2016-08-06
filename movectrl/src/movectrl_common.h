#ifndef MOVECTRL_COMMON_H
#define MOVECTRL_COMMON_H

#include "movectrl/movectrl_const.h"

namespace movectrl
{
  //低速电机PID控制器
  //用于调控角度（代码里用了角减……）
  //调整PID参数前请阅读Robit电机控制简要说明
  class PIDControler{
  public:
    //构造一个PID控制器
    //Kp, Ki, Kd          : PID参数
    //DeadZone            : 电机控制器死区电位
    //IntegralPeriodLimit : 积分最大步长
    //IntegralSaturate    : 积分饱和值
    PIDControler(float Kp, float Ki, float Kd, float DeadZone, float IntegralPeriodLimit, float IntegralSaturate);

    //清零历史记录
    void Clear();

    //进行一次控制
    //我们假定输出值的导数和控制值成线性，假定稳态角度为0
    //注意：本函数的返回值没有上界，调用者需自行确定
    //Target : 目标输出值
    //返回值 : 控制值
    float Run(int Target);

  private:
    //比例-积分-微分控制参数
    float kp, ki, kd;

    //电机死区参数
    float deadZone;

    //单次积分值上限
    float intStep;

    //积分饱和上限
    float intMaxSum;

    //积分和
    float intSum;

    //目标历史记录
    //这里因为采用了原来的低通滤波器参数
    //所以写成硬编码
    int targetHistory[20];
    int diffHistory;

    //更新历史记录
    void UpdateHistory(int Target, int diff);

    //计算微分值
    float CalcDifferent(int Target);
  };


  //运动控制类
  class MoveCtrl{
  public:

    //计算角差
    //baseAng  : 基准角
    //targetAng  : 到达角
    //返回值  : 角度差
    static int DiffAngle(int baseAng, int targetAng);

    //计算方位角
    //X, Y    : 向量
    //返回值  : 方位角（-180~180）
    static int GetVectorDir(int X, int Y);

    //计算距离
    //X, Y    : 向量
    //返回值  : 距离
    static float Distance(int X, int Y);

    //计算圆周角速度
    //radius  : 圆周半径
    //speed    : 机器线速度
    //返回值  : 旋转速度rotate的值
    static float CalcAngleSpeed(float radius, float speed);

    //根据角速度计算线速度
    //radius  : 圆周半径
    //rotate  : 机器角电机速度
    //返回值  : 旋转速度rotate的值
    static float CalcLineSpeed(float radius, float speed);

    //输出电机参数
    void OutputMotorArg(int* mtl, int* mtr, int* mtb, int* zhuan);

    //命令：不带球移动到指定点
    void DirectTo(int X, int Y);

    //命令：不带球移动到指定点，同时朝向指定角度
    void DirectToWithAng(int X, int Y, int angle);

    //命令：带球移动到指定点
    void MarchTo(int X, int Y);

    //命令：带球转向指定方向，不移动
    void DoRotate(int angle);

    //命令：带球前冲
    void DoRush();

    //命令：圆周运动测试
    //执行此命令前，请保证机器中心位于圆周上，前方朝着圆周逆时针的切线
    void DoCircle(int radius);

    //命令：圆周运动测试2
    //执行此命令前，请保证机器中心位于圆周上，前方朝着中心
    void DoCircle2(int radius);

    //命令：立刻停止
    void DoStop();

    //命令：开始追踪动作，或转换追踪目标
    void StartTrack();

    //命令：保持追踪，设置目标点位置
    void KeepTrack(int X, int Y);
  
    //前进一帧
    //selfX, selfY  : 自机坐标
    //selfAng    : 自机角度
    //返回值    : 状态是否变化
    bool Run(int selfX, int selfY, int selfAng);

    //平移速度(线)
    int moveSpeed;

    //旋转速度(角电机)
    int rotateSpeed;

    //持球旋转速度(角电机)
    int dribbleSpeed;

    //构造函数
    MoveCtrl();

    STATE GetState() const{
      return state;
    }

    const char* GetStateStr() const{
      switch(state){
      case STATE::Stop:
        return "Stop";
        break;
      case STATE::Direct:
        return "Direct";
        break;
      case STATE::Rotate:
        return "Rotate";
        break;
      case STATE::March:
        return "March";
        break;
      case STATE::Rush:
        return "Rush";
        break;
      case STATE::Track:
        return "Track";
        break;
      default:
        return "Other";
        break;
      }
    };

  private:
    //自己上一帧的位置，速度
    int lastX, lastY, velocityX, velocityY;

    //三电机参数
    int mtl, mtr, mtb;

    //目标点位置
    int targetX, targetY;

    //目标方向
    int targetAng;

    //方向是否被指定
    bool angSpecified;

    //圆周运动半径
    int circleRadius;
  
    //当前决策状态
    STATE state;

    //转换时间
    int transformTime;

    //PID控制器
    PIDControler PIDCtrl;

    //限制电机最大速度
    void RestrictMotorSpeed();

    //根据速度向量计算电机转速
    void CalcMotorBySpeed(int angle, int speed, int rotate);
  };
};

#endif
