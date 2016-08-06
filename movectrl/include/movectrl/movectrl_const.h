#ifndef MOVECTRL_CONFIG_H
#define MOVECTRL_CONFIG_H

namespace movectrl
{
  //数学常量
  const float PI = 3.1415926535897932384626f;
  const float ANGLE_TO_ARC = PI / 180.0f;
  const float ARC_TO_ANGLE = 180.0f / PI;

  //程序所允许的单电机最大转速
  //机器真正的最大转速参加后面机械常数
  const int MAX_MOTOR_SPEED = 20000;
  //默认平移速度
  const int DEFAULT_MOVE_SPEED = 1200;
  //默认旋转速度
  const int DEFAULT_ROT_SPEED = 600;
  //默认带球速度
  const int DEFAULT_DRIBBLE_SPEED = 700;
  //默认角度允许误差
  const int ANGLE_TOLERANCE = 7;
  //默认角度误差（旋转行为）
  const int ANGLE_TOLERANCE_ROT = 2;
  //最大转换时间
  const int STATE_TRANSFORM_TIME = 30;
  //默认距离允许误差
  const int DIST_TOLERANCE = 10;


  //PID控制参数（原本值随场景变化不可考）
  //纯P调节最优1.2
  //const float DEFAULT_KP = 1.0f;
  const float DEFAULT_KP = 0.6f;
  const float DEFAULT_KI = 0.0f;
  const float DEFAULT_KD = 0.1f;

  //电机死区（原本的就是0）
  const float DEFAULT_DEADZONE = 0.0f;
  //PID积分步长限（原本是2.0）
  const float DEFAULT_INT_STEP = 1.0f;
  //PID积分饱和限（原本是14.0）
  const float DEFAULT_INT_SAT = 14.0f;



  //以下所有常数，单位为厘米、转、秒

  //轮子半径
  const float WHEEL_RADIUS = 5.0;

  //机械半径（机器中心到轮子接地中心的距离）
  const float MACHINE_RADIUS = 22.0;

  //持球半径（理论上应该是机器中心到球心的距离）
  //根据实际需要测试结果，控制带球旋转
  const float BALL_DIST = 60.0;

  //电压转速参数
  //默认值2000代表2000单位电压->1 r/s
  const int ROTATION_SPEED_PARAM = 2000;

  //轮线速度比
  //每产生1cm/s的线速度，需要的单位电压值
  const float VOLTAGE_PER_LINEAR_SPEED = ROTATION_SPEED_PARAM / (2.0f * PI * WHEEL_RADIUS);

  //最大轮转速
  //默认值代表12.5 r/s
  const float MAX_WHEEL_ROTATION_SPEED = 12.5;

  //控制命令
  enum CtrlCmd
  {
    DoStop = 0,
    DirectTo = 1,
    DirectToWithAng = 2,
    MarchTo = 3,
    DoRotate = 4,
    DoRush = 5,
    DoCircle = 6,
    DoCircle2 = 7,
    StartTrack = 9,
    KeepTrack = 10
  };

  //当前决策状态
  enum STATE
  {
    //停止
    //当前状态下，机器不会做任何动作
    Stop = 0,

    //直接
    //当前状态下，机器会一边调整角度一边前进
    //1.目标位置有效，目标角度有效
    //2.目标位置有效，目标角度无效
    Direct = 1,

    //旋转
    //当前状态下，机器以球为中心做旋转，不前进
    //非持球旋转请使用Direct
    //1.目标位置无效，目标角度有效，旋转后停止
    //2.目标位置有效，目标角度无效，旋转后转March
    Rotate = 2,

    //前进
    //当前状态下，机器只能前进，不能旋转
    //如果前进角度和目标位置所在的角度差别太大，机器会自动转到Rotate
    //目标位置有效，目标角度无效
    March = 3,

    //冲刺
    //当前状态下，机器不进行任何计算，向0度角冲锋
    //目标位置无效，目标角度无效
    Rush = 4,

    //追踪
    //追踪每一帧输入的位置
    Track = 5,


    //圆周运动（测试）
    //按直线运动速度以特定半径做圆周运动
    //半径有效
    Circle = 6,
    Circle2 = 7
  };
};

#endif
