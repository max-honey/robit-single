#include "movectrl_common.h"
#include <cmath>
#include <algorithm>
using namespace std;

namespace movectrl
{
  MoveCtrl::MoveCtrl() :
  moveSpeed(DEFAULT_MOVE_SPEED),
  rotateSpeed(DEFAULT_ROT_SPEED),
  dribbleSpeed(DEFAULT_DRIBBLE_SPEED),
  mtl(0),
  mtr(0),
  mtb(0),
  targetX(0),
  targetY(0),
  targetAng(0),
  angSpecified(false),
  circleRadius(0),
  transformTime(0),
  state(STATE::Stop),
  PIDCtrl(
  DEFAULT_KP,
  DEFAULT_KI,
  DEFAULT_KD,
  DEFAULT_DEADZONE,
  DEFAULT_INT_STEP,
  DEFAULT_INT_SAT
  )
  {
    lastX = 0;
    lastY = 0;
    velocityX = 0;
    velocityY = 0;
  }

  int MoveCtrl::DiffAngle(int baseAng, int targetAng){
    int result;

    if(labs(baseAng) >= 360)
      baseAng %= 360;
    if(labs(targetAng) >= 360)
      targetAng %= 360;

    result = targetAng - baseAng;
    if(result > 180){
      result -= 360;
    } else if(result < -180){
      result += 360;
    }

    return result;
  }

  int MoveCtrl::GetVectorDir(int X, int Y){
    return round(atan2f(Y, X) * ARC_TO_ANGLE);
  }

  float MoveCtrl::Distance(int X, int Y){
    return sqrtf(X * X + Y * Y);
  }

  float MoveCtrl::CalcAngleSpeed(float radius, float speed){
    return speed / radius * MACHINE_RADIUS;
  }

  float MoveCtrl::CalcLineSpeed(float radius, float speed){
    return speed / MACHINE_RADIUS * radius;
  }

  void MoveCtrl::RestrictMotorSpeed(){
    int maxval = max(mtl, max(mtr, mtb));
    if(maxval > MAX_MOTOR_SPEED){
      mtl = (int)round(1.0 * mtl / maxval * MAX_MOTOR_SPEED);
      mtr = (int)round(1.0 * mtr / maxval * MAX_MOTOR_SPEED);
      mtb = (int)round(1.0 * mtb / maxval * MAX_MOTOR_SPEED);
    }
  }

  void MoveCtrl::CalcMotorBySpeed(int angle, int speed, int rotate){
    //这里的理解是：L是cos(x+150)，B是cos(x-90)，R是cos(x+30)
    mtl = rotate - (int)round(sin(DiffAngle(-angle, -60) * ANGLE_TO_ARC) * speed);
    mtr = rotate - (int)round(sin(DiffAngle(-angle, 60) * ANGLE_TO_ARC) * speed);
    mtb = rotate - (int)round(sin(DiffAngle(-angle, 180) * ANGLE_TO_ARC) * speed);
  }

  void MoveCtrl::OutputMotorArg(int* mtl, int* mtr, int* mtb, int* zhuan){
    *mtl = this->mtl;
    *mtr = this->mtr;
    *mtb = this->mtb;
    *zhuan = 0;
  }

  void MoveCtrl::DirectTo(int X, int Y){
    targetX = X;
    targetY = Y;
    angSpecified = false;
    state = STATE::Direct;
    PIDCtrl.Clear();
  }

  void MoveCtrl::DirectToWithAng(int X, int Y, int angle){
    targetX = X;
    targetY = Y;
    angSpecified = true;
    targetAng = angle;
    state = STATE::Direct;
    PIDCtrl.Clear();
  }

  bool MoveCtrl::Run(int selfX, int selfY, int selfAng){
    //当前帧移速，角速度
    int curSpd, curRot;
    //平移速度所在角度
    int moveAng;
    //旋转是否到达的标记
    bool rotFlag;
    //返回值
    bool result = false;

    switch(state){
    case STATE::Stop:
      //清零电机
      mtl = mtr = mtb = 0;
      result = false;
      break;
    case STATE::Direct:
      moveAng = GetVectorDir(targetX - selfX, targetY - selfY);
      //如果角度不指定，将角度调整到目标点的方向
      if(!angSpecified){
        targetAng = moveAng;
      }
      //计算旋转
      curRot = DiffAngle(targetAng, selfAng);
      if(labs(curRot) < ANGLE_TOLERANCE_ROT){
        rotFlag = true;
      } else{
        rotFlag = false;
      }

      curRot = (int)round(PIDCtrl.Run(curRot));
      if(labs(curRot) > rotateSpeed){
        curRot = curRot / labs(curRot) * rotateSpeed;
      }
      //计算平移
      moveAng = DiffAngle(moveAng, selfAng);
      if(Distance(targetX - selfX, targetY - selfY) < DIST_TOLERANCE){
        curSpd = 0;
      } else{
        curSpd = moveSpeed;
      }
      //控制电机
      CalcMotorBySpeed(- moveAng, curSpd, curRot);
      //判断是否到达
      if(curSpd == 0 && rotFlag){
        state = STATE::Stop;
        result = true;
      } else{
        if(curSpd == 0 && !angSpecified){
          state = STATE::Stop;
          result = true;
        }else{
          result = false;
        }
      }
      break;
    case STATE::Track:
      //计算角度
      targetAng = GetVectorDir(targetX - selfX, targetY - selfY);
      moveAng = DiffAngle(targetAng, selfAng);
      curRot = (int)round(PIDCtrl.Run(moveAng));
      if(labs(curRot) > rotateSpeed){
        curRot = curRot / labs(curRot) * rotateSpeed;
      }
      //控制电机
      CalcMotorBySpeed(0, moveSpeed, curRot);
      result = false;
      break;
    case STATE::Rush:
      CalcMotorBySpeed(0, moveSpeed, 0);
      result = false;
      break;
    case STATE::March:
      //计算角度
      targetAng = GetVectorDir(targetX - selfX, targetY - selfY);
      //计算旋转
      curRot = DiffAngle(targetAng, selfAng);
      if(labs(curRot) >= ANGLE_TOLERANCE && transformTime <= 0){
        state = STATE::Rotate;
        PIDCtrl.Clear();
      }
      curRot = curRot * 800 / 5;
      curRot = min(1000, max(- 1000, curRot));
      //减少转换时间
      if(transformTime > 0)
        transformTime --;
      //计算平移
      if(Distance(targetX - selfX, targetY - selfY) < DIST_TOLERANCE){
        state = STATE::Stop;
      }
      if(state == STATE::March){
        CalcMotorBySpeed(0, moveSpeed, curRot);
        result = false;
      } else{
        CalcMotorBySpeed(0, 0, 0);
        result = true;
      }
      break;
    case STATE::Rotate:
      if(angSpecified){
        //如果角度指定，调整方向并停止
        curRot = DiffAngle(targetAng, selfAng);
        if(labs(curRot) < ANGLE_TOLERANCE_ROT){
          state = STATE::Stop;
        }
      }else{
        //如果角度不指定，调整方向并前进
        moveAng = GetVectorDir(targetX - selfX, targetY - selfY);
        curRot = DiffAngle(moveAng, selfAng);
        if(labs(curRot) < ANGLE_TOLERANCE_ROT){
          state = STATE::March;
          transformTime = STATE_TRANSFORM_TIME;
        }
      }
      //PID计算角速度
      curRot = (int)round(PIDCtrl.Run(curRot));
      if(labs(curRot) > dribbleSpeed){
        curRot = curRot / labs(curRot) * dribbleSpeed;
      }
      //计算线速度
      curSpd = (int)round(CalcLineSpeed(circleRadius, curRot));

      if(state == STATE::Rotate){
        CalcMotorBySpeed(90, curSpd, curRot);
        result = false;
      } else{
        CalcMotorBySpeed(0, 0, 0);
        result = true;
      }
      break;
    case STATE::Circle:
      curRot = (int)round(CalcAngleSpeed(circleRadius, moveSpeed));
      //确保逆时针
      CalcMotorBySpeed(0, moveSpeed, - curRot);
      result = false;
      break;
    case STATE::Circle2:
      curRot = (int)round(CalcAngleSpeed(circleRadius, moveSpeed));
      //确保逆时针
      CalcMotorBySpeed(- 90, moveSpeed, - curRot);
      result = false;
      break;
    }
    RestrictMotorSpeed();
    return result;
  }

  void MoveCtrl::DoRush(){
    state = STATE::Rush;
  }

  void MoveCtrl::MarchTo(int X, int Y){
    state = STATE::March;
    targetX = X;
    targetY = Y;
    angSpecified = false;
    transformTime = STATE_TRANSFORM_TIME;
    PIDCtrl.Clear();
  }

  void MoveCtrl::DoRotate(int angle){
    state = STATE::Rotate;
    angSpecified = true;
    targetAng = angle;
    PIDCtrl.Clear();
  }

  void MoveCtrl::DoCircle(int radius){
    state = STATE::Circle;
    circleRadius = radius;
  }

  void MoveCtrl::DoCircle2(int radius){
    state = STATE::Circle2;
    circleRadius = radius;
  }

  void MoveCtrl::DoStop(){
    state = STATE::Stop;
  }

  void MoveCtrl::StartTrack(){
    state = STATE::Track;
    PIDCtrl.Clear();
  }

  void MoveCtrl::KeepTrack(int X, int Y){
    targetX = X;
    targetY = Y;
  }

  //////////////////////////////////////////////////////////////////////////

  PIDControler::PIDControler(float Kp, float Ki, float Kd, float DeadZone, float IntegralPeriodLimit, float IntegralSaturate) :
  kp(Kp),
  ki(Ki),
  kd(Kd),
  deadZone(DeadZone),
  intStep(IntegralPeriodLimit),
  intMaxSum(IntegralSaturate),
  intSum(0)
  {
    for(int i = 0; i < 20; i ++)
      targetHistory[i] = 0;
    diffHistory = 0;
  }

  void PIDControler::Clear(){
    intSum = 0;
    for(int i = 0; i < 20; i ++)
      targetHistory[i] = 0;
    diffHistory = 0;
  }

  void PIDControler::UpdateHistory(int Target, int diff){
    diffHistory = diff;
    for(int i = 1; i < 20; i ++)
      targetHistory[i] = targetHistory[i - 1];
    targetHistory[0] = Target;
  }

  //此函数因原理不明被弃用
  float PIDControler::CalcDifferent(int Target){
    //低通滤波器权值，这里用的原来的
    static const int LowPass[20] = {
      10, 8, 6, 5, 4, 3, 2, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0
    };
    float result = LowPass[0] * MoveCtrl::DiffAngle(Target, targetHistory[0]);
    for(int i = 1; i < 20; i ++){
      //这里的减法和我的预期相反，原因不明
      result += LowPass[i] * MoveCtrl::DiffAngle(targetHistory[i], targetHistory[i - 1]);
    }
    return result;
  }

  float PIDControler::Run(int Target){
    //比例项
    float prop = Target;
    //本次单次积分项
    float inte = prop * ki;
    //微分项
    float diff = MoveCtrl::DiffAngle(targetHistory[0], Target);
    diff = diff * 0.7 + diffHistory * 0.3;

    //限制单次积分步长
    inte = min(intStep, max(-intStep, inte));

    //限制积分项饱和
    inte += intSum;
    inte = min(intMaxSum, max(-intMaxSum, inte));

    //计算结果
    float result = prop * kp + inte + diff * kd;
    result = min(128.0f, max(-128.0f, result));

    //调节死区：线性映射
    //float zt = deadZone + (128.0f - deadZone)*(fabs(result)) / 128.0f;
    //result = (result < 0.0f) ? (- zt) : zt;

    //更新历史记录
    UpdateHistory(Target, diff);
    intSum = inte;

    //原本的策略是，这里返回一个历史累计值
    //没必要强行和原来的一致吧……
    return result / 128.0f * MAX_MOTOR_SPEED;
  }
};
