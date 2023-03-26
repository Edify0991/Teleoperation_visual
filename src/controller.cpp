#include <unistd.h>
#include <iostream>
#include <vector>
#include <deque>
#include <math.h>
#include <cstdlib>
#include <Eigen/Dense>
#include <config.h>
#include <virtual_fixture.h>
#include <controller.h>
#include <process_robot_data.h>

//笛卡尔空间六自由度导纳控制,输出dX
Eigen::Matrix<Eigen::Matrix<double,2,1>,6,1> Controller::admittance_cart(const Admittance_Args_Cart sysArgs, Eigen::Matrix<double,6,1> x1, Eigen::Matrix< double,6,1> x2, Eigen::Matrix<double,6,1> u)
{
  Eigen::Matrix<Eigen::Matrix<double,2,2>,6,1> A;
  Eigen::Matrix<Eigen::Matrix<double,2,1>,6,1> B;
  Eigen::Matrix<double,2,1> X;
  Eigen::Matrix<Eigen::Matrix<double,2,1>,6,1> dX;

  A << sysArgs.Alx,sysArgs.Aly,sysArgs.Alz,sysArgs.Arx,sysArgs.Ary,sysArgs.Arz;
  B << sysArgs.Blx,sysArgs.Bly,sysArgs.Blz,sysArgs.Brx,sysArgs.Bry,sysArgs.Brz;

  for(int i=0;i<6;i++)
  {
    X <<(Eigen::MatrixXd(2,1) << x1[i], x2[i]).finished();
    dX[i] = A[i]*X + B[i]*u[i];
  }
  return dX;
}

//笛卡尔空间六自由度导纳控制,输出控制状态
Admittance_Status_Cart Controller::getCrtllerOutput(const Admittance_Args_Cart &sysArgs, const Admittance_Status_Cart &targetStatus, const Admittance_Status_Cart &actualStatus)
{
  double dt  = 1.0 / CTRL_FREQ;
  Admittance_Status_Cart ctrlStatus;
  Eigen::Matrix<Eigen::Matrix<double,2,1>,6,1> dX;
  Eigen::Matrix3d tragetRotationMatrix;
  Eigen::Matrix<double,6,1> poseDiff;

  poseDiff << targetStatus.pose.block<3,1>(0,0) - actualStatus.pose.block<3,1>(0,0),
      rotationVectorDiff_eulerXYZ(targetStatus.pose.block<3,1>(3,0), actualStatus.pose.block<3,1>(3,0));
  //  std::cout << "poseDiff" << poseDiff.transpose() << std::endl;

  //  std::cout <<"speed" << actualStatus.speed.transpose() << std::endl;
  //  std::cout <<"pose" << poseDiff.transpose() << std::endl;
  dX = admittance_cart(sysArgs, poseDiff, targetStatus.speed - actualStatus.speed, targetStatus.force - actualStatus.force);

  for(int i=0;i<6;i++)
  {
    ctrlStatus.speed[i] = actualStatus.speed[i] - dX[i](1,0) * dt;
    ctrlStatus.pose[i] = ctrlStatus.speed[i] * dt;
  }

  Eigen::Quaterniond quaternion = eulerXYZ_quaternion(ctrlStatus.pose.block<3,1>(3,0));

  ctrlStatus.pose << ctrlStatus.pose.block<3,1>(0,0) + actualStatus.pose.block<3,1>(0,0),
      quaternion_rotationVector(quaternion * rotationVector_quaternion(actualStatus.pose.block<3,1>(3,0)));

  return ctrlStatus;
}


/**************************************HRC_Assembly****************************************/

HRC_Assembly::HRC_Assembly(ros::NodeHandle& node_handle, const std::string pub_name):pubName(pub_name),n(node_handle)
{
  pub = n.advertise<std_msgs::Int32>(pubName, 100);
}
//柔顺拖动导纳参数初始化
void HRC_Assembly::dragAdmArgsInit(Admittance_Args_Cart& Args)
{
  Args.Alx = Args.Aly = Args.Alz = (Eigen::MatrixXd(2,2) << 0, 1, 0, -30).finished();
  Args.Arx = Args.Ary = Args.Arz = (Eigen::MatrixXd(2,2) << 0, 1, 0, -80).finished();

  Args.Blx = Args.Bly = Args.Blz = (Eigen::MatrixXd(2,1) << 0, 0.3).finished();
  Args.Brx = Args.Bry = Args.Brz = (Eigen::MatrixXd(2,1) << 0, 0.0).finished();

//    Args.Alx = Args.Aly = Args.Alz = (Eigen::MatrixXd(2,2) << 0, 1, 0, -10).finished();
//    Args.Arx = Args.Ary = Args.Arz = (Eigen::MatrixXd(2,2) << 0, 1, 0, -80).finished();

//    Args.Blx = Args.Bly = Args.Blz = (Eigen::MatrixXd(2,1) << 0, 2.0).finished();
//    Args.Brx = Args.Bry = Args.Brz = (Eigen::MatrixXd(2,1) << 0, 0.0).finished();
}

//位置控制导纳初始化
void HRC_Assembly::poseCtrlArgsInit(Admittance_Args_Cart& Args)
{
  Args.Alx = Args.Aly = Args.Alz = (Eigen::MatrixXd(2,2) << 0, 1, -100, -2 * std::sqrt(1000)).finished();
  Args.Arx = Args.Ary = Args.Arz = (Eigen::MatrixXd(2,2) << 0, 1, -10, -2 * std::sqrt(100)).finished();
  Args.Blx = Args.Bly = Args.Blz = (Eigen::MatrixXd(2,1) << 0, 0).finished();
  Args.Brx = Args.Bry = Args.Brz = (Eigen::MatrixXd(2,1) << 0, 0).finished();
}

//正阻抗导纳初始化
void HRC_Assembly::positiveAdmArgsInit(Admittance_Args_Cart& Args)
{
  Args.Alx = Args.Aly = (Eigen::MatrixXd(2,2) << 0, 1, -5, -2 * std::sqrt(200)).finished();
  Args.Alz = (Eigen::MatrixXd(2,2) << 0, 1, -10, -2 * std::sqrt(200)).finished();
  Args.Arx = Args.Ary = Args.Arz = (Eigen::MatrixXd(2,2) << 0, 1, -5, -2 * std::sqrt(200)).finished();
  Args.Blx = Args.Bly = Args.Blz = (Eigen::MatrixXd(2,1) << 0, 0.01).finished();
  Args.Brx = Args.Bry = Args.Brz = (Eigen::MatrixXd(2,1) << 0, 0).finished();
}

//负阻抗导纳初始化
void HRC_Assembly::negativeAdmArgsInit(Admittance_Args_Cart& Args)
{
  Args.Alx = Args.Aly = (Eigen::MatrixXd(2,2) << 0, 1, 0, -30).finished();
  Args.Arx = Args.Ary = Args.Arz = (Eigen::MatrixXd(2,2) << 0, 1, 0, -80).finished();

  Args.Blx = Args.Bly = (Eigen::MatrixXd(2,1) << 0, 0.3).finished();
  Args.Brx = Args.Bry = Args.Brz = (Eigen::MatrixXd(2,1) << 0, 0.0).finished();

  Args.Alz = (Eigen::MatrixXd(2,2) << 0, 1, -5, -50).finished();
  Args.Blz = (Eigen::MatrixXd(2,1) << 0, -0.02).finished();

  //  Args.Alx = Args.Aly = (Eigen::MatrixXd(2,2) << 0, 1, -10, -2 * std::sqrt(100)).finished();
  //  Args.Alz = (Eigen::MatrixXd(2,2) << 0, 1, +10, -2 * std::sqrt(100)).finished();
  //  Args.Arx = Args.Ary = Args.Arz = (Eigen::MatrixXd(2,2) << 0, 1, -10, -2 * std::sqrt(100)).finished();
  //  Args.Blx = Args.Bly = Args.Blz = (Eigen::MatrixXd(2,1) << 0, 0.3).finished();
  //  Args.Brx = Args.Bry = Args.Brz = (Eigen::MatrixXd(2,1) << 0, 0).finished();
}

//恒力控制导纳初始化
void HRC_Assembly::constantForceAdmArgsInit(Admittance_Args_Cart& Args)
{
  Args.Alx = Args.Aly = Args.Alz = (Eigen::MatrixXd(2,2) << 0, 1, -1, -2 * std::sqrt(1000)).finished();
  Args.Arx = Args.Ary = Args.Arz = (Eigen::MatrixXd(2,2) << 0, 1, -1, -2 * std::sqrt(1000)).finished();
  Args.Blx = Args.Bly = Args.Blz = (Eigen::MatrixXd(2,1) << 0, 0.3).finished();
  Args.Brx = Args.Bry = Args.Brz = (Eigen::MatrixXd(2,1) << 0, 0).finished();
}

void HRC_Assembly::modeSwitch_DI(uint64_t digitalInput)
{
  modeSwitchFlag = false;
  if(digitalInput == 0) return;
  if((digitalInput & (digitalInput-1)) != 0) return;
  int keynum = 0;
  while(digitalInput != 0)
  {
    digitalInput = digitalInput/2;
    keynum += 1;
  }
  if(mode == HRC_Assembly_Mode(keynum)) return;

  modeSwitch_mode(HRC_Assembly_Mode(keynum));
}

void HRC_Assembly::modeSwitch_status(Admittance_Status_Cart Status, Eigen::Matrix<double,6,1> assemblyPose)
{
  modeSwitchFlag = false;
  switch(mode)
  {
  case mode1:
    if(Status.pose[2] < assemblyPose[2] + 0.015)
    {
      modeSwitch_mode(mode3);
    }
    break;
  case mode2:
//    if(Status.pose[2] < 0.07)
//    {
//      modeSwitch_mode(mode3);
//    }
    break;
  case mode3:
    break;
  case mode4:
    break;
  case mode5:
    break;
  default:
    printf("模式错误\n");
    break;
  }
}

void HRC_Assembly::modeSwitch_mode(HRC_Assembly_Mode getMode)
{
  lastMode = mode;
  mode = getMode;
  switch(getMode)
  {
  case mode1:
    poseCtrlArgsInit(admSysArgs);
    printf("mode1\n");
    modeSwitchFlag = true;
    break;
  case mode2:
    dragAdmArgsInit(admSysArgs);
    printf("mode2\n");
    modeSwitchFlag = true;
    break;
  case mode3:
    positiveAdmArgsInit(admSysArgs);
    printf("mode3\n");
    modeSwitchFlag = true;
    break;
  case mode4:
    negativeAdmArgsInit(admSysArgs);
    printf("mode4\n");
    modeSwitchFlag = true;
    break;
  case mode5:
    poseCtrlArgsInit(admSysArgs);
    printf("mode5\n");
    modeSwitchFlag = true;
    break;
  default:
    printf("模式转换指令错误，请重新输入\n");
    break;
  }
  modeMsg.data = mode;
  pub.publish(modeMsg);
}

void HRC_Assembly::admArgsDynamicAdjust(Admittance_Args_Cart& sysArgs, Admittance_Status_Cart &admStatus)
{
  switch(mode){
  case mode1:

    break;
  case mode2:
    break;
  case mode3:
  {
    int k = 3;
    admStatus.force[0] += k * admStatus.force[4];
    admStatus.force[1] += -k * admStatus.force[3];

    double xyFTSum = std::abs(admStatus.force[0]) + std::abs(admStatus.force[1]);
    sysArgs.Blz[1] = 0.0 + 0.02 * ((xyFTSum < 20) ? xyFTSum : 20) / 20;
  }
    break;
  case mode4:
   if(std::abs(admStatus.force[2]) > 40.0){
     sysArgs.Blz = (Eigen::MatrixXd(2,1) << 0, -0.00).finished();
     sysArgs.Blx = sysArgs.Bly = (Eigen::MatrixXd(2,1) << 0, 0.05).finished();
   }else{
     sysArgs.Alz = (Eigen::MatrixXd(2,2) << 0, 1, -5, -50).finished();
     sysArgs.Blz = (Eigen::MatrixXd(2,1) << 0, -0.03).finished();
   }
    break;
  case mode5:
    break;
  default:
    printf("模式错误\n");
    break;
  }
}

/**************************************HRC_Assembly****************************************/

/*****************************************HRC_Assembly_Task********************************************/
//正阻抗导纳初始化
void HRC_Assembly_Task::positiveAdmArgsInit(Admittance_Args_Cart& Args)
{
  Args.Alx = Args.Aly = Args.Alz = (Eigen::MatrixXd(2,2) << 0, 1, -10, -2 * std::sqrt(100)).finished();
  Args.Arx = Args.Ary = Args.Arz = (Eigen::MatrixXd(2,2) << 0, 1, -10, -2 * std::sqrt(100)).finished();
  Args.Blx = Args.Bly = Args.Blz = (Eigen::MatrixXd(2,1) << 0, 0.3).finished();
  Args.Brx = Args.Bry = Args.Brz = (Eigen::MatrixXd(2,1) << 0, 0).finished();
}

//装配任务参数调节,笛卡尔空间六自由度导纳
void HRC_Assembly_Task::assemblyAdmArgsDynamicAdjust( Admittance_Args_Cart& sysArgs, Eigen::Matrix<double,6,1>& u)
{
  int k = 5;
  u[0] += k * u[4];
  u[1] += -k * u[3];
  double xyFTSum = std::abs(u[0]) + std::abs(u[1]);
  sysArgs.Blz[1] = 0.005 + 0.01 * ((xyFTSum < 20) ? xyFTSum : 20) / 20;
}

/*****************************************HRC_Assembly_Task********************************************/

/**********************************************HRC_VF*************************************************/
//柔顺拖动导纳参数初始化
void HRC_VF_Task::dragAdmArgsInit(Admittance_Args_Cart& Args)
{
  Args.Alx = Args.Aly = Args.Alz = (Eigen::MatrixXd(2,2) << 0, 1, 0, -20).finished();
  Args.Arx = Args.Ary = Args.Arz = (Eigen::MatrixXd(2,2) << 0, 1, 0, -20).finished();

  Args.Blx = Args.Bly = Args.Blz = (Eigen::MatrixXd(2,1) << 0, 0.3).finished();
  Args.Brx = Args.Bry = Args.Brz = (Eigen::MatrixXd(2,1) << 0, 15.0).finished();
}

//柔顺拖动导纳参数随速度动态变化
void HRC_VF_Task::dragAdmArgsDynamicAdjust(Admittance_Args_Cart& Args,Eigen::Matrix<double,6,1>& speed){
  Args.Alx(1,1) = -12 + -80 * std::abs(speed(0,0));
  Args.Aly(1,1) = -12 + -80 * std::abs(speed(1,0));
  Args.Alz(1,1) = -12 + -80 * std::abs(speed(2,0));
}

//虚拟夹具z方向变导纳
void HRC_VF_Task::zVFAdmArgsDynamicAdjust(Admittance_Args_Cart& Args,Eigen::Matrix<double,6,1>& pose,Eigen::Matrix<double,6,1>& speed,  Eigen::Matrix<double,6,1>& FT)
{
  if(borderState.triggerbuffer == false){
    borderState.bufferThickness = 0.001 + std::abs(speed(2))*0.1;
  }
  if(pose(2) - borderState.z0 > borderState.bufferThickness)
  {
    borderState.triggerbuffer = false;
    return;
  }
  else if(borderState.triggerbuffer == false)
  {
    borderState.Bmax = (1243 * std::exp(-5.382 * std::abs(speed(2))) +1028 * std::exp(-67.88 * std::abs(speed(2))))/3.33 ;
    borderState.constraintThickness = 0.7493 * std::pow(std::abs(speed(2)), 3) -0.1396 * std::pow(std::abs(speed(2)), 2) + 0.03858 * std::abs(speed(2)) - 0.0001958;
    borderState.triggerbuffer = true;
    borderState.Bmin = -Args.Alz(1,1);
    borderState.bufferKB = (borderState.Bmax - borderState.Bmin) / std::pow(borderState.bufferThickness,0.25);
    borderState.bufferKF = (borderState.Fmax - borderState.Fmin) / std::pow(borderState.bufferThickness,4);
  }

  if(pose(2) - borderState.z0 > borderState.constraintThickness)
  {
    borderState.triggerconstraint = false;
  }
  else if(borderState.triggerconstraint == false)
  {
    borderState.triggerconstraint = true;
    borderState.constraintB = - borderState.Bmax + borderState.bufferKB * std::pow((pose(2) - borderState.z0),0.25);
    borderState.constraintKF = borderState.Flimit / (pose(2) - borderState.z0);
  }

  if(borderState.triggerbuffer && !borderState.triggerconstraint)
  {
    if(speed(2) < 0)
    {
      Args.Alz(1,1) = - borderState.Bmax + borderState.bufferKB * std::pow((pose(2) - borderState.z0),0.25);
    }
    borderState.Flimit = borderState.Fmin + borderState.bufferKF * std::pow((pose(2) - borderState.z0),4);
    if(std::abs(FT(2)) > borderState.Flimit && FT(2) < 0){
      FT(2) = borderState.Flimit * FT(2) / std::abs(FT(2));
    }
    std::cout << "bufferBAdjust " << Args.Alz(1,1) <<std::endl;

    std::cout << "bufferFlimit " << borderState.Flimit <<std::endl<<std::endl;
  }

  if(borderState.triggerconstraint)
  {
    if(speed(2) < 0)
    {
      Args.Alz(1,1) = borderState.constraintB;
    }
    borderState.Flimit = borderState.constraintKF * (pose(2) - borderState.z0);
    if(std::abs(FT(2)) > borderState.Flimit && FT(2) < 0){
      FT(2) = borderState.Flimit * FT(2) / std::abs(FT(2));
    }
    std::cout << "constraintBAdjust " << Args.Alz(1,1) <<std::endl;

    std::cout << "constraintFlimit " << borderState.Flimit <<std::endl<<std::endl;
  }

  if(pose(2) - borderState.z0 < 0 && FT(2) < 0)
  {
    Args.Alz(1,1) = borderState.constraintB;
    FT(2) = 0;
  }
}

/**********************************************HRC_VF*************************************************/
