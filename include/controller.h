#ifndef CONTROLLER_H
#define CONTROLLER_H
#include <ros/ros.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <deque>
#include <math.h>
#include <Eigen/Dense>
#include <config.h>
#include <virtual_fixture.h>
#include  <std_msgs/Int32.h>

typedef struct
{
  Eigen::Matrix<double,2,2> Alx = (Eigen::MatrixXd(2,2) << 0, 1, -10, -2 * std::sqrt(100)).finished();
  Eigen::Matrix<double,2,2> Aly = (Eigen::MatrixXd(2,2) << 0, 1, -10, -2 * std::sqrt(100)).finished();
  Eigen::Matrix<double,2,2> Alz = (Eigen::MatrixXd(2,2) << 0, 1, -10, -2 * std::sqrt(100)).finished();
  Eigen::Matrix<double,2,2> Arx = (Eigen::MatrixXd(2,2) << 0, 0, 0, 0).finished();
  Eigen::Matrix<double,2,2> Ary = (Eigen::MatrixXd(2,2) << 0, 0, 0, 0).finished();
  Eigen::Matrix<double,2,2> Arz = (Eigen::MatrixXd(2,2) << 0, 0, 0, 0).finished();

  Eigen::Matrix<double,2,1> Blx = (Eigen::MatrixXd(2,1) << 0, 0.005).finished();
  Eigen::Matrix<double,2,1> Bly = (Eigen::MatrixXd(2,1) << 0, 0.005).finished();
  Eigen::Matrix<double,2,1> Blz = (Eigen::MatrixXd(2,1) << 0, 0.005).finished();
  Eigen::Matrix<double,2,1> Brx = (Eigen::MatrixXd(2,1) << 0, 0).finished();
  Eigen::Matrix<double,2,1> Bry = (Eigen::MatrixXd(2,1) << 0, 0).finished();
  Eigen::Matrix<double,2,1> Brz = (Eigen::MatrixXd(2,1) << 0, 0).finished();
}Admittance_Args_Cart;

typedef struct
{
  Eigen::Matrix<double,6,1> pose;
  Eigen::Matrix<double,6,1> speed;
  Eigen::Matrix<double,6,1> accel;
  Eigen::Matrix<double,6,1> force;
}Admittance_Status_Cart;

typedef struct
{
  bool triggerbuffer = false;
  double Bmin;
  double Bmax = 180;
  double Fmax = 50;
  double Fmin = 20;
  double z0 = 0.1;
  double bufferThickness = 0.015;
  double bufferKB;
  double bufferKF;
  double Flimit;

  bool triggerconstraint = false;
  double constraintThickness = 0.005;
  double constraintB;
  double constraintKF;
}Border_State;

enum HRC_Assembly_Mode{
  mode1 = 1,
  mode2 = 2,
  mode3 = 3,
  mode4 = 4,
  mode5 = 5,
  contol_mode_end
};

class Controller
{
private:

public:

  Admittance_Status_Cart targetStatus;
  Admittance_Status_Cart actualStatus;
  Admittance_Status_Cart ctrlStatus;

  Eigen::Matrix<Eigen::Matrix<double,2,1>,6,1> admittance_cart(const Admittance_Args_Cart sysArgs, Eigen::Matrix<double,6,1> x1, Eigen::Matrix< double,6,1> x2, Eigen::Matrix<double,6,1> u);

  //笛卡尔空间六自由度导纳控制
  Admittance_Status_Cart getCrtllerOutput(const Admittance_Args_Cart &sysArgs, const Admittance_Status_Cart &targetStatus, const Admittance_Status_Cart &actualStatus);
};


class HRC_Assembly
{
private:
  std::string pubName;
  ros::NodeHandle n;
  ros::Publisher pub;
  std_msgs::Int32 modeMsg;
public:
  Admittance_Args_Cart admSysArgs;
  Admittance_Status_Cart targetStatus;
  Admittance_Status_Cart actualStatus;
  Admittance_Status_Cart ctrlStatus;
  HRC_Assembly_Mode mode = mode2;
  HRC_Assembly_Mode lastMode = mode2;
  bool modeSwitchFlag = false;

  HRC_Assembly(ros::NodeHandle& node_handle, const std::string pub_name);

  //柔顺拖动导纳参数初始化
  void dragAdmArgsInit( Admittance_Args_Cart& Args);
  //位置控制导纳初始化
  void poseCtrlArgsInit(Admittance_Args_Cart& Args);
  //正阻抗导纳初始化
  void positiveAdmArgsInit(Admittance_Args_Cart& Args);
  //负阻抗导纳初始化
  void negativeAdmArgsInit(Admittance_Args_Cart& Args);
  //恒力控制导纳初始化
  void constantForceAdmArgsInit(Admittance_Args_Cart& Args);

  void ModeSwitch_keyboard(char keyChar);
  void modeSwitch_mode(HRC_Assembly_Mode getMode);
  void modeSwitch_DI(uint64_t digitalInput);
  void modeSwitch_status(Admittance_Status_Cart Status, Eigen::Matrix<double,6,1> assemblyPose);

  void admArgsDynamicAdjust(Admittance_Args_Cart& sysArgs, Admittance_Status_Cart &admStatus);
};

class HRC_Assembly_Task
{
private:
  Admittance_Args_Cart admArgs;
  Admittance_Status_Cart targetStatus;
  Admittance_Status_Cart actualStatus;
  Admittance_Status_Cart ctrlStatus;
public:
  //正阻抗导纳初始化
  void positiveAdmArgsInit(Admittance_Args_Cart& Args);
void assemblyAdmArgsDynamicAdjust(Admittance_Args_Cart& sysArgs, Eigen::Matrix<double,6,1>& u);
};

class HRC_VF_Task
{
private:

public:
  Admittance_Args_Cart admArgs;
  Admittance_Status_Cart targetStatus;
  Admittance_Status_Cart actualStatus;
  Admittance_Status_Cart ctrlStatus;
  Border_State borderState;
  //柔顺拖动导纳参数初始化
  void dragAdmArgsInit(Admittance_Args_Cart& Args);
  void dragAdmArgsDynamicAdjust(Admittance_Args_Cart& Args,Eigen::Matrix<double,6,1>& speed);
  //虚拟夹具z方向变导纳
  void zVFAdmArgsDynamicAdjust(Admittance_Args_Cart& Args,Eigen::Matrix<double,6,1>& pose,Eigen::Matrix<double,6,1>& speed,  Eigen::Matrix<double,6,1>& FT);
};
#endif // CONTROLLER_H
