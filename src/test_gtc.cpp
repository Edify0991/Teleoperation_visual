//#include <ros/ros.h>
//#include <unistd.h>
//#include <iostream>

//#include <config.h>
//#include <process_ft_data.h>
//#include <process_robot_data.h>
//#include <controller.h>
//#include "hrc/KeyValue.h"
//#include "process_keyboard_event.h"

//int main(int argc, char **argv)
//{
//  //基础类初始化
//  Process_Robot_Data URData;
//  Controller adm;
//  Robot_Move URMove;
//  Transfer_Matrix transM;
//  transM_P = &transM;


//  //ros初始化
//  ros::init(argc, argv, "data_processing");
//  ros::NodeHandle n;
//  HRC_Assembly ctrllerArgs(n,"/mode");
//  Process_FT_Data KW_FT(n,"/ft_data","/processed_ft_data",TC_);
//  //  Process_Keyboard_Event key(n,"key_value");
//  ros::Subscriber URDataSub = n.subscribe("ur_data", 100, &Process_Robot_Data::robotDataCallback, &URData);
//  ros::Publisher  urConParPub = n.advertise<hrc::RobotControl>("ur_control", 100);
//  ros::Rate loop_rate(CTRL_FREQ);
//  ros::AsyncSpinner AS(1);
//  AS.start();

//  std::string dir;
//  n.getParam("reocrdDir", dir);

//  //等待力传感器数据队列存满
//  while(KW_FT.getFTDeque("origen_world").size() < KW_FT.FTDataBuffer && ros::ok()){
//    loop_rate.sleep();
//  }

//  //定义机器人初始位姿，速度
//  std::vector<double> initPose = INITPOSE;
//  ctrllerArgs.ctrlStatus.pose << Eigen::Map<Eigen::MatrixXd>(&initPose[0], 6, 1);
//  ctrllerArgs.ctrlStatus.speed << 0.0,0.0,0.0,0.0,0.0,0.0;

//  //等待机器人到达初始位姿
//  int i = 0;
//  while(!URData.isReachTargetPose(ctrllerArgs.ctrlStatus.pose,0.001) && ros::ok()){
//    usleep(1000);
//    if(i++ > 6000){
//      printf("机器人没有到达指定初始位置\n");
//      exit(0);
//    }
//  }
//  ctrllerArgs.actualStatus.pose << Eigen::Map<Eigen::MatrixXd>(&initPose[0], 6, 1);
//  ctrllerArgs.actualStatus.speed << 0.0,0.0,0.0,0.0,0.0,0.0;

//  //等待力传感器完成校准
//#if USE_FT_SIM
//  KW_FT.FTNeedCalibration = false;
//#else
//  while(KW_FT.FTNeedCalibration == true && ros::ok()){
//    transM.set_TR_robot_flange(URData.getRobotData("targetFlangePose"));
//    KW_FT.setFTDateZeroError(2.0);
//  }
//#endif


//  Eigen::Matrix<double,6,1> assemblyPose;
//  assemblyPose << 0.11845+0.005, -0.43127+0.005, 0.02391, 0.0009, -3.1434, 0.0062;
//  //  assemblyPose << 0.11929+0.005, -0.43258+0.005, -0.02312, 0.0116, -3.1392, 0.0137;
//  Trajectory_Plan trajectory(URData.getRobotData("actualFlangePose"));
//  trajectory.addAssemblyTrajectory(assemblyPose);
//  trajectory.startFlag = false;
//  ctrllerArgs.dragAdmArgsInit(ctrllerArgs.admSysArgs);


//  //初始化完成，进入主循环
//  while(ros::ok())
//  {
//    ctrllerArgs.targetStatus.pose = trajectory.getNextPlanPose();
////        ctrllerArgs.actualStatus.pose = ctrllerArgs.ctrlStatus.pose;
//    ctrllerArgs.actualStatus.pose = URData.getRobotData("actualFlangePose");
//    ctrllerArgs.actualStatus.speed = ctrllerArgs.ctrlStatus.speed;
//    //    ctrllerArgs.actualStatus.speed = URData.getRobotData("actualFlangeSpeed");
//    ctrllerArgs.actualStatus.force = KW_FT.getFTData("filtered_gtced_world");
//    //    ctrllerArgs.actualStatus.force = Eigen::MatrixXd::Zero(6,1);
//    ctrllerArgs.targetStatus.speed = Eigen::MatrixXd::Zero(6,1);
//    ctrllerArgs.targetStatus.force = Eigen::MatrixXd::Zero(6,1);

//    ctrllerArgs.modeSwitch_DI(URData.getDigitalInput());
//    ctrllerArgs.modeSwitch_status(ctrllerArgs.actualStatus,assemblyPose);

//    if(ctrllerArgs.modeSwitchFlag == true)
//    {
//      if(ctrllerArgs.actualStatus.pose[2] > assemblyPose[2] + 0.03){
//        trajectory.eraseTargetPose();
//        trajectory.addAssemblyTrajectory(assemblyPose);
//      }else{
//        trajectory.nextPlanPose.pose = ctrllerArgs.actualStatus.pose;
//        trajectory.nextPlanPose.accel = 0;
//        trajectory.nextPlanPose.speed = 0;
//      }
//      switch(ctrllerArgs.mode){
//      case mode1:
//        trajectory.nextPlanPose.pose << URData.getRobotData("actualFlangePose");
//        trajectory.startFlag = true;
//        break;
//      case mode2:
//        trajectory.nextPlanPose.pose << URData.getRobotData("actualFlangePose");
//        trajectory.startFlag = false;
//        break;
//      case mode3:
//        trajectory.nextPlanPose.pose << URData.getRobotData("actualFlangePose");
//        trajectory.startFlag = true;
//        break;
//      case mode4:
//        trajectory.startFlag = false;
//        trajectory.nextPlanPose.pose << URData.getRobotData("actualFlangePose");
//        break;
//      case mode5:
//        trajectory.startFlag = false;
//        trajectory.nextPlanPose.pose << URData.getRobotData("actualFlangePose");
//        KW_FT.setGCArgs(FT_);
//        ctrllerArgs.ctrlStatus.speed << 0.0,0.0,0.0,0.0,0.0,0.0;
//        URMove.sendSpeedLMsg(urConParPub, ctrllerArgs.ctrlStatus.speed);
//#if !USE_FT_SIM
//        KW_FT.FTNeedCalibration = true;
//        while(KW_FT.FTNeedCalibration == true && ros::ok()){
//          transM.set_TR_robot_flange(URData.getRobotData("targetFlangePose"));
//          KW_FT.setFTDateZeroError(0.1);
//        }
//#endif
//        ctrllerArgs.modeSwitch_mode(ctrllerArgs.lastMode);
//        break;
//      default:
//        printf("模式错误2\n");
//        break;
//      }
//    }

//    KW_FT.virtualStaticFrictionProcess(ctrllerArgs.actualStatus.force,KW_FT.staticFrictionFT);

//    ctrllerArgs.admArgsDynamicAdjust(ctrllerArgs.admSysArgs, ctrllerArgs.actualStatus);

//    ctrllerArgs.ctrlStatus = adm.getCrtllerOutput(ctrllerArgs.admSysArgs, ctrllerArgs.targetStatus, ctrllerArgs.actualStatus);

//    for (i=0;i<6;i++){
//      if(isnan(ctrllerArgs.ctrlStatus.speed[i])){
//        printf("导纳控制器速度状态计算出现错误%d\n",i);
//        ctrllerArgs.ctrlStatus.speed[i] = 0.0;
//      }
//    }
//    //    if((ctrllerArgs.targetStatus.pose - ctrllerArgs.actualStatus.pose).norm() > 0.8){
//    //      printf("位置误差过大！n");
//    //      ctrllerArgs.ctrlStatus.speed = Eigen::MatrixXd::Zero(6,1);
//    //    }

//    URMove.sendSpeedLMsg(urConParPub, ctrllerArgs.ctrlStatus.speed);
////        URMove.sendPoseMsg(urConParPub, ctrllerArgs.ctrlStatus.pose);

//    //    URMove.sendteachMsg(urConParPub, true);
//    loop_rate.sleep();
//  }

//  ctrllerArgs.ctrlStatus.speed << 0.0,0.0,0.0,0.0,0.0,0.0;
//  URMove.sendSpeedLMsg(urConParPub, ctrllerArgs.ctrlStatus.speed);
//  return 0;
//}
