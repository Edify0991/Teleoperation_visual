#include <ros/ros.h>
#include <unistd.h>
#include <iostream>

#include <config.h>
#include <virtual_fixture.h>
#include <process_ft_data.h>
#include <process_robot_data.h>
#include <controller.h>
#include <virtual_fixture.h>

int main(int argc, char **argv)
{
  //基础类初始化
  Process_Robot_Data URData;
  HRC_VF_Task vf;
  Robot_Move URMove;
  Transfer_Matrix transM;
  Controller controllor;
  transM_P = &transM;

//  Admittance_Args_Cart admArgs;
//  Admittance_Status_Cart targetStatus;
//  Admittance_Status_Cart actualStatus;
//  Admittance_Status_Cart ctrlStatus;

  //ros初始化
  ros::init(argc, argv, "data_processing");
  ros::NodeHandle n;
  Process_FT_Data KW_FT(n,"/ft_data","/processed_ft_data",H_);
  //  ros::Subscriber FTDataSub = n.subscribe("ft_data", 100,  boost::bind(&Process_FT_Data::ftDataCallback, &KW_FT, _1),&KW_FT);
  ros::Subscriber URDataSub = n.subscribe("ur_data", 100, &Process_Robot_Data::robotDataCallback, &URData);
  ros::Publisher  urConParPub = n.advertise<hrc::RobotControl>("ur_control", 100);
  ros::Rate loop_rate(CTRL_FREQ);
  ros::AsyncSpinner AS(1);
  AS.start();

  //等待力传感器数据队列存满
  while(KW_FT.getFTDeque("origen_world").size() < KW_FT.FTDataBuffer && ros::ok()){
    loop_rate.sleep();
  }

  //定义机器人初始位姿，速度
  std::vector<double> initPose = INITPOSE;
  vf.ctrlStatus.pose << Eigen::Map<Eigen::MatrixXd>(&initPose[0], 6, 1);
  vf.ctrlStatus.speed << 0.0,0.0,0.0,0.0,0.0,0.0;

  //等待机器人到达初始位姿
  int i = 0;
  while(!URData.isReachTargetPose(vf.ctrlStatus.pose,0.001) && ros::ok()){
    usleep(1000);
    if(i++ > 6000){
      printf("机器人没有到达指定初始位置\n");
      return 0;
    }
  }
  vf.actualStatus.pose << Eigen::Map<Eigen::MatrixXd>(&initPose[0], 6, 1);
  vf.actualStatus.speed << 0.0,0.0,0.0,0.0,0.0,0.0;

  //等待力传感器完成校准
#if USE_FT_SIM
  KW_FT.FTNeedCalibration = false;
#else
  while(KW_FT.FTNeedCalibration == true && ros::ok()){
    transM.set_TR_robot_flange(URData.getRobotData("targetFlangePose"));
    KW_FT.setFTDateZeroError(0.3);
  }
#endif

  vf.dragAdmArgsInit(vf.admArgs);
  Eigen::Matrix3d tragetRotationMatrix;
  Eigen::Matrix<double,3,1> tragetPosition;
  Eigen::Matrix<double,3,1> tragetPosture;
  Eigen::Matrix<double,3,1> tempRPY;
  int cycNum=0;

  vf.actualStatus.pose = URData.getRobotData("actualFlangePose");
  tragetRotationMatrix = quaternion_rotationMatrix(rotationVector_quaternion(vf.actualStatus.pose.block<3,1>(3,0)));
  tragetPosition = vf.actualStatus.pose.block<3,1>(0,0);
  tragetPosture = vf.actualStatus.pose.block<3,1>(3,0);
  //初始化完成，进入主循环
  while(ros::ok())
  {
//    actualStatus.pose = ctrlStatus.pose;
//    actualStatus.speed = ctrlStatus.speed;
    vf.actualStatus.pose = URData.getRobotData("actualFlangePose");
    vf.actualStatus.speed = URData.getRobotData("actualFlangeSpeed");
    vf.actualStatus.force = KW_FT.getFTData("filtered_gtced_world");
    vf.targetStatus.pose = vf.actualStatus.pose;
    vf.targetStatus.speed = Eigen::MatrixXd::Zero(6,1);
    vf.targetStatus.force = Eigen::MatrixXd::Zero(6,1);

    KW_FT.virtualStaticFrictionProcess(vf.actualStatus.force,KW_FT.staticFrictionFT);

    vf.dragAdmArgsDynamicAdjust(vf.admArgs,vf.actualStatus.speed);
//    std::cout << "dis " << vf.actualStatus.pose(2) - 0.1 << std::endl;
//    std::cout << "B    " << vf.admArgs.Alz(1,1) << std::endl;
    vf.zVFAdmArgsDynamicAdjust(vf.admArgs, vf.actualStatus.pose, vf.ctrlStatus.speed, vf.actualStatus.force);

//    vf.actualStatus.pose = vf.actualStatus.pose;
//     vf.ctrlStatus.speed =
//        controllor.admittance_cart(vf.admArgs,
//          vf.targetStatus.pose - vf.targetStatus.pose,
////          actualFlangeSpeed - vf.targetVel,
//          vf.ctrlStatus.speed - vf.targetStatus.speed,
//          vf.actualStatus.force);


//    vf.ctrlStatus = controllor.getCrtllerOutput(vf.admArgs, vf.targetStatus, vf.actualStatus);

//    URMove.sendSpeedLMsg(urConParPub, vf.ctrlStatus.speed);

//    tragetPosition = ctrlStatus.speed.block<3,1>(0,0) / CTRL_FREQ + tragetPosition;
//    tragetRotationMatrix = RPY_rotationMatrix(ctrlStatus.speed.block<3,1>(3,0) / CTRL_FREQ) *  tragetRotationMatrix;
//    tragetPosture = quaternion_rotationVector(euler_quaternion(rotationMatrix_euler(tragetRotationMatrix)));
//    ctrlStatus.pose << tragetPosition, tragetPosture;

//    URMove.sendPoseMsg(urConParPub, ctrlStatus.pose);

//    URMove.sendteachMsg(urConParPub, true);

    if(cycNum  == 50)
    {
//      std::cout << "vf.controlVel " << vf.controlVel.transpose() << std::endl;
//      std::cout << "vf.controlPose " << vf.controlPose.transpose() << std::endl;
//      std::cout << "actualStatus.force " << actualStatus.force.transpose() << std::endl<< std::endl;
  //    std::cout << "origen_gtced_world: " << KW_FT.getFTData("origen_gtced_world").transpose() << std::endl;
  //    std::cout << "filtered_gtced_world: " << KW_FT.getFTData("filtered_gtced_world").transpose() << std::endl<<std::endl;
  //    std::cout << "Pose: " << URData.getRobotData("actualFlangePose").transpose() << std::endl;
  //    std::cout << "Speed: " << URData.getRobotData("actualFlangeSpeed").transpose() << std::endl;
  //    std::cout << "controlVel: " << vf.controlVel.transpose() << std::endl;
  //    std::cout << "***************************************************************"<< std::endl;
      cycNum = 0;
    }
    cycNum++;
    loop_rate.sleep();
  }
  return 0;
}
