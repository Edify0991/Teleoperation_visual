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
#include <ros/ros.h>
#include <config.h>
#include <process_ft_data.h>

int main(int argc, char** argv) {
    //基础类初始化
    Process_Robot_Data URData;
    HRC_VF_Task vf;
    Robot_Move URMove;
    Transfer_Matrix transM;
    Controller controllor;
    transM_P = &transM;

    //ros初始化
    ros::init(argc, argv, "myadmittance_cart");
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
    Eigen::Matrix<double,6,1> targetVel;    //目标速度控制量
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
        //这里的vf.targetVel没有找到相关定义，是不是想表达目标速度控制量但是没有定义？
        /*vf.ctrlStatus.speed =
        controllor.admittance_cart(vf.admArgs,
          vf.targetStatus.pose - vf.targetStatus.pose,
          actualFlangeSpeed - vf.targetVel,
          vf.ctrlStatus.speed - vf.targetStatus.speed,
          vf.actualStatus.force);   //看起来这里没有这句命令的必要，因为在下一句命令就会将其覆盖
        */
        targetVel = vf.ctrlStatus.speed;
        vf.ctrlStatus = controllor.getCrtllerOutput(vf.admArgs, vf.targetStatus, vf.actualStatus);

        //在getCrtllerOutput函数中处理后得到的是δV而不是V+δV，为什么在这里直接发布？
        //所以我在上一句命令之前及之后写了一些注释中的命令，这样可以吗？
        vf.ctrlStatus.speed[3] = 0;
        vf.ctrlStatus.speed[4] = 0;
        vf.ctrlStatus.speed[5] = 0;
        //URMove.sendSpeedLMsg(urConParPub, vf.ctrlStatus.speed);
        
        //这里原来是tragetPosition = ctrlStatus.speed.block<3,1>(0,0) / CTRL_FREQ + tragetPosition;，但是ctrlStatus是vf类中的应该不能直接使用吧？
        //此外，vf.ctrlStatus中已经存储了对位置的控制量，这里的积分操作是不是多余了？
        tragetPosition = vf.ctrlStatus.speed.block<3,1>(0,0) / CTRL_FREQ + tragetPosition;  
        tragetRotationMatrix = RPY_rotationMatrix(vf.ctrlStatus.speed.block<3,1>(3,0) / CTRL_FREQ) *  tragetRotationMatrix;
        // tragetPosture = quaternion_rotationVector(euler_quaternion(rotationMatrix_euler(tragetRotationMatrix)));
        tragetPosture << 0, 3.1415926, 0;
        vf.ctrlStatus.pose << tragetPosition, tragetPosture;

        URMove.sendPoseMsg(urConParPub, vf.ctrlStatus.pose);
        //std::cout << vf.ctrlStatus.pose.transpose()<<  std::endl;
        //如果要做拖动实验，是不是可以不开示教模式？
        //URMove.sendteachMsg(urConParPub, true);
        loop_rate.sleep();
    }
    return 0;
}