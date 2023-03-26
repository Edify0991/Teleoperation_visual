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
#include <string>
#include <fstream>
#include <iterator>

class PT_adm {
private:

public:
  Eigen::Matrix<double,6,6> M, B, K;
  Admittance_Status_Cart targetStatus;
  Admittance_Status_Cart actualStatus;
  Admittance_Status_Cart ctrlStatus;
  Border_State borderState;
  //柔顺拖动导纳参数初始化
  void AdmArgsInit(Eigen::Matrix<double,6,6>& M, Eigen::Matrix<double,6,6>& B, Eigen::Matrix<double,6,6>& K);
  //void dragAdmArgsDynamicAdjust(Admittance_Args_Cart& Args,Eigen::Matrix<double,6,1>& speed);
  //虚拟夹具z方向变导纳
  //void zVFAdmArgsDynamicAdjust(Admittance_Args_Cart& Args,Eigen::Matrix<double,6,1>& pose,Eigen::Matrix<double,6,1>& speed,  Eigen::Matrix<double,6,1>& FT);
};

void AdmArgsInit(Eigen::Matrix<double,6,6>& M, Eigen::Matrix<double,6,6>& B, Eigen::Matrix<double,6,6>& K) {
    M << 10, 0, 0, 0, 0, 0,
        0, 10, 0, 0, 0, 0,
        0, 0, 10, 0, 0, 0,
        0, 0, 0, 10, 0, 0,
        0, 0, 0, 0, 10, 0,
        0, 0, 0, 0, 0, 10;
    B << 10, 0, 0, 0, 0, 0,
        0, 10, 0, 0, 0, 0,
        0, 0, 10, 0, 0, 0,
        0, 0, 0, 10, 0, 0,
        0, 0, 0, 0, 10, 0,
        0, 0, 0, 0, 0, 10;
    K << 10, 0, 0, 0, 0, 0,
        0, 10, 0, 0, 0, 0,
        0, 0, 10, 0, 0, 0,
        0, 0, 0, 10, 0, 0,
        0, 0, 0, 0, 10, 0,
        0, 0, 0, 0, 0, 10;
}  

Admittance_Status_Cart getControllOutput(const Eigen::Matrix<double,6,6>& M, const Eigen::Matrix<double,6,6>& B, const Eigen::Matrix<double,6,6>& K,
    const Admittance_Status_Cart& targetStatus, const Admittance_Status_Cart& actualStatus) {
    Eigen::Matrix<double,6,1> Xe_acc, Xe_vel, Xe_d;
    Admittance_Status_Cart ctrlStatus;
    double dt  = 1.0 / CTRL_FREQ;
    Eigen::Matrix3d tragetRotationMatrix;
    Eigen::Matrix<double,6,1> poseDiff, velDiff, forceDiff;
    //用轴角xyz表示两个旋转矢量的差角 v1 - v2
    //（为什么?)这一步计算得到Xe_x与Xe_vel，因为计算Xe_acc到Xe_vel的位置和姿态积分可直接叠加，但Xe_vel到Xe_x的姿态积分不能直接叠加
    poseDiff << targetStatus.pose.block<3,1>(0,0) - actualStatus.pose.block<3,1>(0,0),
      rotationVectorDiff_eulerXYZ(targetStatus.pose.block<3,1>(3,0), actualStatus.pose.block<3,1>(3,0));
    velDiff << targetStatus.speed - actualStatus.speed;
    forceDiff << targetStatus.force - actualStatus.force;
    //通过阻抗模型求得Xe_acc
    Xe_acc = M.inverse() * (forceDiff - B * velDiff - K * poseDiff);
    //Xu_vel = Xe_vel + actualStatus.speed
    //Xu_pose = Xe_d + actualStatus.pose
    for(int i=0;i<6;i++)
    {
        Xe_vel = Xe_acc * dt + velDiff;
        ctrlStatus.speed[i] = targetStatus.speed[i] - targetStatus.accel[i] * dt - Xe_vel;
        ctrlStatus.pose[i] = Xe_vel * dt;
    }
    Eigen::Quaterniond quaternion = eulerXYZ_quaternion(targetStatus.pose.block<3,1>(3,0));
    ctrlStatus.pose << ctrlStatus.pose.block<3,1>(0,0) + poseDiff.block<3,1>(0,0),
        quaternion_rotationVector(quaternion * rotationVector_quaternion(poseDiff.block<3,1>(3,0)));
    return ctrlStatus;
}


int main(int argc, char** argv) {
    //基础类初始化
    Process_Robot_Data URData;
    PT_adm pt;
    Robot_Move URMove;
    Transfer_Matrix transM;
    Controller controllor;
    transM_P = &transM;

    //ros初始化
    ros::init(argc, argv, "3DOF_Tra_admittance_cart");
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

    //定义机器人初始位姿，速度，点镇定为初始点
    std::vector<double> initPose = INITPOSE;
    pt.ctrlStatus.pose << Eigen::Map<Eigen::MatrixXd>(&initPose[0], 6, 1);
    pt.ctrlStatus.speed << 0.0,0.0,0.0,0.0,0.0,0.0;

    //等待机器人到达初始位姿
    int i = 0;
    while(!URData.isReachTargetPose(pt.ctrlStatus.pose,0.001) && ros::ok()){
        usleep(1000);
        if(i++ > 6000){
        printf("机器人没有到达指定初始位置\n");
        return 0;
        }
    }
    pt.actualStatus.pose << Eigen::Map<Eigen::MatrixXd>(&initPose[0], 6, 1);
    pt.actualStatus.speed << 0.0,0.0,0.0,0.0,0.0,0.0;

    //等待力传感器完成校准
    #if USE_FT_SIM
    KW_FT.FTNeedCalibration = false;
    #else
    while(KW_FT.FTNeedCalibration == true && ros::ok()){
        transM.set_TR_robot_flange(URData.getRobotData("targetFlangePose"));
        KW_FT.setFTDateZeroError(0.3);
    }
    #endif
    //初始化阻抗模型参数矩阵M、B、K
    AdmArgsInit(pt.M, pt.B, pt.K);
    Eigen::Matrix3d tragetRotationMatrix;
    Eigen::Matrix<double,3,1> tragetPosition;
    Eigen::Matrix<double,3,1> tragetPosture;
    Eigen::Matrix<double,6,1> targetPose;
    Eigen::Matrix<double,3,1> tempRPY;
    Eigen::Matrix<double,6,1> targetVel;    //目标速度控制量
    int cycNum=0;
    //获得当前时刻工具坐标系位姿
    pt.actualStatus.pose = URData.getRobotData("actualFlangePose");
    //将当前工具坐标系姿态先由旋转矢量转换为四元数，再转换为旋转矩阵
    //因为轨迹导纳控制的“定点”即为当前轨迹点位姿，故将该轨迹起始点旋转矩阵设为初始期望姿态
    tragetRotationMatrix = quaternion_rotationMatrix(rotationVector_quaternion(pt.actualStatus.pose.block<3,1>(3,0)));
    //轨迹导纳控制的“定点”即为当前轨迹点位姿，这里将轨迹起始点位置向量设为期望位置
    tragetPosition = pt.actualStatus.pose.block<3,1>(0,0);
    tragetPosture = pt.actualStatus.pose.block<3,1>(3,0);
    targetPose = pt.actualStatus.pose;
    //读取目标位姿、速度、加速度
    char tag = ' '; //记录的文件每行数据均以tag为间隔符
    std::vector<std::vector<double>> TargetPose, TargetSpeed, TargetAcc;
    TargetPose = readFile(tag, "RobotTargetPose.txt");
    TargetSpeed = readFile(tag, "RobotTargetSpeed.txt");
    TargetAcc = readFile(tag, "RobotTargetAcc.txt");
    std::vector<std::vector<double>>::iterator iter_pose, iter_speed, iter_acc;
    //初始化完成，进入主循环
    while(ros::ok())
    {
//    actualStatus.pose = ctrlStatus.pose;
//    actualStatus.speed = ctrlStatus.speed;
        //获取当前时刻工具坐标系位姿与外部力
        pt.actualStatus.pose = URData.getRobotData("actualFlangePose");
        pt.actualStatus.speed = URData.getRobotData("actualFlangeSpeed");
        pt.actualStatus.force = KW_FT.getFTData("filtered_gtced_world");
        //期望位姿为直线轨迹，期望速度，期望外力为0
        iter_pose = TargetPose.begin();
        iter_speed = TargetSpeed.begin();
        iter_acc = TargetAcc.begin();
        pt.targetStatus.pose = Eigen::Map<Eigen::Matrix<double,6,1>, Eigen::Unaligned> (*iter_pose.data(), *iter_pose.size());
        iter_pose++;
        pt.targetStatus.speed = Eigen::Map<Eigen::Matrix<double,6,1>, Eigen::Unaligned> (*iter_speed.data(), *iter_speed.size());
        iter_speed++;
        pt.targetStatus.accel = Eigen::Map<Eigen::Matrix<double,6,1>, Eigen::Unaligned> (*iter_acc.data(), *iter_acc.size());
        iter_acc++;
        pt.targetStatus.force = Eigen::MatrixXd::Zero(6,1);
        //虚拟静摩擦力 消除力动传感器零力附近波
        KW_FT.virtualStaticFrictionProcess(pt.actualStatus.force,KW_FT.staticFrictionFT);
        //可以进一步考虑自适应参数的导纳控制
        //pt.dragAdmArgsDynamicAdjust(vf.admArgs,vf.actualStatus.speed);
        //考虑速度控制
        /*vf.ctrlStatus.speed =
        controllor.admittance_cart(vf.admArgs,
          vf.targetStatus.pose - vf.targetStatus.pose,
          actualFlangeSpeed - vf.targetVel,
          vf.ctrlStatus.speed - vf.targetStatus.speed,
          vf.actualStatus.force);   //看起来这里没有这句命令的必要，因为在下一句命令就会将其覆盖
        */
        //经过两次积分，求得控制输出量
        pt.ctrlStatus = getControllOutput(pt.M, pt.B, pt.K, pt.targetStatus, pt.actualStatus);
        //将转动自由度锁定
        pt.ctrlStatus.speed[3] = 0;
        pt.ctrlStatus.speed[4] = 0;
        pt.ctrlStatus.speed[5] = 0;
        //这里考虑使用速度控制
        //URMove.sendSpeedLMsg(urConParPub, pt.ctrlStatus.speed);
        //此外，vf.ctrlStatus中已经存储了对位置的控制量，这里的积分操作是不是多余了？
        tragetPosition = pt.ctrlStatus.pose.block<3,1>(0,0) + pt.targetStatus.pose.block<3,1>(3,0) - pt.targetStatus.speed.block<3,1>(3,0) / CTRL_FREQ;  
        tragetRotationMatrix = RPY_rotationMatrix(pt.ctrlStatus.pose.block<3,1>(3,0)) * quaternion_rotationMatrix(rotationVector_quaternion(pt.targetStatus.pose.block<3,1>(3,0)) * quaternion_rotationMatrix(rotationVector_quaternion(pt.targetStatus.speed.block<3,1>(3,0)) / CTRL_FREQ);
        tragetPosture = quaternion_rotationVector(euler_quaternion(rotationMatrix_euler(tragetRotationMatrix)));
        tragetPosture << 0, 3.1415926, 0;
        pt.ctrlStatus.pose << tragetPosition, tragetPosture;

        URMove.sendPoseMsg(urConParPub, pt.ctrlStatus.pose);
        //std::cout << vf.ctrlStatus.pose.transpose()<<  std::endl;
        //如果要做拖动实验，是不是可以不开示教模式？
        //URMove.sendteachMsg(urConParPub, true);
        loop_rate.sleep();
    }
    return 0;
}