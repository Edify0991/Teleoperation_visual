#include <unistd.h>
#include <iostream>
#include <mutex>
#include <vector>
#include <deque>
#include <math.h>

#include <Eigen/Dense>

#include <geometry_msgs/WrenchStamped.h>
#include <config.h>
#include <process_ft_data.h>
#include <process_robot_data.h>
#include <record_node.h>
#include <hrc/ProcessedFTData.h>
#include "generic_api.h"

//using namespace std;


Process_FT_Data::Process_FT_Data(ros::NodeHandle& node_handle, const std::string sub_name, const std::string pub_name, std::string toolName): subName(sub_name),pubName(pub_name),n(node_handle)
{
  sub = n.subscribe<geometry_msgs::WrenchStamped>(subName, 100, boost::bind(&Process_FT_Data::ftDataCallback, this, _1));
  pub = n.advertise<hrc::ProcessedFTData>(pubName, 100);


  std::vector<std::string> toolName_vector(TOOLNAME_LIST);
  std::vector<double> G_vector(G_LIST);
  std::vector<double> U_vector(U_LIST);
  std::vector<double> V_vector(V_LIST);
  std::vector<Eigen::Matrix<double,3,1>> F0_vector(F0_LIST);
  std::vector<Eigen::Matrix<double,3,1>> T0_vector(T0_LIST);
  std::vector<Eigen::Matrix<double,3,1>> P_vector(P_LIST);
  std::vector<Eigen::Matrix<double,3,1>> A_vector(A_LIST);
  std::pair<std::map<std::string, Gravity_Compensation_Args>::iterator, bool> Insert_Pair;

  Gravity_Compensation_Args tmpGCArgs;

  for(unsigned long i=0; i != toolName_vector.size(); i++){
    tmpGCArgs.G = G_vector[i];
    tmpGCArgs.U = U_vector[i];
    tmpGCArgs.V = V_vector[i];
    tmpGCArgs.F0 = F0_vector[i];
    tmpGCArgs.T0 = T0_vector[i];
    tmpGCArgs.P = P_vector[i];
    tmpGCArgs.A = A_vector[i];
    GCArgsMap.insert(std::map<std::string, Gravity_Compensation_Args>::value_type (toolName_vector[i], tmpGCArgs));
  }
  auto iter = GCArgsMap.find(toolName);

  if(iter != GCArgsMap.end()){
    GCArgs = iter->second;
    std::cout<<"重力补偿参数为："<<iter->first<<std::endl;
  }
  else{
    std::cout<<"重力补偿参数设置错误!"<<std::endl;
  }
}

void Process_FT_Data::ftDataCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  std::lock_guard<std::mutex> some_guard(ft_mutex);

  Eigen::Matrix<double,6,1> tmpFT;
  tmpFT << msg->wrench.force.x,
      msg->wrench.force.y,
      msg->wrench.force.z,
      msg->wrench.torque.x,
      msg->wrench.torque.y,
      msg->wrench.torque.z;

  if(origen_ft.size() >= FTDataBuffer){
    origen_ft.pop_front();
    filtered_ft.pop_front();
    origen_gtced_ft.pop_front();
    filtered_gtced_ft.pop_front();
    origen_world.pop_front();
    filtered_world.pop_front();
    origen_gtced_world.pop_front();
    filtered_gtced_world.pop_front();
  }
  origen_ft.push_back(tmpFT);
  //力传感器滤波
  tmpFT = filter(origen_ft,filtered_ft);
  filtered_ft.push_back(tmpFT);

  //
  tmpFT = gravityCompensation(GCArgs, transM_P->R_robot_flange*transM_P->R_flange_ft);
  origen_gtced_ft.push_back(origen_ft.back() - tmpFT);
  filtered_gtced_ft.push_back(filtered_ft.back() - tmpFT);

  origen_world.push_back(transFT_World(origen_ft.back(),transM_P));
  filtered_world.push_back(transFT_World(filtered_ft.back(),transM_P));
  origen_gtced_world.push_back(transFT_World(origen_gtced_ft.back(),transM_P));
  filtered_gtced_world.push_back(transFT_World(filtered_gtced_ft.back(),transM_P));

  pubFTMsg.origen_ft.resize(origen_ft.back().size());
  Eigen::Map<Eigen::Matrix<double,6,1>>(&pubFTMsg.origen_ft[0],origen_ft.back().size()) = origen_ft.back();

  pubFTMsg.filtered_ft.resize(filtered_ft.back().size());
  Eigen::Map<Eigen::Matrix<double,6,1>>(&pubFTMsg.filtered_ft[0],filtered_ft.back().size()) = filtered_ft.back();

  pubFTMsg.origen_gtced_ft.resize(origen_gtced_ft.back().size());
  Eigen::Map<Eigen::Matrix<double,6,1>>(&pubFTMsg.origen_gtced_ft[0],origen_gtced_ft.back().size()) = origen_gtced_ft.back();

  pubFTMsg.filtered_gtced_ft.resize(filtered_gtced_ft.back().size());
  Eigen::Map<Eigen::Matrix<double,6,1>>(&pubFTMsg.filtered_gtced_ft[0],filtered_gtced_ft.back().size()) = filtered_gtced_ft.back();

  pubFTMsg.origen_world.resize(origen_world.back().size());
  Eigen::Map<Eigen::Matrix<double,6,1>>(&pubFTMsg.origen_world[0],origen_world.back().size()) = origen_world.back();

  pubFTMsg.filtered_world.resize(filtered_world.back().size());
  Eigen::Map<Eigen::Matrix<double,6,1>>(&pubFTMsg.filtered_world[0],filtered_world.back().size()) = filtered_world.back();

  pubFTMsg.origen_gtced_world.resize(origen_gtced_world.back().size());
  Eigen::Map<Eigen::Matrix<double,6,1>>(&pubFTMsg.origen_gtced_world[0],origen_gtced_world.back().size()) = origen_gtced_world.back();

  pubFTMsg.filtered_gtced_world.resize(filtered_gtced_world.back().size());
  Eigen::Map<Eigen::Matrix<double,6,1>>(&pubFTMsg.filtered_gtced_world[0],filtered_gtced_world.back().size()) = filtered_gtced_world.back();
  pubFTMsg.header.stamp = ros::Time::now();
  pub.publish(pubFTMsg);
  //  std::cout <<"origen_ft.back()"<< origen_ft.back().transpose()<<std::endl;
  //  std::cout <<"origen_gtced_ft.back()"<< origen_gtced_ft.back().transpose()<<std::endl<<std::endl;
}


Eigen::Matrix<double,6,1> Process_FT_Data::transFT_World(const Eigen::Matrix<double,6,1>& FTData, Transfer_Matrix* T)
{
  Eigen::Matrix<double,6,1> tmpFT;
  tmpFT << T->R_world_robot * T->R_robot_flange * T->R_flange_ft * FTData.block<3, 1>(0, 0),
      T->R_world_robot * T->R_robot_flange * T->R_flange_ft * FTData.block<3, 1>(3, 0);
  return tmpFT;
}


std::deque<Eigen::Matrix<double,6,1>> Process_FT_Data::getFTDeque(const char *FTType)
{
  std::lock_guard<std::mutex> some_guard(ft_mutex);
  std::deque<Eigen::Matrix<double,6,1>> tmpFT;

  switch(hashStrUint64(FTType)){
  case hashStrUint64("origen_ft"):
    tmpFT = origen_ft;
    break;
  case hashStrUint64("filtered_ft"):
    tmpFT = filtered_ft;
    break;
  case hashStrUint64("origen_gtced_ft"):
    tmpFT = origen_gtced_ft;
    break;
  case hashStrUint64("filtered_gtced_ft"):
    tmpFT = filtered_gtced_ft;
    break;
  case hashStrUint64("origen_world"):
    tmpFT = origen_world;
    break;
  case hashStrUint64("filtered_world"):
    tmpFT = filtered_world;
    break;
  case hashStrUint64("origen_gtced_world"):
    tmpFT = origen_gtced_world;
    break;
  case hashStrUint64("filtered_gtced_world"):
    tmpFT = filtered_gtced_world;
    break;
  default:
    printf("getFTDeque函数错误输入\n");
    break;
  }
  return tmpFT;
}

Eigen::Matrix<double,6,1> Process_FT_Data::getFTData(const char *FTType)
{
  std::lock_guard<std::mutex> some_guard(ft_mutex);
  Eigen::Matrix<double,6,1> tmpFT;
  switch(hashStrUint64(FTType)){
  case hashStrUint64("origen_ft"):
    tmpFT = origen_ft.back();
    break;
  case hashStrUint64("filtered_ft"):
    tmpFT = filtered_ft.back();
    break;
  case hashStrUint64("origen_gtced_ft"):
    tmpFT = origen_gtced_ft.back();
    break;
  case hashStrUint64("filtered_gtced_ft"):
    tmpFT = filtered_gtced_ft.back();
    break;
  case hashStrUint64("origen_world"):
    tmpFT = origen_world.back();
    break;
  case hashStrUint64("filtered_world"):
    tmpFT = filtered_world.back();
    break;
  case hashStrUint64("origen_gtced_world"):
    tmpFT = origen_gtced_world.back();
    break;
  case hashStrUint64("filtered_gtced_world"):
    tmpFT = filtered_gtced_world.back();
    break;
  default:
    printf("getFT函数错误输入\n");
    break;
  }
  return tmpFT;
}

//工具,力传感器重力补偿
//返回重力补偿值，用当前力传感器值减去重力补偿值即可得到重力补偿后的值
Eigen::Matrix<double,6,1> Process_FT_Data::gravityCompensation(
    const Gravity_Compensation_Args &GCArgs,
    const Eigen::Matrix<double,3,3> R_robot_ft)
{
  Eigen::Matrix<double,6,1> FTc;
  Eigen::Matrix<double,3,3> rotationMatrix;
  Eigen::Matrix<double,3,1> gx, mgx;
  FTc = Eigen::Matrix<double,6,1>::Zero();

  gx = R_robot_ft.transpose() * GCArgs.A;
  mgx << gx(2) * GCArgs.P(1) - gx(1) * GCArgs.P(2),
      gx(0) * GCArgs.P(2) - gx(2) * GCArgs.P(0),
      gx(1) * GCArgs.P(0) - gx(0) * GCArgs.P(1);
  FTc.block<3,1>(0,0) = GCArgs.F0 + gx;
  FTc.block<3,1>(3,0) = GCArgs.T0 + mgx;
  return FTc;
}

//设置重力补偿参数
void Process_FT_Data::setGCArgs(std::string toolName)
{
  auto iter = GCArgsMap.find(toolName);

  if(iter != GCArgsMap.end()){
    GCArgs = iter->second;
    std::cout<<"重力补偿参数为："<<iter->first<<std::endl;
  }
  else{
    std::cout<<"重力补偿参数设置错误!"<<std::endl;
    ros::shutdown();
  }
}



//分析力传感器数据队列中的数据
FT_Data_Analyze Process_FT_Data::analyzeFTDataQueue(const char *FTType){
  Eigen::Matrix<double,6,1> sum, accum;
  sum = accum = Eigen::MatrixXd::Zero(6,1);
  FT_Data_Analyze Args;

  Args.Deque = getFTDeque(FTType);


  std::for_each(std::begin(Args.Deque), std::end(Args.Deque), [&](const Eigen::Matrix<double,6,1> data){
    sum += data;
  });
  Args.mean = sum / Args.Deque.size();

  std::for_each(std::begin(Args.Deque), std::end(Args.Deque), [&](const Eigen::Matrix<double,6,1> data){
    accum += ((data - Args.mean).array() * (data - Args.mean).array()).matrix();
  });
  Args.stdev = sqrt(accum.array()/(Args.mean.size()-1)).matrix();
  return Args;
}

//校准力传感器
bool Process_FT_Data::setFTDateZeroError(const double maxStdevSum)
{
  std::lock_guard<std::mutex> some_guard(ft_mutex);
  std::deque<Eigen::Matrix<double,6,1>> tmpFTDeque;
  Eigen::Matrix<double,6,1> sum, mean, accum, stdev;
  sum = mean = accum = stdev = Eigen::MatrixXd::Zero(6,1);
  double stdevSum = 0;

  if(FTNeedCalibration == false){
    printf("FT sensor data don't need to Calibrate. \n");
    return true;
  }

  std::for_each(std::begin(origen_gtced_ft), std::end(origen_gtced_ft), [&](const Eigen::Matrix<double,6,1> data){
    sum += data;
  });

  mean = sum / origen_gtced_ft.size();

  std::for_each(std::begin(origen_gtced_ft), std::end(origen_gtced_ft), [&](const Eigen::Matrix<double,6,1> data){
    accum += ((data - mean).array() * (data - mean).array()).matrix();
  });
  stdev = sqrt(accum.array()/(origen_gtced_ft.size()-1)).matrix();
  stdevSum = stdev.rowwise().sum()(0,0);

  if(stdevSum < maxStdevSum){
    GCArgs.F0 += mean.block<3,1>(0,0);
    GCArgs.T0 += mean.block<3,1>(3,0);
    printf("FT sensor data have been Calibrated. \n");
    std::cout << "FT Date Zero Error = " << mean.transpose() << std::endl;
    FTNeedCalibration = false;
    return true;
  }else{
    printf("Stdev is too large to Calibrate FT sensor data. Stdev = %f \n",stdevSum);
    return false;
  }
}

//力传感器滤波
Eigen::Matrix<double,6,1> Process_FT_Data::filter(
    const std::deque<Eigen::Matrix<double,6,1>> &origenData,
    const std::deque<Eigen::Matrix<double,6,1>> &processedData,
    const std::vector<double> a,
    const std::vector<double> b)
{
  Eigen::Matrix<double,6,1> tmpFT;
  tmpFT = Eigen::MatrixXd::Zero(6,1);
  if(origenData.size() < a.size() || processedData.size() < b.size() - 1 ){
    return tmpFT;
  }

  for(unsigned long i = 0; i < b.size(); i++){
    tmpFT += *(b.begin() + i) * *(origenData.end() - i - 1);
  }

  if(a.size()>1){
    for(unsigned long i = 1; i < a.size(); i++){
      tmpFT -= *(a.begin() + i) * *(processedData.end() - i);
    }
  }
  return tmpFT;
}

//虚拟静摩擦力 消除力动传感器零力附近波
Eigen::Matrix<double,6,1> Process_FT_Data::virtualStaticFrictionProcess(
    Eigen::Matrix<double,6,1> &ftData,
    Eigen::Matrix<double,6,1> frictionFT)
{
  for (int i = 0;i<6;i++){
    if(ftData[i] > frictionFT[i]){
      ftData[i] = ftData[i] - frictionFT[i];
    }else if(ftData[i] < -frictionFT[i]){
      ftData[i] = ftData[i] + frictionFT[i];
    }else{
      ftData[i] = 0;
    }
  }
  return ftData;
}
