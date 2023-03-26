#ifndef PROCESS_FT_DATA_H
#define PROCESS_FT_DATA_H
#include <iostream>
#include <mutex>
#include <vector>
#include <deque>
#include <map>
#include <math.h>
#include <Eigen/Dense>
#include <geometry_msgs/WrenchStamped.h>
#include <config.h>
#include <process_robot_data.h>
#include <hrc/ProcessedFTData.h>

extern Transfer_Matrix *transM_P;

typedef struct
{
  double G = 0;
  double U = 0;
  double V = 0;
  Eigen::Matrix<double,3,1> F0 = {0,0,0};
  Eigen::Matrix<double,3,1> T0 = {0,0,0};
  Eigen::Matrix<double,3,1> P = {0,0,0};
  Eigen::Matrix<double,3,1> A = {0,0,0};
}Gravity_Compensation_Args;

typedef struct
{
  Eigen::Matrix<double,6,1> mean;
  Eigen::Matrix<double,6,1> stdev;
  std::deque<Eigen::Matrix<double,6,1>> Deque;
}FT_Data_Analyze;

class Process_FT_Data
{
private:
  std::mutex ft_mutex;
  std::deque<Eigen::Matrix<double,6,1>> origen_ft;
  std::deque<Eigen::Matrix<double,6,1>> filtered_ft;
  std::deque<Eigen::Matrix<double,6,1>> origen_gtced_ft;
  std::deque<Eigen::Matrix<double,6,1>> filtered_gtced_ft;
  std::deque<Eigen::Matrix<double,6,1>> origen_world;
  std::deque<Eigen::Matrix<double,6,1>> filtered_world;
  std::deque<Eigen::Matrix<double,6,1>> origen_gtced_world;
  std::deque<Eigen::Matrix<double,6,1>> filtered_gtced_world;

public:
  const unsigned long FTDataBuffer = 384;

  bool FTNeedCalibration = true;
  std::string subName;
  std::string pubName;
  ros::NodeHandle n;
  ros::Subscriber sub;
  ros::Publisher pub;
  hrc::ProcessedFTData pubFTMsg;
  std::map<std::string,Gravity_Compensation_Args> GCArgsMap;
  Gravity_Compensation_Args GCArgs;
  Eigen::Matrix<double,6,1> staticFrictionFT = (Eigen::MatrixXd(6,1) << 1.0,1.0,1.0,0.1,0.1,0.1).finished();

  Process_FT_Data(ros::NodeHandle& node_handle, const std::string sub_name, const std::string pub_name, std::string toolName);

  void ftDataCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg);

  Eigen::Matrix<double,6,1> transFT_World(const Eigen::Matrix<double,6,1>& FTData, Transfer_Matrix* T);

  std::deque<Eigen::Matrix<double,6,1>> getFTDeque(const char *FTType);
  Eigen::Matrix<double,6,1> getFTData(const char *FTType);

  //设置重力补偿参数
  void setGCArgs(std::string toolName);

  //分析力传感器数据队列中的数据
  FT_Data_Analyze analyzeFTDataQueue(const char *FTType);

  //校准力传感器
  bool setFTDateZeroError(const double maxStdevSum = 0.02);

  //力传感器滤波
  Eigen::Matrix<double,6,1> filter(
      const std::deque<Eigen::Matrix<double,6,1>> &origenData,
      const std::deque<Eigen::Matrix<double,6,1>> &processedData,
      const std::vector<double> a = FILTER_A,
      const std::vector<double> b = FILTER_B);

  //工具,力传感器重力补偿
  //返回重力补偿值，用当前力传感器值减去重力补偿值即可得到重力补偿后的值
  Eigen::Matrix<double,6,1> gravityCompensation(
      const Gravity_Compensation_Args &GCArgs,
      const Eigen::Matrix<double,3,3> R_robot_ft);

  //虚拟静摩擦力 消除力动传感器零力附近波
  Eigen::Matrix<double,6,1> virtualStaticFrictionProcess(
      Eigen::Matrix<double,6,1> &origenData,
      Eigen::Matrix<double,6,1> frictionFT);
};

#endif // PROCESS_FT_DATA_H
