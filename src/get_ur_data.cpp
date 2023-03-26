#include <thread>
#include <chrono>
#include <ros/ros.h>
#include <virtual_fixture.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_control_interface.h>
#include <hrc/RobotData.h>
#include <config.h>
#include <stdio.h>
#include <stdlib.h>
#include<cstdlib>//注意。itoa函数要包含这个头文件

using namespace ur_rtde;
using namespace std::chrono;

class  UR_Data{
 private:
 RTDEReceiveInterface rtdeReceive;
 hrc::RobotData urData;
 public:
 UR_Data(std::string IPAdress = "127.0.0.1"):rtdeReceive(IPAdress)
 {}
 
 
 hrc::RobotData get_Parmater(void)
 {
   urData.actualTCPPose = rtdeReceive.getActualTCPPose();
   urData.targetTCPPose = rtdeReceive.getTargetTCPPose();
   urData.actualTCPSpeed = rtdeReceive.getActualTCPSpeed();
   urData.targetTCPSpeed = rtdeReceive.getTargetTCPSpeed();
   urData.actualTCPForce = rtdeReceive.getActualTCPForce();
   urData.actualToolAccel = rtdeReceive.getActualToolAccelerometer();
   urData.digitalInputBits = rtdeReceive.getActualDigitalInputBits();
//    printf("urData.actualTCPSpeed = %2f\n",urData.actualTCPSpeed[0]);
   return urData;
 }
 void publishMsg(ros::Publisher publisher, hrc::RobotData msg)
 {
   msg.header.stamp = ros::Time::now();
   publisher.publish(msg);
 }
};


int main(int argc, char **argv)
{
#if USE_UR_SIM
 UR_Data urRight("127.0.0.1");
#else
 UR_Data urRight("192.168.0.104");
#endif
 ros::init(argc, argv, "ur_data");

 ros::NodeHandle n;

 ros::Publisher urDataPub = n.advertise<hrc::RobotData>("ur_data", 1);

 ros::Rate loop_rate(128);

 while(ros::ok())
 {
   urRight.publishMsg(urDataPub, urRight.get_Parmater());
   loop_rate.sleep();
 }
 return 0;
}


