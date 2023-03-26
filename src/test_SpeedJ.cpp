/*#include <ur_rtde/rtde_control_interface.h>
#include <thread>
#include <chrono>
#include <math.h>
#include <mutex>
#include <ros/ros.h>
#include <ur_move.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <hrc/RobotControl.h>
#include <config.h>
#include <virtual_fixture.h>
#include <record_node.h>
#include "generic_api.h"

using namespace ur_rtde;
using namespace std::chrono;

int main(int argc, char* argv[])
{
  RTDEControlInterface rtde_control("192.168.0.104");

  // Parameters
  double acceleration = 0.5;
  double dt = 1.0/500; // 2ms
  std::vector<double> joint_q = {-1.54, -1.83, -2.28, -0.59, 1.60, 0.023};
  std::vector<double> joint_speed = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  // Move to initial joint position with a regular moveJ
  rtde_control.moveJ(joint_q);

  // Execute 500Hz control loop for 2 seconds, each cycle is ~2ms
  for (unsigned int i=0; i<1000; i++)
  {
    auto t_start = high_resolution_clock::now();
    rtde_control.speedJ(joint_speed, acceleration, dt);
    joint_speed[0] += 0.0005;
    joint_speed[1] += 0.0005;
    auto t_stop = high_resolution_clock::now();
    auto t_duration = std::chrono::duration<double>(t_stop - t_start);

    if (t_duration.count() < dt)
    {
      std::this_thread::sleep_for(std::chrono::duration<double>(dt - t_duration.count()));
    }
  }

  rtde_control.speedStop();
  rtde_control.stopScript();

  return 0;
}*/

#include <ur_rtde/rtde_control_interface.h>
#include <thread>
#include <chrono>

using namespace ur_rtde;
using namespace std::chrono;

int main(int argc, char* argv[])
{
  RTDEControlInterface rtde_control("192.168.0.104");

  // Parameters
  double velocity = 0.5;
  double acceleration = 0.5;
  double dt = 1.0/500; // 2ms
  double lookahead_time = 0.1;
  double gain = 300;
  std::vector<double> joint_q = {-1.54, -1.83, -2.28, -0.59, 1.60, 0.023};

  // Move to initial joint position with a regular moveJ
  rtde_control.moveJ(joint_q);

  // Execute 500Hz control loop for 2 seconds, each cycle is ~2ms
  for (unsigned int i=0; i<1000; i++)
  {
    auto t_start = high_resolution_clock::now();
    rtde_control.servoJ(joint_q, velocity, acceleration, dt, lookahead_time, gain);
    joint_q[0] += 0.001;
    //joint_q[1] += 0.001;
    auto t_stop = high_resolution_clock::now();
    auto t_duration = std::chrono::duration<double>(t_stop - t_start);

    if (t_duration.count() < dt)
    {
      std::this_thread::sleep_for(std::chrono::duration<double>(dt - t_duration.count()));
    }
  }

  rtde_control.servoStop();
  rtde_control.stopScript();

  return 0;
}