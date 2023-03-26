#ifndef PROCESS_KEYBOARD_EVENT_H
#define PROCESS_KEYBOARD_EVENT_H
#include <mutex>
#include <ros/ros.h>
#include "hrc/KeyValue.h"

class Process_Keyboard_Event
{
private:
  std::string subName;
  ros::NodeHandle n;
  ros::Subscriber sub;
  std::mutex key_mutex;
  void ketValueDataCallback(const hrc::KeyValue::ConstPtr& msg);
public:
  std::vector<int> keyValue;
  std::vector<char> keyChar;
  bool newValue = false;

  Process_Keyboard_Event(ros::NodeHandle& node_handle, const std::string sub_name);
};




#endif // PROCESS_KEYBOARD_EVENT_H
