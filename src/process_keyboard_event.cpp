#include "process_keyboard_event.h"
#include "hrc/KeyValue.h"
#include <ros/ros.h>


Process_Keyboard_Event::Process_Keyboard_Event(ros::NodeHandle& node_handle, const std::string sub_name): subName(sub_name),n(node_handle)
{
 sub = n.subscribe<hrc::KeyValue>(subName, 100, boost::bind(&Process_Keyboard_Event::ketValueDataCallback, this, _1));
}

void Process_Keyboard_Event::ketValueDataCallback(const hrc::KeyValue::ConstPtr& msg)
{
  if(newValue != true)
  {
    keyValue.clear();
    keyChar.clear();
  }
  std::lock_guard<std::mutex> some_guard(key_mutex);
  keyValue.push_back(msg->keyValue);
  keyChar.push_back(msg->keyChar);
  newValue = true;
}
