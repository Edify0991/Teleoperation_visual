#include <unistd.h>
#include "ros/ros.h"
#include <std_msgs/Char.h>
#include <termios.h>
#include <hrc/KeyValue.h>

#define KEY_0 48
#define KEY_A 97
/******************************************按键检测************************************************/
//按键扫描，阻塞型
int scanKeyboard()
{
int in;
struct termios new_settings;
struct termios stored_settings;
tcgetattr(0,&stored_settings);
new_settings = stored_settings;
new_settings.c_lflag &= (~ICANON);
new_settings.c_cc[VTIME] = 0;
tcgetattr(0,&stored_settings);
new_settings.c_cc[VMIN] = 1;
tcsetattr(0,TCSANOW,&new_settings);
in = getchar();
tcsetattr(0,TCSANOW,&stored_settings);
return in;
}
/*******************************************按键检测**********************************************/

int main(int argc, char **argv)
{
  int keyValue;
  ros::init(argc, argv, "keyboard_detection");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<hrc::KeyValue>("key_value", 1);

  ros::Rate loop_rate(128);
  hrc::KeyValue keyValueMsg;

  while(ros::ok())
  {
    keyValue = scanKeyboard();
//    printf("keyValue%d\n",keyValue);
    keyValueMsg.header.stamp = ros::Time::now();
    keyValueMsg.keyValue = keyValue;
    keyValueMsg.keyChar = static_cast<char>(keyValue);
    pub.publish(keyValueMsg);
    loop_rate.sleep();
  }
  return 0;
}
