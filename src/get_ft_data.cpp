#include  <stdio.h>
#include  <stdlib.h>
#include  <ros/ros.h>
#include  <unistd.h>
#include  <sys/types.h>
#include  <sys/signal.h>
#include  <sys/stat.h>
#include  <fcntl.h>
#include  <termios.h>
#include  <errno.h>
#include  <limits.h>
#include  <string.h>
#include  <time.h>
#include <termios.h>
#include  <config.h>
#include <Eigen/Dense>
#include  <deque>
#include  <geometry_msgs/WrenchStamped.h>

using namespace std;

class FT_Sensor
{
private:
  char* serialName;
  int Serialhandle;
  double G = 9.8;
  std::deque<unsigned char> receiveddata;
public:
  geometry_msgs::WrenchStamped ft_data;
  FT_Sensor(char* getserialName = const_cast<char*>("/dev/ttyUSB0")):serialName(getserialName)
  {
    struct termios OnesensorTermios_old;
    struct termios OnesensorTermios;
    Serialhandle = open(serialName, O_RDWR | O_NOCTTY |O_NONBLOCK);
    if(Serialhandle<0)
    {
      printf("Open serial port /dev/ttyUSB0 failed!\n");
    }
    tcgetattr(Serialhandle , &OnesensorTermios_old);
    bzero( &OnesensorTermios, sizeof(OnesensorTermios));
    cfmakeraw(&OnesensorTermios);
    OnesensorTermios.c_cflag=B460800;
    OnesensorTermios.c_cflag |= CLOCAL | CREAD;
    OnesensorTermios.c_cflag &= ~CSIZE;
    OnesensorTermios.c_cflag |= CS8;
    OnesensorTermios.c_cflag &= ~PARENB;
    OnesensorTermios.c_cflag &= ~CSTOPB;
    tcflush(Serialhandle,TCIFLUSH);
    tcflush(Serialhandle,TCOFLUSH);
    OnesensorTermios.c_cc[VTIME] = 1;
    OnesensorTermios.c_cc[VMIN] = 1;
    tcflush (Serialhandle, TCIFLUSH);
    tcsetattr(Serialhandle,TCSANOW,&OnesensorTermios);
    //wirte command
    char writedata[10]= {0};
    writedata[0] = 0x48;
    writedata[1] = 0xAA;
    writedata[2] = 0x0D;
    writedata[3] = 0x0A;
    int len = 0, total_len = 0;
    for (total_len = 0 ; total_len < 4;)
    {
      len = 0;
      len = static_cast<int>(write(Serialhandle, &writedata[total_len], 4 - total_len));
      //          printf("WriteData fd = %d ,len =%d,data = %s\n",Serialhandle,len,writedata);
      if (len > 0)
      {
        total_len += len;
      }
      else if(len <= 0)
      {
        len = -1;
        break;
      }
    }
    if(len < 0){
      printf("Write Data Fail!\n");
    }
    else{
      printf("Successful KW_FT sensor initialization.\n");
    }
  }

  int get_ft_data()
  {
    unsigned char readdata[150];
    memset(readdata,0,150);//the length//
    int max_fd = 0;
    fd_set readset = {0};
    struct timeval tv = {0,0};
    int len, i;
    len = 0;

    FD_ZERO(&readset);
    FD_SET(static_cast<unsigned int>(Serialhandle), &readset);
    max_fd = Serialhandle + 1;
    tv.tv_sec=0;
    tv.tv_usec=0;
    if (select(max_fd, &readset, nullptr, nullptr, &tv ) < 0)
    {
      printf("ReadData: select error\n");
      return -1;
    }
    int nRet = FD_ISSET(Serialhandle, &readset);
    if (nRet)
    {
      len = static_cast<int>(read(Serialhandle, readdata, 150));
    }
    if( len > 0 )
    {
      for(i=0;i<len;i++)
        receiveddata.push_back(readdata[i]);
      //for(int i=0;i<m_rec_data_length;i++)
      //printf(" %d ",m_rec_data[i]);
      // printf("\n");
    }
    return len;
  }
  int deal_ft_data()
  {
    unsigned char data[28];
    union xxx{char m[4]; float n;}z;
    int i;
    unsigned long ReceivedDataLangth;

    while(receiveddata.size()>0)
    {
      ReceivedDataLangth = receiveddata.size();  //缓存目前收到数据的长度，以免循环过程中有新数据写入或读出影响操作
      if((ReceivedDataLangth>=28)&&(receiveddata.at(26) == 0x0d)&&(receiveddata.at(27) == 0x0a))
      {
        for(i =0;i<28;i++)
        {
          data[i] = receiveddata.front();
          receiveddata.pop_front();
        }
        for(i=0;i<4;i++)
        {
          z.m[i]=data[i+2];
        }
        ft_data.wrench.force.x=G * static_cast<double>(z.n);
        for(i=0;i<4;i++)
        {
          z.m[i]=data[i+6];
        }
        ft_data.wrench.force.y=G * static_cast<double>(z.n);
        for(i=0;i<4;i++)
        {
          z.m[i]=data[i+10];
        }
        ft_data.wrench.force.z=G * static_cast<double>(z.n);
        for(i=0;i<4;i++)
        {
          z.m[i]=data[i+14];
        }
        ft_data.wrench.torque.x=G * static_cast<double>(z.n);
        for(i=0;i<4;i++)
        {
          z.m[i]=data[i+18];
        }
        ft_data.wrench.torque.y=G * static_cast<double>(z.n);
        for(i=0;i<4;i++)
        {
          z.m[i]=data[i+22];
        }
        ft_data.wrench.torque.z=G * static_cast<double>(z.n);

        // printf("Fx= %2f Kg,Fy= %2f Kg,Fz= %2f Kg,Mx= %2f Kg/M,My= %2f Kg/M,Mz= %2f Kg/M\n",Force[0],Force[1],Force[2],Force[3],Force[4],Force[5]);
      }
      else if((ReceivedDataLangth>=28)&&(ReceivedDataLangth<120))
      {
        if(receiveddata.at(0) == 0x0a)
          receiveddata.pop_front();
        else
        {
          i=0;
          while ((i<=28)&&(receiveddata.at(0)!=0x0d)&&(receiveddata.at(1)!=0x0a))
          {
            receiveddata.pop_front();
            i++;
          }
          if(receiveddata.size()>=2)
          {
            receiveddata.pop_front();
            receiveddata.pop_front();
          }
        }
      }
      else if(ReceivedDataLangth >= 120)
        receiveddata.clear();
      else if(ReceivedDataLangth < 28)
        break;
    }
    return 0;
  }
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "ft_data");
  ros::NodeHandle n;

#if USE_FT_SIM
  geometry_msgs::WrenchStamped ft_data;
//  Eigen::Matrix<double,3,1> FValue = Eigen::MatrixXd::Zero(3,1);
#define cycleTime 1
#define maxForce  20
  int cycleNum = 0;
  ft_data.wrench.force.x = 0.0;
  ft_data.wrench.force.y = 0.0;
  ft_data.wrench.force.z = 0.0;
  ft_data.wrench.torque.x = 0.0;
  ft_data.wrench.torque.y = 0.0;
  ft_data.wrench.torque.z = 0.0;

#else
  std::string tmpSerialNum;
  char *serialNum;
  n.getParam("serialNum", tmpSerialNum);
  serialNum = const_cast<char*>(tmpSerialNum.c_str());
  FT_Sensor ft_sensor(serialNum);

#endif

  ros::Publisher chatter_pub = n.advertise<geometry_msgs::WrenchStamped>("ft_data", 100);

  ros::Rate loop_rate(960);

  while (ros::ok())
  {
#if USE_FT_SIM
//    ft_data.wrench.force.z = maxForce * sin(2 * M_PI * (cycleNum++ % 960) / (960 * cycleTime));
//    ft_data.wrench.force.z = 20.0;
//    printf("ft_data.wrench.force.x = %2f \n",ft_data.wrench.force.x);
    ft_data.header.stamp = ros::Time::now();
    chatter_pub.publish(ft_data);
#else
    ft_sensor.get_ft_data();
    ft_sensor.deal_ft_data();
    ft_sensor.ft_data.header.stamp = ros::Time::now();
    chatter_pub.publish(ft_sensor.ft_data);
#endif
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

//w:119, s115, a97, d100, z122, x120;
