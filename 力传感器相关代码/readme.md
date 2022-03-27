## kw_ft_sensor 为坤维力传感器ros包
## hrc 里有更多的力传感器使用算法。
## 论文为力传感器工件重力参数补偿
## 如果不改变机器人姿态，不用看hrc包


hrc_gtc_main.cpp 为工件重力补偿主程序
process_ft_data.cpp/h 为处理力传感器数据的相关程序
keyboard_detection.cpp/h 为键盘按键检测程序（hrc_gtc_main）有用到

其他文件为采集/驱动ur机器人代码，为保持ros包完整性，一并交付，但不要随便运行或外传
