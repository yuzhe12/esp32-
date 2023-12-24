#ifndef __MY_SENSOR_H__
#define __MY_SENSOR_H__
 
#include <Wire.h>
#include <stdio.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
#include <Math.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "ArduinoJson.h"
#include "AsyncUDP.h"
#include "My_button2.h"

//下面部分为传感器使用类，可调用的函数内容如下，具体函数内容请前往sendData.cpp
class My_Sensor//设置传感器类
{
public:
	My_Sensor()//初始化变量
	{
		roll = 0;   
		pitch = 0;  
		yaw = 0;    
		//euler = 0;
		accx = 0;
    accy = 0;
    accz = 0;
    gyrox = 0;
    gyroy = 0;
    gyroz = 0;
    magx = 0;
    magy = 0;
    magz = 0;
		is_cali = 0;
	}

  // 外部调用函数
	void initSensor();  // 传感器初始化
  void getData(String name, int port);  // 读取传感器数据 处理传感器数据 发送传感器数据

  AsyncUDP udpClient;  // 创建用于udp协议发送数据的变量
  
private:
  //传感器各数值变量
	float roll;     // 翻滚量存储变量
	float pitch;    // 俯仰角存储变量
	float yaw;      // 偏航角存储变量
	//float euler;    // 欧拉角存储变量
	float accx;      // 传感器加速度数据存储变量
  float accy;      // 传感器加速度数据存储变量
  float accz;      // 传感器加速度数据存储变量
  float gyrox;      // 传感器角速度数据存储变量
  float gyroy;      // 传感器角速度数据存储变量
  float gyroz;      // 传感器角速度数据存储变量
  float magx;
  float magy;
  float magz;
	int is_cali;    // 传感器校准比较变量
	String json;    // 存储传感器获得的各种信息并打包为json格式的变量
	//String json2; // 暂时未启用

  //私有函数声明
  String GetInString(String sensorName, int temp, int system);  // 获得传感器温度 校准状态数据 用于测试
  String GetJSONString(String sensorName, float accx, float accy, float accz, float gyrox, float gyroy, float gyroz, float magx, float magy, float magz, int system) ;  // 将读取到的运动数据按照json格式存放，格式为{"n":"name","o":{"x":"0.0301","y":"0.3863","z":"-0.7606","w":"-0.5209","euler":"23.23","acc":"9.80"p":"11"}}
  void UDPSendData(String message, String ip, int port);  // 通过udp协议发送数据，参数： string message: 需要发送的数据   string ip: ip  string port: 端口
  void UDPConnect();  // UDP连接
  //void connectToWiFi ();  // 连接wifi
  float readPower();  // 读取 GPIO 引脚的模拟值,并转换为电压值
};


#endif