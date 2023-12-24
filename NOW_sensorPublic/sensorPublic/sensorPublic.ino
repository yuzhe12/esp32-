#include "Arduino.h"
#include "WiFiUser.h"
#include "My_Sensor.h"
#include "My_time.h"
#include "My_button2.h"

My_Sensor my_sensor;                       // 创建一个传感器变量
int port;                                  // 设置默认的udp传输发送端口
String imu_name = "imu-1";
//RTC_DATA_ATTR int bootCount = 0;        // 用于记录关机开机的次数 存放在rtc-data-attr中
//String name;
const char *NVS_Name = "myNVS";//创建一个nvs键空间名字为myNVS
extern String MySendName; 
void setup() {
    Serial.begin(115200);  // 初始化串口通信 设置波特率为115200  
    my_sensor.initSensor();  // 初始化传感器
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_33,0);  // 设置唤醒设置关机->开机 （从低功耗进入正常工作模式）；唤醒引脚为io33 ；当1 = High或 0 = Low 唤醒
    Init_timeset();  // 初始化定时器设置
    pinMode(LED, OUTPUT);  // ledio设置为输出

    /************************************
      Init_KeyDetectionSet()函数注释：
      1.长按，进入低功耗模式（待机）
      2.短按，打印11，状态标志位power = 1 
      3.双击，清空网络信息，重新配网
    ************************************/
    Init_KeyDetectionSet();  // 按键检测初始化设置

    digitalWrite(LED, HIGH);  // 开机 开机指示灯亮 
    power = 1;  // 状态位置1
    inter = 0;

    // 用于电池电量读取 未实现
     pinMode(12, OUTPUT);
     digitalWrite(12, HIGH);
     pinMode(34, INPUT);
     
    NVSINIT(NVS_Name);//初始化创建一个nvs键空间的名称
    // 记录开关机次数  用于测试
    // ++bootCount;    
    // Serial.println("Boot number: " + String(bootCount));

    //打印ESP32的唤醒原因
    // print_wakeup_reason();     
}

void loop() {
  // 启动按钮状态检测
  button.loop();
  // 处于开机状态
  if (power == 1)
  {
    digitalWrite(LED, HIGH);            // 状态指示灯常亮
    checkDNS_HTTP();                    // 检测客户端DNS&HTTP请求，也就是检查配网页面那部分
    checkConnect(true);                 // 检测网络连接状态，参数true表示如果断开重新连接
    // name = getSensorName();
    if(flag)//判断定时器中断标志位是否触发
  {
    port = getSensorPort();             // 获得配网时输入的端口
    // if(name == "") name = "sensor"; 
    if(port == 0) port = 8080;          // 若没输入 则默认为8080
    if(flag_Rate == true)
    {
      Rerate();
    }
    my_sensor.getData(MySendName, port);  // 发送数据
    flag = false;//复原标志位
  }
  }
  
}






