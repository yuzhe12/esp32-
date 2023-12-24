#include "MY_Sensor.h"
#include "Ip_adress.h"
#define BNO055_DELAY_MS (0)

// WiFi Credentials
// #define WIFI_NETWORK "SZU NEWIFI2"
// #define WIFI_PASSWORD "12345678"
#define WIFI_TIMEOUT_MS 20000
extern String Wifi_ip;//声明Wifi_ip变量
unsigned char num1, num2, num3, num4;//设置将192.168.1.103的ip地址拆分出来的变量：num1=192；num2=168；num3 = 1； num2 = 103
// UDP Server Details
int udp_port = 8080;   // UDP接收端口
WiFiUDP udp;

Adafruit_BNO055 sensor = Adafruit_BNO055();  // 创建 sensor bno055对象  详情参照 adfruit_bno055 开源代码


/***************************************************************
  *  函数名称：My_Sensor::initSensor
  *  函数作用：初始化传感器
  *  输入参数：无   
  *  返 回 值：无
  *  函数的使用方法：在主函数中创建类名并直接调用函数，示例：My_Sensor my_sensor；//创建对象  my_sensor.initSensor(); //函数调用 
  *  日    期： 2023.8.29
 **************************************************************/
void My_Sensor::initSensor()
	{
    // put your setup code here, to run once:
    // Serial.begin(115200);
    // Connect to WiFI first
    // connectToWiFi ();
		sensor.begin();//打开传感器
    sensor.setMode(OPERATION_MODE_AMG);//setmode模式
		delay(500);//延迟秒等待传感器开启
		int8_t temp = sensor.getTemp();//获取传感器温度存入变量temp中
		sensor.setExtCrystalUse(true);//使用外部晶振
    
    // 电池电量检测的设置 未实现
    // pinMode(14, INPUT);
    // pinMode(12, OUTPUT);
    // digitalWrite(12, HIGH);
	}



  /***************************************************************
  *  函数名称：My_Sensor::getData
  *  函数作用：读取传感器四元数数据，处理传感器数据，以json格式upd传输方法发送传感器数据
  *  输入参数：name（传感器名称）  port（目标端口号）  
  *  返 回 值：无
  *  函数的使用方法：在主函数中创建类名传入参数并直接调用函数，示例：My_Sensor my_sensor；//创建对象  my_sensor.getData("szu001", port); //函数调用 
  *  日    期： 2023.8.29
 **************************************************************/
  void My_Sensor::getData(String name, int port) {

  // 将模拟值转换为电压值 未实现
  // float aaa = 0;
  // aaa = analogRead(14);
  // aaa = aaa * 3.3 / 4096;
 
  imu::Quaternion quat = sensor.getQuat();            // 获取四元数数据

  // 获得传感器校准状态 判断是否校准
  uint8_t system, gyro, accel, mg = 0;
  if(is_cali == 1){
    system = 3;
  }else{
    sensor.getCalibration(&system, &gyro, &accel, &mg); // 显示校准数据
    if(system > 2){
      system = 3;
      is_cali = 1;
    }
  }

  // 获得传感器欧拉角数据
  imu::Vector<3> euler_v = quat.toEuler();   // 转换为欧拉角
  yaw = euler_v.y() * 180/M_PI;              // 偏航
  pitch = euler_v.x() * 180/M_PI;            // 俯仰
  roll = euler_v.z() * 180/M_PI;             // 翻滚
  //euler = sqrt(yaw * yaw + pitch * pitch + roll * roll);  // 计算欧拉角

  
  // 获得传感器四元数数据
  quat.normalize ();                        // 归一化
  float w,x,y,z;
  w=quat.w();    
  x=quat.x();
  y=quat.y();
  z=quat.z();

  // 获得传感器加速度数据
  imu::Vector<3> acc_v = sensor.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);     // 获取加速度
  imu::Vector<3> gyro_v = sensor.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);     // 获取陀螺仪角速度
  imu::Vector<3> mag_v = sensor.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);     // 获取磁力计
  accx = acc_v.x();  //获取x轴加速度
  accy = acc_v.y();  //获取y轴加速度
  accz = acc_v.z();  //获取z轴加速度
  gyrox = gyro_v.x();  //获取x轴角速度
  gyroy = gyro_v.y();  //获取y轴角速度
  gyroz = gyro_v.z();  //获取z轴角速度
  magx = mag_v.x(); //获取x轴磁力计
  magy = mag_v.y(); //获取y轴磁力计
  magz = mag_v.z(); //获取z轴磁力计
  // 串口打印 用于测试
  //  Serial.print(system);
  //  Serial.print(",");
  //  Serial.print(accel);
  //  Serial.print(",");
  //  Serial.print(gyro);
  //  Serial.print(",");
  //  Serial.print(mg);
  //  Serial.print(",");
  //  Serial.print(roll);
  //  Serial.print(",");
  //  Serial.print(pitch);
  //  Serial.print(",");
  //  Serial.println(yaw);
  //  json = GetJSONString ("lowerarm_r", x, y ,z,w);
  //  int temp = sensor.getTemp();

  float vol = My_Sensor::readPower();
  // 将上述数据打包为json格式
  json = GetJSONString (name, accx, accy, accz, gyrox, gyroy, gyroz, magx, magy, magz, system);
  Input_adress(Wifi_ip, num1, num2, num3, num4);//用对应的ip地址赋值给这几个变量
  IPAddress destination_ip(num1, num2, num3, num4);  // 接收方设备的IP地址
  // 发送数据
  //UDPSendData(json, SERVER, port);//第一个参数json是包含传感器数据的JSON字符串，它将被发送到服务器。第二个参数SERVER表示服务器的地址或主机名，用于指定数据发送的目标。第三个参数port表示服务器上接收数据的端口号，用于指定数据发送的目标端口。
  udp.beginPacket(destination_ip, udp_port);
  udp.write((const uint8_t*)json.c_str(), json.length());
  udp.endPacket();
  // 通过设置一定延时调整发送的频率
  //delay(10000);

}



  /***************************************************************
  *  函数名称：My_Sensor::GetInString
  *  函数作用：获得传感器温度 校准状态数据 用于测试
  *  输入参数：sensorName（传感器名称）  temp（温度）  system（校准状态）
  *  返 回 值：string类型
  *  函数的使用方法：在主函数中创建类名传入参数并直接调用函数，示例：My_Sensor my_sensor；//创建对象  my_sensor.GetInString("szu001", 30， 1); //函数调用 
  *  日    期： 2023.8.29
 **************************************************************/
String My_Sensor::GetInString(String sensorName, int temp, int system) 
{
    StaticJsonDocument<1024> staticJsonDocument;  //创建静态json文档对象staticJsonDocument
    staticJsonDocument["name"] = sensorName;      //设置json文档中的传感器名称
 
    staticJsonDocument["temperature"]=temp;       //设置传感器温度
    staticJsonDocument["calibration"]=system;     //设置传感器校准状态
    char docBuf[300];                            //创建缓冲区
    serializeJson(staticJsonDocument, docBuf);    //将staticJsonDocument对象序列化为JSON格式，并将结果存储在docBuf缓冲区中
    return String(docBuf);                        //将缓冲区中的JSON字符串转换为String类型，并返回数据。
}




/***************************************************************
  *  函数名称：My_Sensor::GetJSONString
  *  函数作用：将传感器的各个参数封装为一个JSON字符串，并将其作为String类型返回
  *  输入参数：sensorName（传感器名称） x,y,z,w（四元数数据） euler（欧拉角） acc（加速度） system（校准状态指数） power（功耗）
  *  返 回 值：string类型
  *  函数的使用方法：在主函数中创建类名传入参数并直接调用函数，示例：My_Sensor my_sensor；//创建对象  my_sensor.GetJSONString("szu001", 1， 2， 3， 4， 5， 6， 1); //函数调用 
  *  日    期： 2023.8.29
 **************************************************************/
// 将读取到的运动数据按照json格式存放
// 格式为{"n":"name","o":{"x":"0.0301","y":"0.3863","z":"-0.7606","w":"-0.5209","euler":"23.23","accx":"9.80","accy":"9.80","accz":"9.80",p":"11"}}
String My_Sensor::GetJSONString(String sensorName, float accx, float accy, float accz, float gyrox, float gyroy, float gyroz, float magx, float magy, float magz, int system) 
{
    StaticJsonDocument<1024> staticJsonDocument;//创建静态的JSON文档对象staticJsonDocument，缓冲区大小为1024
    staticJsonDocument["n"] = sensorName;       //设置传感器名称

    //DynamicJsonDocument orientation(768);       //创建动态json文档对象orientation，缓冲区大小为768

    //staticJsonDocument["x"] = String(x, 4);            // 四元数 x 的值
   // staticJsonDocument["y"] = String(y, 4);            // 四元数 y 的值
    //staticJsonDocument["z"] = String(z, 4);            // 四元数 z 的值
    //staticJsonDocument["w"] = String(w, 4);            // 四元数 w 的值
    //orientation["euler"] = String(euler, 4);    // 欧拉角
    staticJsonDocument["accx"] = String(accx, 4);        // 加速度
    staticJsonDocument["accy"] = String(accy, 4);        // 加速度
    staticJsonDocument["accz"] = String(accz, 4);        // 加速度
    staticJsonDocument["gx"] = String(gyrox, 4);        // 角速度
    staticJsonDocument["gy"] = String(gyroy, 4);        // 角速度
    staticJsonDocument["gz"] = String(gyroz, 4);        // 角速度
    staticJsonDocument["magx"] = String(magx, 4);        // 角速度
    staticJsonDocument["magy"] = String(magy, 4);        // 角速度
    staticJsonDocument["magz"] = String(magz, 4);        // 角速度
    staticJsonDocument["s"] = String(system, 2);       // 校准状态指数
    //staticJsonDocument["p"] = String(power, 3);        //电压
    //orientation["o"] = orientation;      //将orientation对象存储到staticJsonDocument对象的"o"键中
    char docBuf[300];                           //创建一个字符数组docBuf作为序列化后的JSON字符串的缓冲区。
    serializeJson(staticJsonDocument, docBuf);  //将staticJsonDocument对象序列化为JSON格式，并将结果存储在docBuf缓冲区中。

    return String(docBuf);                      //将缓冲区中的JSON字符串转换为String类型，并将其返回。
}



/***************************************************************
  *  函数名称：UDPConnect
  *  函数作用：通过 UDP 协议连接到一个指定的服务器。
  *  输入参数：无
  *  返 回 值：无
  *  函数的使用方法：在主函数中创建类名传入参数并直接调用函数，示例：My_Sensor my_sensor；//创建对象  my_sensor.UDPConnect(); //函数调用 
  *  日    期： 2023.8.29
 **************************************************************/
//void My_Sensor::UDPConnect()//通过udp协议连接
//{
    //IPAddress ipAddress = IPAddress();// 创建一个 IPAddress 对象
    //ipAddress.fromString(SERVER); // 通过字符串设置 IPAddress 对象的值，SERVER 是一个表示服务器 IP 地址的全局变量
    //My_Sensor::udpClient.connect(ipAddress, PORT); // 使用 UDP 客户端对象的 connect 方法连接到指定的服务器 IP 地址和端口号（PORT 是一个表示端口号的全局变量）  
    //Serial.println ("Server Connected");
//}



/***************************************************************
  *  函数名称：My_Sensor::UDPSendData
  *  函数作用：通过udp协议发送数据
  *  输入参数：message（需要发送的数据） ip（ip地址） port（端口号）
  *  返 回 值：无
  *  函数的使用方法：在主函数中创建类名传入参数并直接调用函数，示例：My_Sensor my_sensor；//创建对象  my_sensor.UDPSendData("szu001", 192.168.3.2 ，6666); //函数调用 
  *  日    期： 2023.8.29
 **************************************************************/
void My_Sensor::UDPSendData(String message, String ip, int port)
{
    char charBuffer[1024];
    message.toCharArray(charBuffer, 1024);  // 转成数组

    if (My_Sensor::udpClient.connected()){
      My_Sensor::udpClient.broadcastTo(charBuffer, port);
    } else {
      Serial.println ("Server Not Connected");
      IPAddress ipAddress = IPAddress();
      ipAddress.fromString(ip);
      My_Sensor::udpClient.connect(ipAddress, port);   
      Serial.println ("Server Connected");
    }
}



/***************************************************************
  *  函数名称：connectToWiFi
  *  函数作用：通过wifi连接到一个指定的网络
  *  输入参数：无
  *  返 回 值：无
  *  函数的使用方法：在主函数中创建类名传入参数并直接调用函数，示例：My_Sensor my_sensor；//创建对象  my_sensor.connectToWiFi(); //函数调用 
  *  日    期： 2023.8.29
 **************************************************************/
/* void My_Sensor::connectToWiFi (){
  Serial.print ("Connecting to WiFI");
   WiFi.mode (WIFI_STA);// 将 Wi-Fi 模式设置为 Station 模式，即作为客户端连接到 Wi-Fi 网络
   WiFi.begin (WIFI_NETWORK, WIFI_PASSWORD);// 开启wifi
   unsigned long startAttemptTime = millis ();// 记录连接尝试开始的时间

   // 在规定的时间内（WIFI_TIMEOUT_MS）等待连接成功
   while (WiFi.status () != WL_CONNECTED && millis() - startAttemptTime < WIFI_TIMEOUT_MS){
     Serial.print (".");
     delay(100);
   }

   // 错误处理
   if (WiFi.status () != WL_CONNECTED){
     Serial.print ("Failed!");
   } else {
    Serial.print ("Connected!");
     Serial.print (WiFi.localIP());
   }
 }
*/




/***************************************************************
  *  函数名称：readPower
  *  函数作用：读取 GPIO 引脚的模拟值，获取电压
  *  输入参数：无
  *  返 回 值：无
  *  函数的使用方法：在主函数中创建类名传入参数并直接调用函数，示例：My_Sensor my_sensor；//创建对象  my_sensor.readPower(); //函数调用 
  *  日    期： 2023.8.29
 **************************************************************/

 float My_Sensor::readPower(){
    // 读取 GPIO 引脚的模拟值
   int analogValue = analogRead(voltagePin);

   // 将模拟值转换为电压值
   float voltage = analogValue*3.3/4095; // 根据实际参考电压和模数转换位数进行调整

   // 输出电压值
    Serial.print("Voltage: ");
    //Serial.print(analogValue);       
    Serial.print(voltage);                                                                    
    Serial.println(" V");
    Serial.print("34: ");
    Serial.print(analogRead(34));
    return voltage;
    
 }
 



