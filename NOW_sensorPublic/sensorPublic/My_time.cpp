#include "My_time.h"

hw_timer_t *my_timer = NULL; //创建硬件定时器
bool flag = false; //创建中断标志位
extern const char *NVS_Name;
int MySendRate = 100;                // 初始化发送数据的速度
bool flag_Rate = false;
//定时中断回调函数
void IRAM_ATTR timer_flag()
{
  if(MySendRate != 100 && flag_Rate == false)
  {
    flag_Rate = true;
  }
  flag = true;//进入定时中断，触发标志位
}

//初始化定时器设置
  void Init_timeset()
  {
    //初始化定时器
  my_timer = timerBegin(0, 80, true);//配置定时器，这里使用0号定时器，还可使用1-3号定时器；80为分频系数，定时器基础频率为80MHz，设置80可以保证1us记录一次；ture为向上计数，也可设置false为向下计数
  //配置定时器s
  timerAttachInterrupt(my_timer, &timer_flag, true);//true表示边沿触发，可以设置false改为电平触发
  //设置定时模式
  timerAlarmWrite(my_timer, 1000*1000/MySendRate, true);//10*1000表示10ms(及100hz)触发一次中断，true表示每10ms触发一次，可以设置为false改为单次触发
  //启动定时器
  timerAlarmEnable(my_timer);
  }


  void Rerate()
  {
    timerAlarmWrite(my_timer, 1000*1000/MySendRate, true);//10*1000表示10ms(及100hz)触发一次中断，true表示每10ms触发一次，可以设置为false改为单次触发
    //启动定时器
    timerAlarmEnable(my_timer);
  }

  void Save_Rerate(int Rate)
  {
    MySendRate = Rate;
  }
  