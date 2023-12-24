#include "Ip_adress.h"


void Input_adress(String ip, unsigned char &num1, unsigned char &num2, unsigned char &num3, unsigned char &num4)
{
  String delimiter = ".";
  // 提取第一个数字
  int pos = ip.indexOf(delimiter);
  num1 = ip.substring(0, pos).toInt();

  // 提取第二个数字
  ip = ip.substring(pos + 1);
  pos = ip.indexOf(delimiter);
  num2 = ip.substring(0, pos).toInt();

  // 提取第三个数字
  ip = ip.substring(pos + 1);
  pos = ip.indexOf(delimiter);
  num3 = ip.substring(0, pos).toInt();

  // 提取第四个数字
  ip = ip.substring(pos + 1);
  num4 = ip.toInt();
}