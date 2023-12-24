#ifndef PREFERENCES_HELPER_H
#define PREFERENCES_HELPER_H
//要使用这个库，可在其它文件中包含#include "PreferencesHelper.h"
#include <Preferences.h>

extern Preferences preferences;//定义一个Preferences对象
extern const char *NVS_Name;

void NVSINIT(const char *name);//创建一个nvs空间，在其中name为空间名
void Input_NVSString(const char *name, const char *space_name, String data);//在nvs中存储字符串变量，其中name为创建的nvs空间名，space_name为储存字符串的键名，data为要存储的字符串
String Output_NVSString(const char *name, const char *space_name);//读取nvs中存储的字符串变量，其中name为创建的nvs空间名，space_name为储存字符串的键名
void Input_NVSInt(const char *name, const char *space_name, int data);//在nvs中存储整型变量，其中name为创建的nvs空间名，space_name为储存整型变量的键名，data为要存储的整型数据
int Output_NVSInt(const char *name, const char *space_name);//读取nvs中存储的整型变量，其中name为创建的nvs空间名，space_name为储存整型变量的键名

#endif
