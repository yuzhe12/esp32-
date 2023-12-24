#include "PreferencesHelper.h"

Preferences preferences;
void NVSINIT(const char *name)
{
  preferences.begin(name, false);    // 打开NVS命名空间为 "NVS_space"
  preferences.end();//关闭NVS空间
}

void Input_NVSString(const char *name, const char *space_name, String data)
{
  preferences.begin(name, false);
  preferences.putString(space_name, data);
  preferences.end();
}

String Output_NVSString(const char *name, const char *space_name)
{
  preferences.begin(name, true);
  String data = preferences.getString(space_name, "Default String");
  preferences.end();
  return data;
}

void Input_NVSInt(const char *name, const char *space_name, int data)
{
  preferences.begin(name, false);
  preferences.putInt(space_name, data);
  preferences.end();
}

int Output_NVSInt(const char *name, const char *space_name)
{
  preferences.begin(name, true);
  int data = preferences.getInt(space_name, 0);
  preferences.end();
  return data;
}