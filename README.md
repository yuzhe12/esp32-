# sensorPubilc

功能：
      1. 短按按键io0 设备进入active模式 即正常工作模式，指示灯亮io16。

      2. 开机后立即开始连接网络，若连接不上wifi则进入配网模式。
    
      3. 若正常连接wifi则esp32通过udp无线传输json的数据给指定的端口。
      
      4. 长按按键io0 设备进入低功耗模式（关机）。
      
      5. 配网时，设备首先搜索周围的可用无线网络，随即进入ap模式（热点），用户可以连接热点，在弹出来的网页中输入需要连接的无线网络信息，即可完成配网。

