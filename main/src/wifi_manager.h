#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <WiFi.h>
#include "../user_config.h"

// WiFi管理类
class WiFiManager {
public:
    WiFiManager();
    
    // 连接WiFi
    bool connect();
    
    // 获取IP地址
    IPAddress getIP();
    
    // 检查连接状态
    bool isConnected();
    
private:
    const char* ssid;
    const char* password;
};

#endif // WIFI_MANAGER_H

