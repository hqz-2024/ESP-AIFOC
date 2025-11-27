#include "wifi_manager.h"

// 构造函数
WiFiManager::WiFiManager() 
    : ssid(WIFI_SSID),
      password(WIFI_PASSWORD)
{
}

// 连接WiFi
bool WiFiManager::connect() {
    Serial.println("Connecting to WiFi...");
    Serial.print("SSID: ");
    Serial.println(ssid);
    
    WiFi.begin(ssid, password);
    
    int timeout = 0;
    while (WiFi.status() != WL_CONNECTED && timeout < 30) {
        delay(500);
        Serial.print(".");
        timeout++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected!");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
        return true;
    } else {
        Serial.println("\nWiFi connection failed!");
        return false;
    }
}

// 获取IP地址
IPAddress WiFiManager::getIP() {
    return WiFi.localIP();
}

// 检查连接状态
bool WiFiManager::isConnected() {
    return WiFi.status() == WL_CONNECTED;
}

