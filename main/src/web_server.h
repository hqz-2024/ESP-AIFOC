#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <WebServer.h>
#include "../user_config.h"
#include "motor_control.h"

// Web服务器类
class WebServerManager {
public:
    WebServerManager(MotorControl* motorCtrl);
    
    // 初始化Web服务器
    void begin();
    
    // 处理客户端请求（需要在loop中调用）
    void handleClient();
    
private:
    WebServer server;
    MotorControl* motorControl;
    
    // 路由处理函数
    void handleRoot();
    void handleSetSpeed();
    
    // 静态包装函数（用于WebServer回调）
    static void handleRootStatic();
    static void handleSetSpeedStatic();
    
    // 静态实例指针
    static WebServerManager* instance;
};

#endif // WEB_SERVER_H

