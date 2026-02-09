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
    void handleSetAngle();
    void handleSetTorque();
    void handleSetMode();
    void handleSetControl();  // 统一控制接口
    void handleSetVibration();  // 震动模式设置

    // 静态包装函数（用于WebServer回调）
    static void handleRootStatic();
    static void handleSetSpeedStatic();
    static void handleSetAngleStatic();
    static void handleSetTorqueStatic();
    static void handleSetModeStatic();
    static void handleSetControlStatic();  // 统一控制接口
    static void handleSetVibrationStatic();  // 震动模式设置
    
    // 静态实例指针
    static WebServerManager* instance;
};

#endif // WEB_SERVER_H

