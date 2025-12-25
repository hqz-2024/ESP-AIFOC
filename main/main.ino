/*
 * ESP32 FOC 电机控制系统
 *
 * 功能模块：
 * - motor_control: 电机FOC控制
 * - wifi_manager: WiFi连接管理
 * - web_server: Web控制界面
 * - serial_protocol: 串口通讯协议
 *
 * 配置文件：user_config.h
 */

#include "user_config.h"
#include "src/motor_control.h"
#include "src/wifi_manager.h"
#include "src/web_server.h"
#include "src/serial_protocol.h"

// 创建功能模块对象
MotorControl motorControl;
WiFiManager wifiManager;
WebServerManager webServer(&motorControl);
SerialProtocol serialProtocol(&motorControl);

void setup() {
    // 初始化串口
    Serial.begin(SERIAL_BAUDRATE);
    delay(1000);

    // 初始化串口协议
    serialProtocol.begin();

    Serial.println("\n========================================");
    Serial.println("ESP32 FOC Motor Control System");
    Serial.println("========================================\n");

    // 初始化电机控制
    if (!motorControl.init()) {
        Serial.println("ERROR: Motor initialization failed!");
        while(1) { delay(1000); }
    }

    // // 连接WiFi
    // if (!wifiManager.connect()) {
    //     Serial.println("WARNING: WiFi connection failed!");
    //     Serial.println("System will continue without WiFi...");
    // } else {
    //     // 启动Web服务器
    //     webServer.begin();
    // }

    Serial.println("\n========================================");
    Serial.println("System Ready!");
    Serial.println("========================================\n");
}

void loop() {
    // FOC算法循环（高优先级）
    motorControl.loopFOC();

    // 运动控制 - 根据控制模式使用不同的目标值
    int mode = motorControl.getControlMode();
    switch (mode) {
        case 0:  // 速度控制
            motorControl.move(motorControl.getTargetVelocity());
            break;
        case 1:  // 位置控制
            motorControl.move(motorControl.getTargetAngle());
            break;
        case 2:  // 扭矩控制
            motorControl.move(motorControl.getTargetTorque());
            break;
    }

    // 处理串口命令
    serialProtocol.processCommands();

    // 自动发送串口数据
    serialProtocol.update();

    // // 处理Web请求
    // if (wifiManager.isConnected()) {
    //     webServer.handleClient();
    // }
}
