/*
 * ESP32 FOC 电机控制系统
 *
 * 功能模块：
 * - motor_control: 电机FOC控制
 * - wifi_manager: WiFi连接管理
 * - web_server: Web控制界面
 *
 * 配置文件：user_config.h
 */

#include "user_config.h"
#include "src/motor_control.h"
#include "src/wifi_manager.h"
#include "src/web_server.h"

// 创建功能模块对象
MotorControl motorControl;
WiFiManager wifiManager;
WebServerManager webServer(&motorControl);

void setup() {
    // 初始化串口
    Serial.begin(SERIAL_BAUDRATE);
    delay(1000);
    Serial.println("\n========================================");
    Serial.println("ESP32 FOC Motor Control System");
    Serial.println("========================================\n");

    // 初始化电机控制
    if (!motorControl.init()) {
        Serial.println("ERROR: Motor initialization failed!");
        while(1) { delay(1000); }
    }

    // 连接WiFi
    if (!wifiManager.connect()) {
        Serial.println("WARNING: WiFi connection failed!");
        Serial.println("System will continue without WiFi...");
    } else {
        // 启动Web服务器
        webServer.begin();
    }

    Serial.println("\n========================================");
    Serial.println("System Ready!");
    Serial.println("========================================\n");
}

void loop() {
    // FOC算法循环（高优先级）
    motorControl.loopFOC();

    // 运动控制
    motorControl.move(motorControl.getTargetVelocity());

    // 处理Web请求
    if (wifiManager.isConnected()) {
        webServer.handleClient();
    }

    // 定期输出详细传感器信息
    static unsigned long last_print = 0;
    if (millis() - last_print > STATUS_PRINT_INTERVAL) {
        motorControl.printSensorInfo();
        Serial.print("Speed: ");
        Serial.print(motorControl.getVelocity(), 2);
        Serial.print(" rad/s, Angle: ");
        Serial.print(motorControl.getAngle(), 2);
        Serial.print(" rad, Target: ");
        Serial.print(motorControl.getTargetVelocity(), 2);
        Serial.println(" rad/s");
        last_print = millis();
    }
}
