#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <Arduino.h>
#include "motor_control.h"

// 串口协议处理类
class SerialProtocol {
public:
    SerialProtocol(MotorControl* mc);
    
    // 初始化
    void begin();
    
    // 处理串口命令（在loop中调用）
    void processCommands();
    
    // 发送数据帧
    void sendSensorData();
    void sendMotorData();
    void sendCurrentData();
    
    // 数据流控制
    void setStreamEnabled(bool enabled);
    void setStreamInterval(unsigned long interval_ms);
    
    // 自动发送数据（在loop中调用）
    void update();

private:
    MotorControl* motorControl;
    
    // 命令缓冲区
    static const int CMD_BUFFER_SIZE = 128;
    char cmdBuffer[CMD_BUFFER_SIZE];
    int cmdIndex;
    
    // 数据流控制
    bool streamEnabled;
    unsigned long streamInterval;
    unsigned long lastStreamTime;
    
    // 命令解析
    void parseCommand(char* cmd);
    void handleSetMode(char* params);
    void handleSetTarget(char* params);
    void handleSetPID(char* params);
    void handleSetLimit(char* params);
    void handleStream(char* params);
    void handleGetPID(char* params);
    void handleGetLimit(char* params);
    void handleSetVibration(char* params);
    
    // 发送PID参数
    void sendPIDParams(const char* type, PIDController* pid, LowPassFilter* lpf);
    void sendAllPIDParams();

    // 发送ACK
    void sendAck(const char* cmd, const char* status);
    void sendAckOK(const char* cmd);
    void sendAckError(const char* cmd, const char* error);
    
    // 辅助函数
    float parseFloat(char** str);
    int parseInt(char** str);
    char* nextToken(char** str, char delimiter);
};

#endif

