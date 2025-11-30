#include "serial_protocol.h"
#include <Wire.h>

#define AS5600_I2C_ADDRESS 0x36

SerialProtocol::SerialProtocol(MotorControl* mc) 
    : motorControl(mc), cmdIndex(0), streamEnabled(true), 
      streamInterval(100), lastStreamTime(0) {
}

void SerialProtocol::begin() {
    cmdIndex = 0;
    streamEnabled = true;
    streamInterval = 100;  // 默认100ms发送一次
    lastStreamTime = 0;
}

// 处理串口命令
void SerialProtocol::processCommands() {
    while (Serial.available() > 0) {
        char c = Serial.read();
        
        // 接收到换行符，处理命令
        if (c == '\n' || c == '\r') {
            if (cmdIndex > 0) {
                cmdBuffer[cmdIndex] = '\0';
                parseCommand(cmdBuffer);
                cmdIndex = 0;
            }
        }
        // 缓存字符
        else if (cmdIndex < CMD_BUFFER_SIZE - 1) {
            cmdBuffer[cmdIndex++] = c;
        }
        // 缓冲区溢出，重置
        else {
            cmdIndex = 0;
        }
    }
}

// 自动发送数据
void SerialProtocol::update() {
    if (!streamEnabled) return;
    
    unsigned long now = millis();
    if (now - lastStreamTime >= streamInterval) {
        sendSensorData();
        sendMotorData();
        #if CURRENT_SENSE_TYPE > 0
        sendCurrentData();
        #endif
        lastStreamTime = now;
    }
}

// 解析命令
void SerialProtocol::parseCommand(char* cmd) {
    // 跳过前导空格
    while (*cmd == ' ') cmd++;
    
    // 提取命令类型
    char* comma = strchr(cmd, ',');
    if (comma) {
        *comma = '\0';
        char* params = comma + 1;
        
        if (strcmp(cmd, "SET_MODE") == 0) {
            handleSetMode(params);
        }
        else if (strcmp(cmd, "SET_TARGET") == 0) {
            handleSetTarget(params);
        }
        else if (strcmp(cmd, "SET_PID") == 0) {
            handleSetPID(params);
        }
        else if (strcmp(cmd, "SET_LIMIT") == 0) {
            handleSetLimit(params);
        }
        else if (strcmp(cmd, "STREAM") == 0) {
            handleStream(params);
        }
        else if (strcmp(cmd, "GET_PID") == 0) {
            handleGetPID(params);
        }
        else if (strcmp(cmd, "GET_LIMIT") == 0) {
            handleGetLimit(params);
        }
        else if (strcmp(cmd, "RELEASE") == 0) {
            motorControl->setControlSource(MotorControl::CONTROL_NONE);
            sendAckOK("RELEASE");
        }
        else {
            sendAckError(cmd, "UNSUPPORTED");
        }
    }
    else {
        sendAckError("UNKNOWN", "FORMAT");
    }
}

// 设置控制模式
void SerialProtocol::handleSetMode(char* params) {
    // 串口控制自动获取控制权
    motorControl->setControlSource(MotorControl::CONTROL_SERIAL);

    if (strcmp(params, "POS") == 0) {
        motorControl->setControlMode(1);  // 位置控制
        sendAckOK("SET_MODE");
    }
    else if (strcmp(params, "VEL") == 0) {
        motorControl->setControlMode(0);  // 速度控制
        sendAckOK("SET_MODE");
    }
    else if (strcmp(params, "CURR") == 0) {
        motorControl->setControlMode(2);  // 电流控制
        sendAckOK("SET_MODE");
    }
    else {
        sendAckError("SET_MODE", "INVALID_MODE");
    }
}

// 设置目标值
void SerialProtocol::handleSetTarget(char* params) {
    // 串口控制自动获取控制权
    motorControl->setControlSource(MotorControl::CONTROL_SERIAL);

    float value = atof(params);
    int mode = motorControl->getControlMode();

    switch (mode) {
        case 0:  // 速度控制
            motorControl->setTargetVelocity(value);
            break;
        case 1:  // 位置控制
            motorControl->setTargetAngle(value);
            break;
        case 2:  // 扭矩控制
            motorControl->setTargetTorque(value);
            break;
    }
    sendAckOK("SET_TARGET");
}

// 辅助函数：解析下一个token
char* SerialProtocol::nextToken(char** str, char delimiter) {
    if (*str == NULL) return NULL;
    char* start = *str;
    char* end = strchr(start, delimiter);
    if (end) {
        *end = '\0';
        *str = end + 1;
    } else {
        *str = NULL;
    }
    return start;
}

// 设置PID参数
void SerialProtocol::handleSetPID(char* params) {
    char* ptr = params;
    char* loop = nextToken(&ptr, ',');
    if (!loop) {
        sendAckError("SET_PID", "FORMAT");
        return;
    }

    float P = atof(nextToken(&ptr, ','));
    float I = atof(nextToken(&ptr, ','));
    float D = atof(nextToken(&ptr, ','));
    float LPF = atof(nextToken(&ptr, ','));

    if (strcmp(loop, "VEL") == 0) {
        motorControl->setPIDVelocity(P, I, D, LPF);
        sendAckOK("SET_PID");
    }
    else if (strcmp(loop, "ANG") == 0) {
        motorControl->setPIDAngle(P, I, D, LPF);
        sendAckOK("SET_PID");
    }
    else if (strcmp(loop, "IQ") == 0) {
        motorControl->setPIDCurrentQ(P, I, D, LPF);
        sendAckOK("SET_PID");
    }
    else if (strcmp(loop, "ID") == 0) {
        motorControl->setPIDCurrentD(P, I, D, LPF);
        sendAckOK("SET_PID");
    }
    else {
        sendAckError("SET_PID", "INVALID_LOOP");
    }
}

// 设置限制参数
void SerialProtocol::handleSetLimit(char* params) {
    char* ptr = params;
    char* type = nextToken(&ptr, ',');
    if (!type) {
        sendAckError("SET_LIMIT", "FORMAT");
        return;
    }

    float value = atof(nextToken(&ptr, ','));

    if (strcmp(type, "VOLT") == 0) {
        motorControl->setVoltageLimit(value);
        sendAckOK("SET_LIMIT");
    }
    else if (strcmp(type, "VEL") == 0) {
        motorControl->setVelocityLimit(value);
        sendAckOK("SET_LIMIT");
    }
    else if (strcmp(type, "CURR") == 0) {
        motorControl->setCurrentLimit(value);
        sendAckOK("SET_LIMIT");
    }
    else {
        sendAckError("SET_LIMIT", "INVALID_TYPE");
    }
}

// 数据流控制
void SerialProtocol::handleStream(char* params) {
    if (strcmp(params, "ON") == 0) {
        streamEnabled = true;
        sendAckOK("STREAM");
    }
    else if (strcmp(params, "OFF") == 0) {
        streamEnabled = false;
        sendAckOK("STREAM");
    }
    else {
        // 尝试解析周期
        char* ptr = params;
        char* onoff = nextToken(&ptr, ',');
        if (strcmp(onoff, "ON") == 0 && ptr) {
            streamEnabled = true;
            streamInterval = atoi(ptr);
            sendAckOK("STREAM");
        }
        else {
            sendAckError("STREAM", "FORMAT");
        }
    }
}

// 查询PID参数
void SerialProtocol::handleGetPID(char* params) {
    // 实现查询功能（可选）
    sendAckError("GET_PID", "NOT_IMPLEMENTED");
}

// 查询限制参数
void SerialProtocol::handleGetLimit(char* params) {
    // 实现查询功能（可选）
    sendAckError("GET_LIMIT", "NOT_IMPLEMENTED");
}

// 发送ACK
void SerialProtocol::sendAck(const char* cmd, const char* status) {
    Serial.print("ACK,");
    Serial.print(cmd);
    Serial.print(",");
    Serial.println(status);
}

void SerialProtocol::sendAckOK(const char* cmd) {
    sendAck(cmd, "OK");
}

void SerialProtocol::sendAckError(const char* cmd, const char* error) {
    Serial.print("ACK,");
    Serial.print(cmd);
    Serial.print(",ERR,");
    Serial.println(error);
}

// 数据流控制
void SerialProtocol::setStreamEnabled(bool enabled) {
    streamEnabled = enabled;
}

void SerialProtocol::setStreamInterval(unsigned long interval_ms) {
    streamInterval = interval_ms;
}

// 发送传感器数据帧
void SerialProtocol::sendSensorData() {
#if SENSOR_TYPE == 1
    // 读取AS5600原始数据
    Wire.beginTransmission(AS5600_I2C_ADDRESS);
    Wire.write(0x0C);  // 角度寄存器
    Wire.endTransmission();
    Wire.requestFrom(AS5600_I2C_ADDRESS, 2);

    uint16_t raw_angle = 0;
    if (Wire.available() >= 2) {
        raw_angle = (Wire.read() & 0x0F) << 8;
        raw_angle |= Wire.read();
    }

    // 读取状态
    Wire.beginTransmission(AS5600_I2C_ADDRESS);
    Wire.write(0x0B);  // 状态寄存器
    Wire.endTransmission();
    Wire.requestFrom(AS5600_I2C_ADDRESS, 1);

    uint8_t status = 0;
    if (Wire.available()) {
        status = Wire.read();
    }

    // 读取AGC
    Wire.beginTransmission(AS5600_I2C_ADDRESS);
    Wire.write(0x1A);  // AGC寄存器
    Wire.endTransmission();
    Wire.requestFrom(AS5600_I2C_ADDRESS, 1);

    uint8_t agc = 0;
    if (Wire.available()) {
        agc = Wire.read();
    }

    // 读取幅度
    Wire.beginTransmission(AS5600_I2C_ADDRESS);
    Wire.write(0x1B);  // 幅度寄存器
    Wire.endTransmission();
    Wire.requestFrom(AS5600_I2C_ADDRESS, 2);

    uint16_t magnitude = 0;
    if (Wire.available() >= 2) {
        magnitude = (Wire.read() & 0x0F) << 8;
        magnitude |= Wire.read();
    }

    // 获取SimpleFOC传感器数据
    float angle_rad = motorControl->getSensorAngle();
    float vel_rad_s = motorControl->getSensorVelocity();

    // 计算角度（度）
    float deg = (raw_angle * 360.0) / 4096.0;
    float angle_deg_total = angle_rad * 57.2958;

    // 发送SENSOR帧
    // 格式：SENSOR,raw_angle,raw_max,deg,status,agc,magnitude,angle_rad,angle_deg_total,vel_rad_s
    Serial.print("SENSOR,");
    Serial.print(raw_angle);
    Serial.print(",4096,");
    Serial.print(deg, 2);
    Serial.print(",0x");
    Serial.print(status, HEX);
    Serial.print(",");
    Serial.print(agc);
    Serial.print(",");
    Serial.print(magnitude);
    Serial.print(",");
    Serial.print(angle_rad, 4);
    Serial.print(",");
    Serial.print(angle_deg_total, 2);
    Serial.print(",");
    Serial.println(vel_rad_s, 4);
#endif
}

// 发送电机数据帧
void SerialProtocol::sendMotorData() {
    float motor_angle_rad = motorControl->getAngle();
    float motor_deg_total = motor_angle_rad * 57.2958;
    float motor_vel_rad_s = motorControl->getVelocity();

    float target_vel = 0;
    int mode = motorControl->getControlMode();
    if (mode == 0) {
        target_vel = motorControl->getTargetVelocity();
    }

    float voltage_q = motorControl->getVoltageQ();
    float voltage_d = motorControl->getVoltageD();
    float current_sp = motorControl->getCurrentSP();
    float current_limit = motorControl->getCurrentLimit();

    // 发送MOTOR帧
    // 格式：MOTOR,motor_angle_rad,motor_deg_total,motor_vel_rad_s,target_vel_rad_s,voltage_q,voltage_d,current_sp,current_limit
    Serial.print("MOTOR,");
    Serial.print(motor_angle_rad, 4);
    Serial.print(",");
    Serial.print(motor_deg_total, 2);
    Serial.print(",");
    Serial.print(motor_vel_rad_s, 4);
    Serial.print(",");
    Serial.print(target_vel, 2);
    Serial.print(",");
    Serial.print(voltage_q, 2);
    Serial.print(",");
    Serial.print(voltage_d, 2);
    Serial.print(",");
    Serial.print(current_sp, 3);
    Serial.print(",");
    Serial.println(current_limit, 3);
}

// 发送电流数据帧
void SerialProtocol::sendCurrentData() {
#if CURRENT_SENSE_TYPE > 0
    float ia = motorControl->getCurrentA();
    float ib = motorControl->getCurrentB();
    float ic = motorControl->getCurrentC();
    float iq = motorControl->getCurrentQ();
    float id = motorControl->getCurrentD();

    // 发送CURRENT帧
    // 格式：CURRENT,ia,ib,ic,iq,id
    Serial.print("CURRENT,");
    Serial.print(ia, 3);
    Serial.print(",");
    Serial.print(ib, 3);
    Serial.print(",");
    Serial.print(ic, 3);
    Serial.print(",");
    Serial.print(iq, 3);
    Serial.print(",");
    Serial.println(id, 3);
#endif
}

