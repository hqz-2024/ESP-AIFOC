#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <SimpleFOC.h>
#include <Wire.h>
#include "../user_config.h"

// 编码器中断回调函数声明（仅当使用编码器时）
#if SENSOR_TYPE == 0
void doEncoderA();
void doEncoderB();
#endif

// 电机控制类
class MotorControl {
public:
    MotorControl();
    
    // 初始化电机系统
    bool init();
    
    // FOC循环（需要在loop中高频调用）
    void loopFOC();
    
    // 运动控制（设置目标速度）
    void move(float target);
    
    // 获取电机状态
    float getVelocity();
    float getAngle();
    float getTargetVelocity();
    float getTargetAngle();
    float getTargetTorque();

    // 设置目标值
    void setTargetVelocity(float velocity);
    void setTargetAngle(float angle);
    void setTargetTorque(float torque);

    // 切换控制模式
    void setControlMode(int mode);  // 0=velocity, 1=angle, 2=torque
    int getControlMode();

    // 打印传感器信息
    void printSensorInfo();

#if CURRENT_SENSE_TYPE > 0
    // 获取电流信息
    float getCurrentA();
    float getCurrentB();
    float getCurrentC();
    void printCurrentInfo();
#endif

    // 获取电机对象引用（用于高级操作）
    BLDCMotor& getMotor() { return motor; }

#if SENSOR_TYPE == 0
    // 获取编码器对象引用（用于中断回调）
    Encoder& getEncoder() { return encoder; }
#endif

private:
#if SENSOR_TYPE == 0
    Encoder encoder;                    // 编码器传感器
#elif SENSOR_TYPE == 1
    MagneticSensorI2C sensor;           // AS5600磁传感器
#endif

#if CURRENT_SENSE_TYPE == 1
    InlineCurrentSense current_sense;   // 在线电流传感器
#elif CURRENT_SENSE_TYPE == 2
    LowsideCurrentSense current_sense;  // 低侧电流传感器
#endif

    BLDCDriver3PWM driver;
    BLDCMotor motor;

    // 目标值
    float target_velocity;
    float target_angle;
    float target_torque;
    int control_mode;  // 0=velocity, 1=angle, 2=torque
};

// 全局电机控制对象指针（在cpp中定义）
extern MotorControl* g_motorControl;

#endif // MOTOR_CONTROL_H

