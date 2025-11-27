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
    
    // 设置目标速度
    void setTargetVelocity(float velocity);

    // 打印传感器信息
    void printSensorInfo();

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
    BLDCDriver3PWM driver;
    BLDCMotor motor;
    float target_velocity;
};

// 全局电机控制对象指针（在cpp中定义）
extern MotorControl* g_motorControl;

#endif // MOTOR_CONTROL_H

