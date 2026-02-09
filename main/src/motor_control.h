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
    void setControlMode(int mode);  // 0=velocity, 1=angle, 2=torque, 3=vibration
    int getControlMode();

    // 震动模式控制
    void setVibrationParams(float amplitude, float frequency, float torque);
    void getVibrationParams(float& amplitude, float& frequency, float& torque);
    void updateVibration();  // 在loop中调用以更新震动

    // PID参数设置
    void setPIDVelocity(float P, float I, float D, float LPF);
    void setPIDAngle(float P, float I, float D, float LPF);
    void setPIDCurrentQ(float P, float I, float D, float LPF);
    void setPIDCurrentD(float P, float I, float D, float LPF);

    // 获取PID控制器
    PIDController* getVelocityPID();
    PIDController* getAnglePID();
    PIDController* getCurrentQPID();
    PIDController* getCurrentDPID();

    // 获取LPF滤波器
    LowPassFilter* getVelocityLPF();
    LowPassFilter* getAngleLPF();
    LowPassFilter* getCurrentQLPF();
    LowPassFilter* getCurrentDLPF();

    // 限制参数设置
    void setVoltageLimit(float limit);
    void setVelocityLimit(float limit);
    void setCurrentLimit(float limit);

    // 获取限制参数
    float getVoltageLimit();
    float getVelocityLimit();

    // 获取传感器数据
    float getSensorAngle();
    float getSensorVelocity();

    // 获取电压数据
    float getVoltageQ();
    float getVoltageD();

    // 获取电流设定值和限制
    float getCurrentSP();
    float getCurrentLimit();

    // 控制权限管理
    enum ControlSource {
        CONTROL_NONE = 0,
        CONTROL_WEB = 1,
        CONTROL_SERIAL = 2
    };

    void setControlSource(ControlSource source);
    ControlSource getControlSource();
    bool checkControlPermission(ControlSource source);

    // 打印传感器信息
    void printSensorInfo();

#if CURRENT_SENSE_TYPE > 0
    // 获取电流信息
    float getCurrentA();
    float getCurrentB();
    float getCurrentC();
    float getCurrentQ();
    float getCurrentD();
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
    int control_mode;  // 0=velocity, 1=angle, 2=torque, 3=vibration

    // 震动模式参数
    float vibration_amplitude;   // 震动振幅 (rad)
    float vibration_frequency;   // 震动频率 (Hz)
    float vibration_torque;      // 震动扭矩限制 (A或V)
    unsigned long vibration_last_time;  // 上次更新时间
    float vibration_phase;       // 当前相位

    // 控制权限
    ControlSource current_control_source;
};

// 全局电机控制对象指针（在cpp中定义）
extern MotorControl* g_motorControl;

#endif // MOTOR_CONTROL_H

