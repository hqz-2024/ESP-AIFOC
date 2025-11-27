#include "motor_control.h"

// 全局电机控制对象指针
MotorControl* g_motorControl = nullptr;

#if SENSOR_TYPE == 0
// 编码器中断回调函数（仅当使用编码器时）
void doEncoderA() {
    if (g_motorControl) {
        g_motorControl->getEncoder().handleA();
    }
}

void doEncoderB() {
    if (g_motorControl) {
        g_motorControl->getEncoder().handleB();
    }
}
#endif

// 构造函数
MotorControl::MotorControl()
    :
#if SENSOR_TYPE == 0
      encoder(ENCODER_PIN_A, ENCODER_PIN_B, ENCODER_PPR),
#elif SENSOR_TYPE == 1
      sensor(AS5600_I2C),  // 使用SimpleFOC预定义的AS5600配置
#endif
      driver(DRIVER_PWM_A, DRIVER_PWM_B, DRIVER_PWM_C, DRIVER_ENABLE),
      motor(MOTOR_POLE_PAIRS),
      target_velocity(0.0f)
{
    g_motorControl = this;
}

// 初始化电机系统
bool MotorControl::init() {
    Serial.println("Initializing motor control...");

    // 初始化传感器
#if SENSOR_TYPE == 0
    // 编码器传感器
    Serial.println("Initializing Encoder sensor...");
    encoder.init();
    encoder.enableInterrupts(doEncoderA, doEncoderB);
    motor.linkSensor(&encoder);
    Serial.println("Encoder initialized");

#elif SENSOR_TYPE == 1
    // AS5600磁传感器（I2C）
    Serial.println("Initializing AS5600 magnetic sensor (I2C)...");

    // 初始化I2C总线（ESP32需要指定引脚）
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000);  // 设置I2C时钟为400kHz

    // 初始化传感器
    sensor.init(&Wire);
    motor.linkSensor(&sensor);
    Serial.println("AS5600 initialized");
#endif

    // 初始化驱动器
    driver.voltage_power_supply = DRIVER_VOLTAGE;
    driver.pwm_frequency = DRIVER_PWM_FREQ;
    if (!driver.init()) {
        Serial.println("Driver init failed!");
        return false;
    }
    motor.linkDriver(&driver);
    Serial.println("Driver initialized");

    // 配置电机 - 电压模式速度控制
    motor.controller = MotionControlType::velocity;
    motor.torque_controller = TorqueControlType::voltage;

    // PID参数
    motor.PID_velocity.P = PID_VELOCITY_P;
    motor.PID_velocity.I = PID_VELOCITY_I;
    motor.PID_velocity.D = PID_VELOCITY_D;
    motor.LPF_velocity.Tf = LPF_VELOCITY_TF;

    // 限制参数
    motor.voltage_limit = MOTOR_VOLTAGE_LIMIT;
    motor.velocity_limit = MOTOR_VELOCITY_LIMIT;

    // 初始化电机
    motor.init();

    // FOC初始化（对齐传感器）
    Serial.println("Starting FOC initialization...");
    motor.initFOC();

    Serial.println("Motor control initialized successfully!");
    return true;
}

// FOC循环
void MotorControl::loopFOC() {
    motor.loopFOC();
}

// 运动控制
void MotorControl::move(float target) {
    motor.move(target);
}

// 获取当前速度
float MotorControl::getVelocity() {
    return motor.shaft_velocity;
}

// 获取当前角度
float MotorControl::getAngle() {
    return motor.shaft_angle;
}

// 获取目标速度
float MotorControl::getTargetVelocity() {
    return target_velocity;
}

// 设置目标速度
void MotorControl::setTargetVelocity(float velocity) {
    // 限制速度范围
    if (velocity > MOTOR_VELOCITY_LIMIT) {
        velocity = MOTOR_VELOCITY_LIMIT;
    }
    if (velocity < -MOTOR_VELOCITY_LIMIT) {
        velocity = -MOTOR_VELOCITY_LIMIT;
    }

    target_velocity = velocity;

    Serial.print("Target velocity set to: ");
    Serial.print(target_velocity);
    Serial.println(" rad/s");
}

// 打印传感器信息
void MotorControl::printSensorInfo() {
#if SENSOR_TYPE == 0
    // 编码器信息
    Serial.println("========== Encoder Info ==========");
    Serial.print("Encoder Count: ");
    Serial.println(encoder.getCount());
    Serial.print("Encoder Velocity: ");
    Serial.print(encoder.getVelocity(), 4);
    Serial.println(" rad/s");

#elif SENSOR_TYPE == 1
    // AS5600磁传感器信息
    Serial.println("========== AS5600 Sensor Info ==========");

    // 读取原始角度值（0-4095）
    Wire.beginTransmission(AS5600_I2C_ADDRESS);
    Wire.write(0x0C);  // 角度寄存器高字节
    Wire.endTransmission();
    Wire.requestFrom(AS5600_I2C_ADDRESS, 2);

    if (Wire.available() >= 2) {
        uint16_t raw_angle = (Wire.read() & 0x0F) << 8;  // 高4位
        raw_angle |= Wire.read();  // 低8位

        Serial.print("Raw Angle: ");
        Serial.print(raw_angle);
        Serial.print(" / 4096  (");
        Serial.print((raw_angle * 360.0) / 4096.0, 2);
        Serial.println(" deg)");
    }

    // 读取状态寄存器
    Wire.beginTransmission(AS5600_I2C_ADDRESS);
    Wire.write(0x0B);  // 状态寄存器
    Wire.endTransmission();
    Wire.requestFrom(AS5600_I2C_ADDRESS, 1);

    if (Wire.available()) {
        byte status = Wire.read();
        Serial.print("Status: 0x");
        Serial.print(status, HEX);
        Serial.print(" - ");

        if (status & 0x20) Serial.print("[Magnet Detected] ");
        if (status & 0x10) Serial.print("[Magnet Too Strong] ");
        if (status & 0x08) Serial.print("[Magnet Too Weak] ");
        Serial.println();
    }

    // 读取AGC（自动增益控制）
    Wire.beginTransmission(AS5600_I2C_ADDRESS);
    Wire.write(0x1A);  // AGC寄存器
    Wire.endTransmission();
    Wire.requestFrom(AS5600_I2C_ADDRESS, 1);

    if (Wire.available()) {
        byte agc = Wire.read();
        Serial.print("AGC (Auto Gain): ");
        Serial.print(agc);
        Serial.println(" / 255");
    }

    // 读取幅度（磁场强度）
    Wire.beginTransmission(AS5600_I2C_ADDRESS);
    Wire.write(0x1B);  // 幅度寄存器高字节
    Wire.endTransmission();
    Wire.requestFrom(AS5600_I2C_ADDRESS, 2);

    if (Wire.available() >= 2) {
        uint16_t magnitude = (Wire.read() & 0x0F) << 8;
        magnitude |= Wire.read();
        Serial.print("Magnitude: ");
        Serial.print(magnitude);
        Serial.println(" / 4095");
    }

    // SimpleFOC传感器角度
    Serial.print("Sensor Angle: ");
    Serial.print(sensor.getAngle(), 4);
    Serial.print(" rad  (");
    Serial.print(sensor.getAngle() * 57.2958, 2);
    Serial.println(" deg)");

    Serial.print("Sensor Velocity: ");
    Serial.print(sensor.getVelocity(), 4);
    Serial.println(" rad/s");
#endif

    // 电机信息（通用）
    Serial.println("---------- Motor Info ----------");
    Serial.print("Motor Angle: ");
    Serial.print(motor.shaft_angle, 4);
    Serial.print(" rad  (");
    Serial.print(motor.shaft_angle * 57.2958, 2);
    Serial.println(" deg)");

    Serial.print("Motor Velocity: ");
    Serial.print(motor.shaft_velocity, 4);
    Serial.println(" rad/s");

    Serial.print("Target Velocity: ");
    Serial.print(target_velocity, 2);
    Serial.println(" rad/s");

    Serial.print("Voltage Q: ");
    Serial.print(motor.voltage.q, 2);
    Serial.print(" V, Voltage D: ");
    Serial.print(motor.voltage.d, 2);
    Serial.println(" V");

    Serial.println("====================================");
    Serial.println();
}

