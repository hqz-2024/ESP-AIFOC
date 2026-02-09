#include "motor_control.h"

// 调试日志宏
#if ENABLE_DEBUG_LOG
    #define DEBUG_PRINT(x) Serial.print(x)
    #define DEBUG_PRINTLN(x) Serial.println(x)
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTLN(x)
#endif

#if ENABLE_INIT_LOG
    #define INIT_PRINT(x) Serial.print(x)
    #define INIT_PRINTLN(x) Serial.println(x)
#else
    #define INIT_PRINT(x)
    #define INIT_PRINTLN(x)
#endif

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
#if CURRENT_SENSE_TYPE == 1
      // 在线电流传感器 - 只使用A和B相（2相检测）
      current_sense(CURRENT_SENSE_SHUNT_R, CURRENT_SENSE_AMP_GAIN, CURRENT_SENSE_A, CURRENT_SENSE_B),
#elif CURRENT_SENSE_TYPE == 2
      // 低侧电流传感器 - 只使用A和B相（2相检测）
      current_sense(CURRENT_SENSE_SHUNT_R, CURRENT_SENSE_AMP_GAIN, CURRENT_SENSE_LOW_A, CURRENT_SENSE_LOW_B),
#endif
      driver(DRIVER_PWM_A, DRIVER_PWM_B, DRIVER_PWM_C, DRIVER_ENABLE),
      motor(MOTOR_POLE_PAIRS),
      target_velocity(0.0f),
      target_angle(0.0f),
      target_torque(0.0f),
      control_mode(CONTROL_MODE),
      vibration_amplitude(0.5f),
      vibration_frequency(1.0f),
      vibration_torque(1.0f),
      vibration_last_time(0),
      vibration_phase(0.0f),
      current_control_source(CONTROL_NONE)
{
    g_motorControl = this;
}

// 初始化电机系统
bool MotorControl::init() {
    INIT_PRINTLN("Initializing motor control...");

    // 初始化传感器
#if SENSOR_TYPE == 0
    // 编码器传感器
    INIT_PRINTLN("Initializing Encoder sensor...");
    encoder.init();
    encoder.enableInterrupts(doEncoderA, doEncoderB);
    motor.linkSensor(&encoder);
    INIT_PRINTLN("Encoder initialized");

#elif SENSOR_TYPE == 1
    // AS5600磁传感器（I2C）
    INIT_PRINTLN("Initializing AS5600 magnetic sensor (I2C)...");

    // 初始化I2C总线（ESP32需要指定引脚）
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000);  // 设置I2C时钟为400kHz

    // 初始化传感器
    sensor.init(&Wire);
    motor.linkSensor(&sensor);
    INIT_PRINTLN("AS5600 initialized");
#endif

    // 初始化驱动器
    driver.voltage_power_supply = DRIVER_VOLTAGE;
    driver.pwm_frequency = DRIVER_PWM_FREQ;
    if (!driver.init()) {
        INIT_PRINTLN("Driver init failed!");
        return false;
    }
    motor.linkDriver(&driver);
    INIT_PRINTLN("Driver initialized");

#if CURRENT_SENSE_TYPE > 0
    // 初始化电流传感器
    // 重要：必须先链接驱动器，电流传感器初始化时需要使用驱动器进行校准
    INIT_PRINTLN("Initializing current sense...");
    current_sense.linkDriver(&driver);  // ← 先链接驱动器！
    if (current_sense.init()) {
        motor.linkCurrentSense(&current_sense);
        INIT_PRINTLN("Current sense initialized");
    } else {
        INIT_PRINTLN("WARNING: Current sense init failed!");
    }
#endif

    // 配置控制模式
    switch (control_mode) {
        case 0:  // 速度控制
            motor.controller = MotionControlType::velocity;
            INIT_PRINTLN("Control mode: Velocity");
            break;
        case 1:  // 位置控制
            motor.controller = MotionControlType::angle;
            INIT_PRINTLN("Control mode: Angle");
            break;
        case 2:  // 扭矩控制
            motor.controller = MotionControlType::torque;
            INIT_PRINTLN("Control mode: Torque");
            break;
        case 3:  // 震动模式
            motor.controller = MotionControlType::angle;
            INIT_PRINTLN("Control mode: Vibration");
            break;
        default:
            motor.controller = MotionControlType::velocity;
            control_mode = 0;
            INIT_PRINTLN("Control mode: Velocity (default)");
            break;
    }

    // 配置扭矩控制类型
#if CURRENT_SENSE_TYPE > 0
    motor.torque_controller = TorqueControlType::foc_current;  // FOC电流模式
    INIT_PRINTLN("Torque control: FOC Current");
#else
    motor.torque_controller = TorqueControlType::voltage;  // 电压模式
    INIT_PRINTLN("Torque control: Voltage");
#endif

    // PID速度控制参数
    motor.PID_velocity.P = PID_VELOCITY_P;
    motor.PID_velocity.I = PID_VELOCITY_I;
    motor.PID_velocity.D = PID_VELOCITY_D;
    motor.LPF_velocity.Tf = LPF_VELOCITY_TF;

    // PID位置控制参数
    motor.P_angle.P = PID_ANGLE_P;
    motor.P_angle.I = PID_ANGLE_I;
    motor.P_angle.D = PID_ANGLE_D;
    motor.LPF_angle.Tf = LPF_ANGLE_TF;

#if CURRENT_SENSE_TYPE > 0
    // PID电流控制参数（FOC模式）
    motor.PID_current_q.P = PID_CURRENT_Q_P;
    motor.PID_current_q.I = PID_CURRENT_Q_I;
    motor.PID_current_q.D = PID_CURRENT_Q_D;
    motor.PID_current_d.P = PID_CURRENT_D_P;
    motor.PID_current_d.I = PID_CURRENT_D_I;
    motor.PID_current_d.D = PID_CURRENT_D_D;
    motor.LPF_current_q.Tf = LPF_CURRENT_TF;
    motor.LPF_current_d.Tf = LPF_CURRENT_TF;
#endif

    // 限制参数
    motor.voltage_limit = MOTOR_VOLTAGE_LIMIT;
    motor.velocity_limit = MOTOR_VELOCITY_LIMIT;
#if CURRENT_SENSE_TYPE > 0
    motor.current_limit = MOTOR_CURRENT_LIMIT;
#endif

    // 初始化电机
    motor.init();

    // FOC初始化（对齐传感器）
    INIT_PRINTLN("Starting FOC initialization...");
    motor.initFOC();

    INIT_PRINTLN("Motor control initialized successfully!");
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

// 获取目标角度
float MotorControl::getTargetAngle() {
    return target_angle;
}

// 获取目标扭矩
float MotorControl::getTargetTorque() {
    return target_torque;
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

    DEBUG_PRINT("Target velocity set to: ");
    DEBUG_PRINT(target_velocity);
    DEBUG_PRINTLN(" rad/s");
}

// 设置目标角度
void MotorControl::setTargetAngle(float angle) {
    target_angle = angle;

    DEBUG_PRINT("Target angle set to: ");
    DEBUG_PRINT(target_angle);
    DEBUG_PRINTLN(" rad");
}

// 设置目标扭矩
void MotorControl::setTargetTorque(float torque) {
#if CURRENT_SENSE_TYPE > 0
    // 限制电流范围
    if (torque > MOTOR_CURRENT_LIMIT) {
        torque = MOTOR_CURRENT_LIMIT;
    }
    if (torque < -MOTOR_CURRENT_LIMIT) {
        torque = -MOTOR_CURRENT_LIMIT;
    }
#endif

    target_torque = torque;

    DEBUG_PRINT("Target torque set to: ");
    DEBUG_PRINT(target_torque);
#if CURRENT_SENSE_TYPE > 0
    DEBUG_PRINTLN(" A");
#else
    DEBUG_PRINTLN(" V");
#endif
}

// 切换控制模式
void MotorControl::setControlMode(int mode) {
    if (mode < 0 || mode > 3) {
        DEBUG_PRINTLN("Invalid control mode! Use 0=velocity, 1=angle, 2=torque, 3=vibration");
        return;
    }

    control_mode = mode;

    switch (control_mode) {
        case 0:
            motor.controller = MotionControlType::velocity;
            DEBUG_PRINTLN("Control mode changed to: Velocity");
            break;
        case 1:
            motor.controller = MotionControlType::angle;
            DEBUG_PRINTLN("Control mode changed to: Angle");
            break;
        case 2:
            motor.controller = MotionControlType::torque;
            DEBUG_PRINTLN("Control mode changed to: Torque");
            break;
        case 3:
            motor.controller = MotionControlType::angle;
            vibration_phase = 0.0f;
            vibration_last_time = millis();
            DEBUG_PRINTLN("Control mode changed to: Vibration");
            break;
    }
}

// 获取控制模式
int MotorControl::getControlMode() {
    return control_mode;
}

#if CURRENT_SENSE_TYPE > 0
// 获取A相电流
float MotorControl::getCurrentA() {
    return current_sense.getPhaseCurrents().a;
}

// 获取B相电流
float MotorControl::getCurrentB() {
    return current_sense.getPhaseCurrents().b;
}

// 获取C相电流（通过计算得到：Ic = -Ia - Ib）
float MotorControl::getCurrentC() {
    PhaseCurrent_s currents = current_sense.getPhaseCurrents();
    // 基尔霍夫电流定律：Ia + Ib + Ic = 0
    return -(currents.a + currents.b);
}

// 获取Q轴电流
float MotorControl::getCurrentQ() {
    return current_sense.getFOCCurrents(motor.electrical_angle).q;
}

// 获取D轴电流
float MotorControl::getCurrentD() {
    return current_sense.getFOCCurrents(motor.electrical_angle).d;
}

// 打印电流信息
void MotorControl::printCurrentInfo() {
    PhaseCurrent_s currents = current_sense.getPhaseCurrents();
    DQCurrent_s dq_currents = current_sense.getFOCCurrents(motor.electrical_angle);

    // 计算C相电流（Ic = -Ia - Ib）
    float current_c = -(currents.a + currents.b);

    Serial.println("========== Current Info ==========");
    Serial.print("Phase A: ");
    Serial.print(currents.a, 3);
    Serial.print(" A, Phase B: ");
    Serial.print(currents.b, 3);
    Serial.print(" A, Phase C: ");
    Serial.print(current_c, 3);
    Serial.println(" A (calculated)");

    Serial.print("Current Q (torque): ");
    Serial.print(dq_currents.q, 3);
    Serial.print(" A, Current D (flux): ");
    Serial.print(dq_currents.d, 3);
    Serial.println(" A");

    Serial.println("==================================");
}
#endif

// PID参数设置
void MotorControl::setPIDVelocity(float P, float I, float D, float LPF) {
    motor.PID_velocity.P = P;
    motor.PID_velocity.I = I;
    motor.PID_velocity.D = D;
    motor.LPF_velocity.Tf = LPF;
}

void MotorControl::setPIDAngle(float P, float I, float D, float LPF) {
    motor.P_angle.P = P;
    motor.P_angle.I = I;
    motor.P_angle.D = D;
    motor.LPF_angle.Tf = LPF;
}

void MotorControl::setPIDCurrentQ(float P, float I, float D, float LPF) {
#if CURRENT_SENSE_TYPE > 0
    motor.PID_current_q.P = P;
    motor.PID_current_q.I = I;
    motor.PID_current_q.D = D;
    motor.LPF_current_q.Tf = LPF;
#endif
}

void MotorControl::setPIDCurrentD(float P, float I, float D, float LPF) {
#if CURRENT_SENSE_TYPE > 0
    motor.PID_current_d.P = P;
    motor.PID_current_d.I = I;
    motor.PID_current_d.D = D;
    motor.LPF_current_d.Tf = LPF;
#endif
}

// 获取PID控制器
PIDController* MotorControl::getVelocityPID() {
    return &motor.PID_velocity;
}

PIDController* MotorControl::getAnglePID() {
    return &motor.P_angle;
}

PIDController* MotorControl::getCurrentQPID() {
#if CURRENT_SENSE_TYPE > 0
    return &motor.PID_current_q;
#else
    return nullptr;
#endif
}

PIDController* MotorControl::getCurrentDPID() {
#if CURRENT_SENSE_TYPE > 0
    return &motor.PID_current_d;
#else
    return nullptr;
#endif
}

// 获取LPF滤波器
LowPassFilter* MotorControl::getVelocityLPF() {
    return &motor.LPF_velocity;
}

LowPassFilter* MotorControl::getAngleLPF() {
    return &motor.LPF_angle;
}

LowPassFilter* MotorControl::getCurrentQLPF() {
#if CURRENT_SENSE_TYPE > 0
    return &motor.LPF_current_q;
#else
    return nullptr;
#endif
}

LowPassFilter* MotorControl::getCurrentDLPF() {
#if CURRENT_SENSE_TYPE > 0
    return &motor.LPF_current_d;
#else
    return nullptr;
#endif
}

// 限制参数设置
void MotorControl::setVoltageLimit(float limit) {
    motor.voltage_limit = limit;

    // 同步更新PID控制器的limit参数（参考BLDCMotor::init()的逻辑）
#if CURRENT_SENSE_TYPE > 0
    // 电流环的limit = 电压限制
    motor.PID_current_q.limit = limit;
    motor.PID_current_d.limit = limit;
    DEBUG_PRINT("Voltage limit set to: ");
    DEBUG_PRINT(limit);
    DEBUG_PRINT("V, PID_current_q/d.limit = ");
    DEBUG_PRINTLN(limit);
#endif

    // 如果速度环控制电压（非电流模式），更新速度环limit
    if (motor.torque_controller == TorqueControlType::voltage) {
        motor.PID_velocity.limit = limit;
        DEBUG_PRINT("PID_velocity.limit = ");
        DEBUG_PRINTLN(limit);
    }
}

void MotorControl::setVelocityLimit(float limit) {
    motor.velocity_limit = limit;

    // 同步更新位置环的limit参数
    motor.P_angle.limit = limit;

    DEBUG_PRINT("Velocity limit set to: ");
    DEBUG_PRINT(limit);
    DEBUG_PRINT(" rad/s, P_angle.limit = ");
    DEBUG_PRINTLN(limit);
}

void MotorControl::setCurrentLimit(float limit) {
#if CURRENT_SENSE_TYPE > 0
    motor.current_limit = limit;

    // 同步更新速度环的limit参数（FOC电流模式下，速度环控制电流）
    if (motor.torque_controller == TorqueControlType::foc_current) {
        motor.PID_velocity.limit = limit;
        DEBUG_PRINT("Current limit set to: ");
        DEBUG_PRINT(limit);
        DEBUG_PRINT("A, PID_velocity.limit = ");
        DEBUG_PRINTLN(limit);
    }
#endif
}

// 获取限制参数
float MotorControl::getVoltageLimit() {
    return motor.voltage_limit;
}

float MotorControl::getVelocityLimit() {
    return motor.velocity_limit;
}

// 获取传感器数据
float MotorControl::getSensorAngle() {
#if SENSOR_TYPE == 0
    return encoder.getAngle();
#elif SENSOR_TYPE == 1
    return sensor.getAngle();
#else
    return 0;
#endif
}

float MotorControl::getSensorVelocity() {
#if SENSOR_TYPE == 0
    return encoder.getVelocity();
#elif SENSOR_TYPE == 1
    return sensor.getVelocity();
#else
    return 0;
#endif
}

// 获取电压数据
float MotorControl::getVoltageQ() {
    return motor.voltage.q;
}

float MotorControl::getVoltageD() {
    return motor.voltage.d;
}

// 获取电流设定值和限制
float MotorControl::getCurrentSP() {
    return motor.current_sp;
}

float MotorControl::getCurrentLimit() {
#if CURRENT_SENSE_TYPE > 0
    return motor.current_limit;
#else
    return 0;
#endif
}

// 控制权限管理
void MotorControl::setControlSource(ControlSource source) {
    current_control_source = source;
}

MotorControl::ControlSource MotorControl::getControlSource() {
    return current_control_source;
}

bool MotorControl::checkControlPermission(ControlSource source) {
    // 如果没有控制源，允许任何源控制
    if (current_control_source == CONTROL_NONE) {
        current_control_source = source;
        return true;
    }

    // 串口优先级最高，可以抢占控制权
    if (source == CONTROL_SERIAL) {
        current_control_source = source;
        return true;
    }

    // Web端只能在没有其他控制源时控制
    if (source == CONTROL_WEB && current_control_source == CONTROL_WEB) {
        return true;
    }

    // 其他情况拒绝
    return false;
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

#if CURRENT_SENSE_TYPE > 0
    Serial.print("Current SP: ");
    Serial.print(motor.current_sp, 3);
    Serial.print(" A, Current Limit: ");
    Serial.print(motor.current_limit, 3);
    Serial.println(" A");
#endif

    Serial.println("====================================");
    Serial.println();
}

// 设置震动参数
void MotorControl::setVibrationParams(float amplitude, float frequency, float torque) {
    vibration_amplitude = amplitude;
    vibration_frequency = frequency;
    vibration_torque = torque;

    DEBUG_PRINT("Vibration params set - Amplitude: ");
    DEBUG_PRINT(amplitude);
    DEBUG_PRINT(" rad, Frequency: ");
    DEBUG_PRINT(frequency);
    DEBUG_PRINT(" Hz, Torque: ");
    DEBUG_PRINTLN(torque);
}

// 获取震动参数
void MotorControl::getVibrationParams(float& amplitude, float& frequency, float& torque) {
    amplitude = vibration_amplitude;
    frequency = vibration_frequency;
    torque = vibration_torque;
}

// 更新震动（在loop中调用）
void MotorControl::updateVibration() {
    if (control_mode != 3) return;

    unsigned long current_time = millis();
    float dt = (current_time - vibration_last_time) / 1000.0f;
    vibration_last_time = current_time;

    // 更新相位
    vibration_phase += 2.0f * PI * vibration_frequency * dt;
    if (vibration_phase > 2.0f * PI) {
        vibration_phase -= 2.0f * PI;
    }

    // 计算目标角度（正弦波）
    float target = vibration_amplitude * sin(vibration_phase);

    // 设置扭矩限制
#if CURRENT_SENSE_TYPE > 0
    motor.current_limit = vibration_torque;
#else
    motor.voltage_limit = vibration_torque;
#endif

    // 应用目标
    motor.move(target);
}

