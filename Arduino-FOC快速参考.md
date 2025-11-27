# Arduino-FOC 快速参考手册

## 🚀 快速开始

### 基本代码结构
```cpp
#include <SimpleFOC.h>

// 1. 创建对象
Encoder sensor = Encoder(2, 3, 2048);
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);
BLDCMotor motor = BLDCMotor(11);

void doA(){sensor.handleA();}
void doB(){sensor.handleB();}

void setup() {
    // 2. 初始化传感器
    sensor.init();
    sensor.enableInterrupts(doA, doB);
    motor.linkSensor(&sensor);
    
    // 3. 初始化驱动器
    driver.voltage_power_supply = 12;
    driver.init();
    motor.linkDriver(&driver);
    
    // 4. 配置电机
    motor.controller = MotionControlType::velocity;
    motor.PID_velocity.P = 0.2;
    motor.PID_velocity.I = 20;
    motor.velocity_limit = 50;
    
    // 5. 初始化
    motor.init();
    motor.initFOC();
}

void loop() {
    motor.loopFOC();  // 尽可能快
    motor.move(10);   // 目标值
}
```

---

## 📋 核心类构造函数

| 类名 | 构造函数 | 示例 |
|------|----------|------|
| **BLDCMotor** | `BLDCMotor(pole_pairs, R, KV)` | `BLDCMotor motor(11, 10.5, 120);` |
| **StepperMotor** | `StepperMotor(pole_pairs, R, KV)` | `StepperMotor motor(50);` |
| **BLDCDriver3PWM** | `BLDCDriver3PWM(A, B, C, EN)` | `BLDCDriver3PWM driver(9, 5, 6, 8);` |
| **BLDCDriver6PWM** | `BLDCDriver6PWM(AH, AL, BH, BL, CH, CL, EN)` | `BLDCDriver6PWM driver(9, 10, 5, 6, 3, 11, 8);` |
| **Encoder** | `Encoder(A, B, CPR, INDEX)` | `Encoder sensor(2, 3, 2048);` |
| **MagneticSensorSPI** | `MagneticSensorSPI(CS, bits, reg)` | `MagneticSensorSPI sensor(10, 14, 0x3FFF);` |
| **MagneticSensorI2C** | `MagneticSensorI2C(addr)` | `MagneticSensorI2C sensor(0x36);` |
| **HallSensor** | `HallSensor(A, B, C, pp)` | `HallSensor sensor(2, 3, 4, 11);` |
| **InlineCurrentSense** | `InlineCurrentSense(R, gain, A, B, C)` | `InlineCurrentSense cs(0.01, 50, A0, A2);` |
| **LowsideCurrentSense** | `LowsideCurrentSense(R, gain, A, B, C)` | `LowsideCurrentSense cs(0.01, 50, A0, A1, A2);` |

---

## 🎮 控制模式枚举

### MotionControlType (运动控制)
```cpp
motor.controller = MotionControlType::torque;           // 扭矩控制
motor.controller = MotionControlType::velocity;         // 速度控制
motor.controller = MotionControlType::angle;            // 位置控制
motor.controller = MotionControlType::velocity_openloop; // 开环速度
motor.controller = MotionControlType::angle_openloop;    // 开环位置
```

### TorqueControlType (扭矩控制)
```cpp
motor.torque_controller = TorqueControlType::voltage;     // 电压模式
motor.torque_controller = TorqueControlType::dc_current;  // DC电流模式
motor.torque_controller = TorqueControlType::foc_current; // FOC电流模式
```

### FOCModulationType (调制方式)
```cpp
motor.foc_modulation = FOCModulationType::SinePWM;        // 正弦PWM
motor.foc_modulation = FOCModulationType::SpaceVectorPWM; // 空间矢量PWM
motor.foc_modulation = FOCModulationType::Trapezoid_120;  // 梯形120°
motor.foc_modulation = FOCModulationType::Trapezoid_150;  // 梯形150°
```

---

## ⚙️ 常用配置参数

### 电机参数
```cpp
motor.pole_pairs = 11;              // 极对数
motor.phase_resistance = 10.5;      // 相电阻(Ω)
motor.KV_rating = 120;              // KV值(rpm/V)
motor.voltage_limit = 12;           // 电压限制(V)
motor.current_limit = 2;            // 电流限制(A)
motor.velocity_limit = 50;          // 速度限制(rad/s)
```

### 速度PID
```cpp
motor.PID_velocity.P = 0.2;         // 比例
motor.PID_velocity.I = 20;          // 积分
motor.PID_velocity.D = 0.001;       // 微分
motor.PID_velocity.limit = 12;      // 输出限制
motor.PID_velocity.output_ramp = 1000; // 斜坡限制
motor.LPF_velocity.Tf = 0.01;       // 滤波时间常数
```

### 位置P控制
```cpp
motor.P_angle.P = 20;               // 比例
motor.P_angle.limit = 50;           // 输出限制
motor.P_angle.output_ramp = 10000;  // 斜坡限制
```

### 电流PID (FOC模式)
```cpp
motor.PID_current_q.P = 5;
motor.PID_current_q.I = 300;
motor.PID_current_d.P = 5;
motor.PID_current_d.I = 300;
motor.LPF_current_q.Tf = 0.005;
motor.LPF_current_d.Tf = 0.005;
```

### 驱动器参数
```cpp
driver.pwm_frequency = 20000;       // PWM频率(Hz)
driver.voltage_power_supply = 12;   // 电源电压(V)
driver.voltage_limit = 12;          // 电压限制(V)
```

---

## 🔧 核心函数

### 初始化函数
```cpp
sensor.init();                      // 初始化传感器
driver.init();                      // 初始化驱动器，返回1成功
current_sense.init();               // 初始化电流传感器，返回1成功
motor.init();                       // 初始化电机
motor.initFOC();                    // 初始化FOC，返回1成功
```

### 链接函数
```cpp
motor.linkSensor(&sensor);          // 链接传感器
motor.linkDriver(&driver);          // 链接驱动器
motor.linkCurrentSense(&current_sense); // 链接电流传感器
current_sense.linkDriver(&driver);  // 电流传感器链接驱动器
```

### 实时控制函数
```cpp
motor.loopFOC();                    // FOC算法循环（高频）
motor.move(target);                 // 运动控制（target可选）
motor.enable();                     // 使能电机
motor.disable();                    // 禁用电机
```

### 读取函数
```cpp
float angle = motor.shaft_angle;           // 读取角度(rad)
float velocity = motor.shaft_velocity;     // 读取速度(rad/s)
float angle = sensor.getAngle();           // 传感器角度
float velocity = sensor.getVelocity();     // 传感器速度
PhaseCurrent_s i = cs.getPhaseCurrents();  // 相电流
float idc = cs.getDCCurrent();             // DC电流
DQCurrent_s idq = cs.getFOCCurrents(angle); // DQ电流
```

---

## 📊 单位换算

| 物理量 | 库单位 | 换算 |
|--------|--------|------|
| 角度 | 弧度(rad) | 2π rad = 360° = 1转 |
| 速度 | rad/s | 2π rad/s = 60 RPM = 1转/秒 |
| 电流 | 安培(A) | - |
| 电压 | 伏特(V) | - |
| 电阻 | 欧姆(Ω) | - |

**常用换算：**
- RPM → rad/s: `rad_s = RPM * 2 * PI / 60`
- rad/s → RPM: `RPM = rad_s * 60 / (2 * PI)`
- 度 → 弧度: `rad = deg * PI / 180`

---

## 🐛 调试工具

### 启用调试
```cpp
Serial.begin(115200);
SimpleFOCDebug::enable(&Serial);    // 详细调试信息
motor.useMonitoring(Serial);        // 启用监控
```

### 监控输出
```cpp
motor.monitor();                    // 输出状态（在loop中）
```

### Commander命令接口
```cpp
Commander commander = Commander(Serial);
void doMotor(char* cmd) { commander.motor(&motor, cmd); }

void setup() {
    commander.add('M', doMotor, "motor");
}

void loop() {
    commander.run();
}
```

**常用命令：**
- `M` - 获取电机状态
- `MP0.5` - 设置速度PID的P=0.5
- `MV20` - 设置速度限制=20
- `T10` - 设置目标值=10

---

## ⚡ 性能参考

### loopFOC() 执行时间

| MCU | 电压模式 | DC电流 | FOC电流 |
|-----|----------|--------|---------|
| Arduino UNO | 700μs | 1.2ms | 1.5ms |
| ESP32 | 100μs | 200μs | 300μs |
| STM32 | 100μs | 150μs | 200μs |

**建议：** loopFOC() 调用频率 > 1kHz

---

## 📌 常见问题

1. **电机不转** → 检查极对数、传感器方向、电源电压
2. **initFOC失败** → 启用调试、检查传感器、确保电机可转动
3. **速度不稳** → 调整PID、增加滤波、检查电源
4. **电流检测失败** → 检查ADC引脚、增益、PWM定时器

---

**文档版本**: v1.0  
**适用库版本**: Arduino-FOC v2.3.x  
**更新日期**: 2025-11-27
