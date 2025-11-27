# Arduino-FOC 库功能和函数详细分析

## 📚 库概述

**Arduino SimpleFOC** 是一个开源的磁场定向控制(Field Oriented Control, FOC)库，用于控制无刷直流电机(BLDC)和步进电机。

### 主要特点
- ✅ 支持多种电机类型：BLDC电机、步进电机、混合步进电机
- ✅ 支持多种位置传感器：编码器、磁传感器、霍尔传感器
- ✅ 支持多种驱动器：3PWM、6PWM驱动器
- ✅ 支持电流检测：在线电流检测、低侧电流检测、高侧电流检测
- ✅ 跨平台支持：Arduino、ESP32、STM32、Teensy等
- ✅ 易于配置和调试

---

## 🔧 核心类和功能模块

### 1. 电机类 (Motor Classes)

#### 1.1 BLDCMotor - 无刷直流电机类

**构造函数：**
```cpp
BLDCMotor(int pole_pairs, float phase_resistance = NOT_SET, float KV_rating = NOT_SET)
```

**参数说明：**
- `pole_pairs` - 电机极对数（必需）
- `phase_resistance` - 相电阻值，单位：欧姆（可选）
- `KV_rating` - 电机KV值，单位：rpm/V（可选）

**主要属性：**
```cpp
// 控制模式
MotionControlType controller;  // 运动控制类型
TorqueControlType torque_controller;  // 扭矩控制类型
FOCModulationType foc_modulation;  // FOC调制类型

// 传感器对齐参数
float voltage_sensor_align;  // 传感器对齐电压，默认3V
float velocity_index_search;  // 编码器索引搜索速度，默认1 rad/s
float sensor_offset;  // 传感器偏移量，单位：弧度

// 电机参数
float phase_resistance;  // 相电阻，单位：欧姆
float phase_inductance;  // 相电感，单位：亨利
float KV_rating;  // KV值，单位：rpm/V

// 限制参数
float voltage_limit;  // 电压限制，单位：伏特
float current_limit;  // 电流限制，单位：安培
float velocity_limit;  // 速度限制，单位：rad/s

// PID控制器
PIDController PID_velocity;  // 速度PID控制器
PController P_angle;  // 角度P控制器
LowPassFilter LPF_velocity;  // 速度低通滤波器

// 状态变量
float shaft_angle;  // 电机轴角度，单位：弧度
float shaft_velocity;  // 电机轴速度，单位：rad/s
float target;  // 目标值
float voltage_q;  // q轴电压
```

**主要方法：**

```cpp
// 初始化和配置
void init();  // 初始化电机
int initFOC();  // 初始化FOC算法，返回1成功，0失败

// 链接外部组件
void linkSensor(Sensor* sensor);  // 链接位置传感器
void linkDriver(BLDCDriver* driver);  // 链接驱动器
void linkCurrentSense(CurrentSense* current_sense);  // 链接电流传感器

// 实时控制
void loopFOC();  // FOC算法循环，需要尽可能快地调用
void move(float target = NOT_SET);  // 运动控制循环

// 监控和调试
void useMonitoring(Stream& serial);  // 启用监控功能
void monitor();  // 输出监控信息

// 电机特性测量
void characteriseMotor(float voltage);  // 测量电机相电阻和电感
```

**返回值说明：**
- `init()` - 无返回值
- `initFOC()` - 返回 `int`：1表示成功，0表示失败
- `loopFOC()` - 无返回值
- `move()` - 无返回值
- `getAngle()` - 返回 `float`：当前角度（弧度）
- `getVelocity()` - 返回 `float`：当前速度（rad/s）

---

#### 1.2 StepperMotor - 步进电机类

**构造函数：**
```cpp
StepperMotor(int pole_pairs, float phase_resistance = NOT_SET, float KV_rating = NOT_SET)
```

参数和方法与 BLDCMotor 类似，但针对步进电机优化。

---

### 2. 驱动器类 (Driver Classes)

#### 2.1 BLDCDriver3PWM - 3路PWM驱动器

**构造函数：**
```cpp
BLDCDriver3PWM(int phA, int phB, int phC, int en = NOT_SET)
BLDCDriver3PWM(int phA, int phB, int phC, int enA, int enB, int enC)
```

**参数说明：**
- `phA, phB, phC` - A、B、C相的PWM引脚
- `en` - 使能引脚（可选）
- `enA, enB, enC` - 各相独立使能引脚（可选）

**主要属性：**
```cpp
float pwm_frequency;  // PWM频率，单位：Hz，默认20kHz
float voltage_power_supply;  // 电源电压，单位：伏特
float voltage_limit;  // 电压限制，单位：伏特
```

**主要方法：**
```cpp
int init();  // 初始化驱动器，返回1成功，0失败
void enable();  // 使能驱动器
void disable();  // 禁用驱动器
void setPwm(float Ua, float Ub, float Uc);  // 设置三相PWM电压
void setPhaseState(PhaseState sa, PhaseState sb, PhaseState sc);  // 设置相状态
```

**返回值说明：**
- `init()` - 返回 `int`：1表示成功，0表示失败
- 其他方法无返回值

---

#### 2.2 BLDCDriver6PWM - 6路PWM驱动器

**构造函数：**
```cpp
BLDCDriver6PWM(int phA_h, int phA_l, int phB_h, int phB_l, int phC_h, int phC_l, int en = NOT_SET)
```

**参数说明：**
- `phX_h` - X相高侧PWM引脚
- `phX_l` - X相低侧PWM引脚
- `en` - 使能引脚（可选）

方法与 BLDCDriver3PWM 类似。

---

### 3. 位置传感器类 (Position Sensor Classes)

#### 3.1 Encoder - 编码器

**构造函数：**
```cpp
Encoder(int encA, int encB, int cpr, int index = NOT_SET)
```

**参数说明：**
- `encA, encB` - 编码器A、B通道引脚
- `cpr` - 每转脉冲数（CPR）
- `index` - 索引引脚（可选）

**主要属性：**
```cpp
Quadrature quadrature;  // 正交模式：ON或OFF
Pullup pullup;  // 上拉电阻：USE_EXTERN或USE_INTERN
float min_elapsed_time;  // 最小采样时间，默认100us
```

**主要方法：**
```cpp
void init();  // 初始化编码器
void enableInterrupts(void (*doA)(), void (*doB)(), void (*doIndex)() = nullptr);  // 启用中断
void handleA();  // A通道中断处理
void handleB();  // B通道中断处理
void handleIndex();  // 索引通道中断处理
float getAngle();  // 获取角度，返回弧度
float getVelocity();  // 获取速度，返回rad/s
void update();  // 更新传感器值
```

**返回值说明：**
- `getAngle()` - 返回 `float`：当前角度（弧度）
- `getVelocity()` - 返回 `float`：当前速度（rad/s）

---

#### 3.2 MagneticSensorSPI - SPI磁传感器

**构造函数：**
```cpp
MagneticSensorSPI(int cs, float bit_resolution, int angle_register)
```

**参数说明：**
- `cs` - 片选引脚
- `bit_resolution` - 位分辨率（如14位传感器为14）
- `angle_register` - 角度寄存器地址

**主要方法：**
```cpp
void init();  // 初始化传感器
float getAngle();  // 获取角度，返回弧度
float getVelocity();  // 获取速度，返回rad/s
```

**示例：**
```cpp
// AS5147传感器，14位分辨率
MagneticSensorSPI sensor = MagneticSensorSPI(10, 14, 0x3FFF);
```

---

#### 3.3 MagneticSensorI2C - I2C磁传感器

**构造函数：**
```cpp
MagneticSensorI2C(uint8_t chip_address)
```

**参数说明：**
- `chip_address` - I2C芯片地址（如AS5600为0x36）

**主要方法：**
```cpp
void init();  // 初始化传感器
float getAngle();  // 获取角度
float getVelocity();  // 获取速度
```

---

#### 3.4 HallSensor - 霍尔传感器

**构造函数：**
```cpp
HallSensor(int hallA, int hallB, int hallC, int pole_pairs)
```

**参数说明：**
- `hallA, hallB, hallC` - 霍尔传感器A、B、C引脚
- `pole_pairs` - 电机极对数

**主要方法：**
```cpp
void init();  // 初始化传感器
void enableInterrupts(void (*doA)(), void (*doB)(), void (*doC)());  // 启用中断
void handleA();  // A通道中断处理
void handleB();  // B通道中断处理
void handleC();  // C通道中断处理
float getAngle();  // 获取角度
float getVelocity();  // 获取速度
```

---

### 4. 电流检测类 (Current Sensing Classes)

#### 4.1 InlineCurrentSense - 在线电流检测

**构造函数：**
```cpp
InlineCurrentSense(float shunt_resistor, float gain, int pinA, int pinB, int pinC = NOT_SET)
InlineCurrentSense(float mVpA, int pinA, int pinB, int pinC = NOT_SET)
```

**参数说明：**
- `shunt_resistor` - 分流电阻值，单位：欧姆
- `gain` - 放大器增益
- `mVpA` - 毫伏/安培比率（用于霍尔电流传感器如ACS712）
- `pinA, pinB, pinC` - A、B、C相ADC引脚（C相可选）

**主要属性：**
```cpp
float gain_a, gain_b, gain_c;  // 各相增益
bool skip_align;  // 跳过对齐，默认false
```

**主要方法：**
```cpp
int init();  // 初始化电流传感器，返回1成功，0失败
void linkDriver(BLDCDriver* driver);  // 链接驱动器
int driverAlign(float voltage);  // 与驱动器对齐，返回对齐状态
PhaseCurrent_s getPhaseCurrents();  // 获取相电流
float getDCCurrent();  // 获取DC电流幅值
float getDCCurrent(float angle);  // 获取带符号的DC电流
DQCurrent_s getFOCCurrents(float angle);  // 获取DQ电流
```

**返回值说明：**
- `init()` - 返回 `int`：1成功，0失败
- `driverAlign()` - 返回 `int`：0失败，1成功无变化，2成功引脚重配，3成功增益反转，4成功引脚和增益都改变
- `getPhaseCurrents()` - 返回 `PhaseCurrent_s` 结构体，包含 `a, b, c` 三个float值
- `getDCCurrent()` - 返回 `float`：电流值（安培）
- `getFOCCurrents()` - 返回 `DQCurrent_s` 结构体，包含 `d, q` 两个float值

---

#### 4.2 LowsideCurrentSense - 低侧电流检测

**构造函数：**
```cpp
LowsideCurrentSense(float shunt_resistor, float gain, int pinA, int pinB, int pinC = NOT_SET)
```

参数和方法与 InlineCurrentSense 类似，但用于低侧电流检测架构。

---

### 5. 控制模式 (Control Modes)

#### 5.1 运动控制类型 (MotionControlType)

```cpp
enum MotionControlType {
    torque,              // 扭矩控制
    velocity,            // 速度控制（闭环）
    angle,               // 角度/位置控制（闭环）
    velocity_openloop,   // 速度开环控制
    angle_openloop       // 角度开环控制
};
```

**使用方法：**
```cpp
motor.controller = MotionControlType::velocity;  // 设置为速度控制模式
```

---

#### 5.2 扭矩控制类型 (TorqueControlType)

```cpp
enum TorqueControlType {
    voltage,      // 电压模式（默认，不需要电流传感器）
    dc_current,   // DC电流模式（需要电流传感器）
    foc_current   // FOC电流模式（需要电流传感器，最精确）
};
```

**使用方法：**
```cpp
motor.torque_controller = TorqueControlType::foc_current;  // 设置为FOC电流模式
```

---

#### 5.3 FOC调制类型 (FOCModulationType)

```cpp
enum FOCModulationType {
    SinePWM,          // 正弦PWM调制（默认）
    SpaceVectorPWM,   // 空间矢量PWM调制（效率更高）
    Trapezoid_120,    // 梯形120度换相（适合霍尔传感器）
    Trapezoid_150     // 梯形150度换相
};
```

**使用方法：**
```cpp
motor.foc_modulation = FOCModulationType::SpaceVectorPWM;  // 设置为空间矢量PWM
```

---

### 6. PID控制器 (PID Controller)

#### 6.1 PIDController - PID控制器类

**主要属性：**
```cpp
float P;  // 比例增益
float I;  // 积分增益
float D;  // 微分增益
float output_ramp;  // 输出斜坡限制，单位：值/秒
float limit;  // 输出限制
```

**使用示例：**
```cpp
// 配置速度PID控制器
motor.PID_velocity.P = 0.2;
motor.PID_velocity.I = 20;
motor.PID_velocity.D = 0.001;
motor.PID_velocity.output_ramp = 1000;  // 1000 rad/s²
motor.PID_velocity.limit = 12;  // 12V限制
```

---

#### 6.2 PController - P控制器类

**主要属性：**
```cpp
float P;  // 比例增益
float output_ramp;  // 输出斜坡限制
float limit;  // 输出限制
```

**使用示例：**
```cpp
// 配置角度P控制器
motor.P_angle.P = 20;
motor.P_angle.output_ramp = 10000;
motor.P_angle.limit = 50;  // 50 rad/s限制
```

---

#### 6.3 LowPassFilter - 低通滤波器

**主要属性：**
```cpp
float Tf;  // 滤波时间常数，单位：秒
```

**使用示例：**
```cpp
// 配置速度低通滤波器
motor.LPF_velocity.Tf = 0.01;  // 10ms时间常数
```

---

### 7. 通信和监控 (Communication & Monitoring)

#### 7.1 Commander - 命令接口

**构造函数：**
```cpp
Commander(Stream& serial)
```

**参数说明：**
- `serial` - 串口对象（如Serial）

**主要方法：**
```cpp
void add(char id, void (*callback)(char*), const char* label = nullptr);  // 添加命令
void motor(BLDCMotor* motor, char* cmd);  // 电机完整配置命令
void motion(BLDCMotor* motor, char* cmd);  // 运动控制命令
void scalar(float* value, char* cmd);  // 标量变量命令
void pid(PIDController* pid, char* cmd);  // PID配置命令
void lpf(LowPassFilter* lpf, char* cmd);  // 低通滤波器配置命令
void run();  // 运行命令解析器
```

**使用示例：**
```cpp
Commander commander = Commander(Serial);

void doTarget(char* cmd) {
    commander.scalar(&motor.target, cmd);
}

void doMotor(char* cmd) {
    commander.motor(&motor, cmd);
}

void setup() {
    Serial.begin(115200);
    commander.add('T', doTarget, "target");
    commander.add('M', doMotor, "motor");
}

void loop() {
    commander.run();  // 处理串口命令
}
```

**命令格式：**
- `T10` - 设置目标值为10
- `M` - 获取电机状态
- `MP0.5` - 设置速度PID的P值为0.5
- `MV20` - 设置速度限制为20

---

#### 7.2 SimpleFOCDebug - 调试输出

**主要方法：**
```cpp
static void enable(Stream* serial);  // 启用调试输出
static void disable();  // 禁用调试输出
```

**使用示例：**
```cpp
void setup() {
    Serial.begin(115200);
    SimpleFOCDebug::enable(&Serial);  // 启用详细调试信息

    driver.init();
    motor.init();
    current_sense.init();
    motor.initFOC();
}
```

**调试输出内容：**
- 驱动器初始化详情（定时器、通道、引脚配置）
- 电流传感器初始化详情（ADC配置、校准结果）
- 电机初始化详情（传感器对齐、电气角度偏移）
- FOC初始化详情（相位对齐状态）

---

### 8. 数据结构 (Data Structures)

#### 8.1 PhaseCurrent_s - 相电流结构

```cpp
struct PhaseCurrent_s {
    float a;  // A相电流，单位：安培
    float b;  // B相电流，单位：安培
    float c;  // C相电流，单位：安培（可能为0）
};
```

---

#### 8.2 DQCurrent_s - DQ电流结构

```cpp
struct DQCurrent_s {
    float d;  // D轴电流（磁通电流），单位：安培
    float q;  // Q轴电流（扭矩电流），单位：安培
};
```

---

#### 8.3 PhaseState - 相状态枚举

```cpp
enum PhaseState {
    _ACTIVE,          // 激活状态
    _HIGH_Z,          // 高阻态
    _HIGH_IMPEDANCE   // 高阻态（同_HIGH_Z）
};
```

---

### 9. 单位系统 (Units)

| 物理量 | 单位 | 说明 | 转换 |
|--------|------|------|------|
| 位置/角度 | 弧度 (rad) | 电机和传感器位置 | 2π rad = 360° = 1转 |
| 速度 | 弧度/秒 (rad/s) | 电机和传感器速度 | 2π rad/s = 1转/秒 = 60 RPM |
| 扭矩/电流 | 安培 (A) | 电机扭矩或电流 | 1 Nm = Kt × A (Kt为扭矩常数) |
| 电压 | 伏特 (V) | 相电压 | - |
| 电阻 | 欧姆 (Ω) | 相电阻 | - |
| 电感 | 亨利 (H) | 相电感 | 通常以mH表示 |

---

### 10. 典型使用流程

#### 10.1 完整的FOC控制示例

```cpp
#include <SimpleFOC.h>

// 1. 创建传感器对象
MagneticSensorSPI sensor = MagneticSensorSPI(10, 14, 0x3FFF);

// 2. 创建驱动器对象
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);

// 3. 创建电流传感器对象（可选）
InlineCurrentSense current_sense = InlineCurrentSense(0.01, 50, A0, A2);

// 4. 创建电机对象
BLDCMotor motor = BLDCMotor(11);  // 11极对

// 5. 创建命令接口
Commander commander = Commander(Serial);
void doMotor(char* cmd) { commander.motor(&motor, cmd); }

void setup() {
    // 6. 初始化串口和调试
    Serial.begin(115200);
    SimpleFOCDebug::enable(&Serial);

    // 7. 初始化传感器
    sensor.init();
    motor.linkSensor(&sensor);

    // 8. 配置和初始化驱动器
    driver.voltage_power_supply = 12;
    driver.pwm_frequency = 20000;
    driver.init();
    motor.linkDriver(&driver);

    // 9. 配置和初始化电流传感器（可选）
    current_sense.linkDriver(&driver);
    current_sense.init();
    motor.linkCurrentSense(&current_sense);

    // 10. 配置电机参数
    motor.phase_resistance = 10.5;  // 10.5欧姆
    motor.KV_rating = 120;  // 120 rpm/V

    // 11. 配置控制模式
    motor.controller = MotionControlType::velocity;
    motor.torque_controller = TorqueControlType::foc_current;
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

    // 12. 配置PID参数
    motor.PID_velocity.P = 0.2;
    motor.PID_velocity.I = 20;
    motor.PID_velocity.D = 0.001;
    motor.LPF_velocity.Tf = 0.01;

    // 13. 配置限制
    motor.voltage_limit = 12;
    motor.current_limit = 2;
    motor.velocity_limit = 50;

    // 14. 初始化电机
    motor.init();

    // 15. 对齐传感器和启动FOC
    motor.initFOC();

    // 16. 添加命令
    commander.add('M', doMotor, "motor");

    Serial.println("Motor ready!");
}

void loop() {
    // 17. FOC算法循环（尽可能快）
    motor.loopFOC();

    // 18. 运动控制循环
    motor.move();

    // 19. 处理用户命令
    commander.run();

    // 20. 监控输出（可选，会降低性能）
    // motor.monitor();
}
```

---

### 11. 常用函数速查表

#### 11.1 电机控制函数

| 函数 | 参数 | 返回值 | 说明 |
|------|------|--------|------|
| `motor.init()` | 无 | void | 初始化电机硬件 |
| `motor.initFOC()` | 无 | int | 初始化FOC，返回1成功 |
| `motor.loopFOC()` | 无 | void | FOC算法循环，需高频调用 |
| `motor.move(target)` | float target | void | 运动控制，target可选 |
| `motor.disable()` | 无 | void | 禁用电机 |
| `motor.enable()` | 无 | void | 使能电机 |
| `motor.linkSensor(&sensor)` | Sensor* | void | 链接传感器 |
| `motor.linkDriver(&driver)` | BLDCDriver* | void | 链接驱动器 |
| `motor.linkCurrentSense(&cs)` | CurrentSense* | void | 链接电流传感器 |

---

#### 11.2 传感器函数

| 函数 | 参数 | 返回值 | 说明 |
|------|------|--------|------|
| `sensor.init()` | 无 | void | 初始化传感器 |
| `sensor.getAngle()` | 无 | float | 获取角度（弧度） |
| `sensor.getVelocity()` | 无 | float | 获取速度（rad/s） |
| `sensor.update()` | 无 | void | 更新传感器值 |
| `encoder.enableInterrupts(doA, doB)` | 函数指针 | void | 启用编码器中断 |
| `encoder.handleA()` | 无 | void | A通道中断处理 |
| `encoder.handleB()` | 无 | void | B通道中断处理 |

---

#### 11.3 驱动器函数

| 函数 | 参数 | 返回值 | 说明 |
|------|------|--------|------|
| `driver.init()` | 无 | int | 初始化驱动器，返回1成功 |
| `driver.enable()` | 无 | void | 使能驱动器 |
| `driver.disable()` | 无 | void | 禁用驱动器 |
| `driver.setPwm(Ua, Ub, Uc)` | float, float, float | void | 设置三相电压 |
| `driver.setPhaseState(sa, sb, sc)` | PhaseState × 3 | void | 设置相状态 |

---

#### 11.4 电流传感器函数

| 函数 | 参数 | 返回值 | 说明 |
|------|------|--------|------|
| `current_sense.init()` | 无 | int | 初始化，返回1成功 |
| `current_sense.linkDriver(&driver)` | BLDCDriver* | void | 链接驱动器 |
| `current_sense.getPhaseCurrents()` | 无 | PhaseCurrent_s | 获取相电流 |
| `current_sense.getDCCurrent()` | 无 | float | 获取DC电流幅值 |
| `current_sense.getDCCurrent(angle)` | float | float | 获取带符号DC电流 |
| `current_sense.getFOCCurrents(angle)` | float | DQCurrent_s | 获取DQ电流 |
| `current_sense.driverAlign(voltage)` | float | int | 对齐驱动器 |

---

#### 11.5 监控和调试函数

| 函数 | 参数 | 返回值 | 说明 |
|------|------|--------|------|
| `motor.useMonitoring(Serial)` | Stream& | void | 启用监控 |
| `motor.monitor()` | 无 | void | 输出监控信息 |
| `SimpleFOCDebug::enable(&Serial)` | Stream* | void | 启用调试输出 |
| `SimpleFOCDebug::disable()` | 无 | void | 禁用调试输出 |
| `commander.add(id, callback, label)` | char, 函数指针, char* | void | 添加命令 |
| `commander.run()` | 无 | void | 运行命令解析 |

---

### 12. 配置参数参考

#### 12.1 电机配置参数

```cpp
// 基本参数
motor.pole_pairs = 11;              // 极对数
motor.phase_resistance = 10.5;      // 相电阻（欧姆）
motor.phase_inductance = 0.0005;    // 相电感（亨利）
motor.KV_rating = 120;              // KV值（rpm/V）

// 控制模式
motor.controller = MotionControlType::velocity;
motor.torque_controller = TorqueControlType::foc_current;
motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

// 传感器对齐
motor.voltage_sensor_align = 3;     // 对齐电压（伏特）
motor.velocity_index_search = 1;    // 索引搜索速度（rad/s）
motor.sensor_offset = 0;            // 传感器偏移（弧度）
motor.sensor_direction = Direction::CW;  // 传感器方向
motor.zero_electric_offset = 0;     // 电气零点偏移

// 限制参数
motor.voltage_limit = 12;           // 电压限制（伏特）
motor.current_limit = 2;            // 电流限制（安培）
motor.velocity_limit = 50;          // 速度限制（rad/s）

// 速度PID
motor.PID_velocity.P = 0.2;
motor.PID_velocity.I = 20;
motor.PID_velocity.D = 0.001;
motor.PID_velocity.output_ramp = 1000;
motor.PID_velocity.limit = 12;

// 角度P控制
motor.P_angle.P = 20;
motor.P_angle.output_ramp = 10000;
motor.P_angle.limit = 50;

// 速度滤波
motor.LPF_velocity.Tf = 0.01;       // 时间常数（秒）

// 电流PID（FOC电流模式）
motor.PID_current_q.P = 5;
motor.PID_current_q.I = 300;
motor.PID_current_q.D = 0;
motor.PID_current_q.limit = 12;
motor.PID_current_q.output_ramp = 0;

motor.PID_current_d.P = 5;
motor.PID_current_d.I = 300;
motor.PID_current_d.D = 0;
motor.PID_current_d.limit = 12;
motor.PID_current_d.output_ramp = 0;

// 电流滤波
motor.LPF_current_q.Tf = 0.005;
motor.LPF_current_d.Tf = 0.005;

// 运动控制降采样
motor.motion_downsample = 0;        // 0=禁用，>0=降采样倍数
```

---

#### 12.2 驱动器配置参数

```cpp
// 3PWM驱动器
driver.pwm_frequency = 20000;       // PWM频率（Hz）
driver.voltage_power_supply = 12;   // 电源电压（伏特）
driver.voltage_limit = 12;          // 电压限制（伏特）
driver.dead_zone = 0.02;            // 死区时间（微秒，仅6PWM）
```

---

#### 12.3 传感器配置参数

```cpp
// 编码器
encoder.quadrature = Quadrature::ON;     // 正交模式
encoder.pullup = Pullup::USE_EXTERN;     // 上拉电阻
encoder.min_elapsed_time = 0.0001;       // 最小采样时间（秒）

// 磁传感器
// （通常无需额外配置）
```

---

#### 12.4 电流传感器配置参数

```cpp
// 在线电流传感器
current_sense.gain_a = 1.0 / shunt / gain;  // A相增益
current_sense.gain_b = 1.0 / shunt / gain;  // B相增益
current_sense.gain_c = 1.0 / shunt / gain;  // C相增益
current_sense.skip_align = false;           // 跳过对齐
```

---

### 13. 性能优化建议

#### 13.1 执行时间参考

不同MCU上 `motor.loopFOC()` 的执行时间：

| MCU | 电压模式 | DC电流模式 | FOC电流模式 |
|-----|----------|------------|-------------|
| Arduino UNO | ~700 μs | ~1.2 ms | ~1.5 ms |
| ESP32 | ~100 μs | ~200 μs | ~300 μs |
| STM32 Bluepill | ~200 μs | ~500 μs | ~700 μs |
| STM32 Nucleo | ~100 μs | ~150 μs | ~200 μs |

**建议：**
- 尽可能快地调用 `motor.loopFOC()`，理想频率 > 1kHz
- 避免在 `loop()` 中使用 `delay()`
- 使用 `motor.motion_downsample` 降低运动控制频率

---

#### 13.2 PWM频率选择

```cpp
// 推荐PWM频率
driver.pwm_frequency = 20000;  // 20kHz（通用推荐）
driver.pwm_frequency = 25000;  // 25kHz（STM32）
driver.pwm_frequency = 30000;  // 30kHz（ESP32）
```

**注意事项：**
- 频率过高：ADC采样时间不足（低侧电流检测）
- 频率过低：运行噪音大、效率低
- 推荐范围：15-30 kHz

---

#### 13.3 电流检测优化

**低侧电流检测要求：**
- 所有PWM引脚必须在同一定时器上
- PWM频率不宜过高（推荐20kHz）
- 确保ADC采样在低侧开关导通时进行

**在线电流检测优化：**
- 可使用任意PWM引脚
- 对PWM频率要求较低
- 精度更高但硬件成本高

---

### 14. 故障排查指南

#### 14.1 常见问题

**问题1：电机不转或抖动**
- 检查极对数设置是否正确
- 检查传感器方向和偏移
- 检查电源电压是否足够
- 尝试增加 `motor.voltage_sensor_align`

**问题2：initFOC() 失败**
- 启用调试：`SimpleFOCDebug::enable(&Serial)`
- 检查传感器连接和初始化
- 检查驱动器引脚配置
- 确保电机可以自由转动

**问题3：电流检测不工作**
- 检查ADC引脚是否正确
- 检查增益和分流电阻值
- 使用 `current_sense.driverAlign()` 检查对齐
- 确保PWM引脚在同一定时器（低侧检测）

**问题4：速度不稳定**
- 调整PID参数（降低P，增加I）
- 增加速度滤波器时间常数
- 检查电源是否稳定
- 降低速度限制

---

#### 14.2 调试技巧

```cpp
// 1. 启用详细调试
SimpleFOCDebug::enable(&Serial);

// 2. 使用监控功能
motor.useMonitoring(Serial);
motor.monitor();  // 在loop中调用

// 3. 测试传感器
Serial.println(sensor.getAngle());
Serial.println(sensor.getVelocity());

// 4. 测试电流传感器
PhaseCurrent_s current = current_sense.getPhaseCurrents();
Serial.print(current.a); Serial.print("\t");
Serial.print(current.b); Serial.print("\t");
Serial.println(current.c);

// 5. 测试驱动器
driver.setPwm(3, 3, 3);  // 设置固定电压测试

// 6. 跳过对齐（如果已知参数）
motor.zero_electric_offset = 2.15;
motor.sensor_direction = Direction::CW;
current_sense.skip_align = true;
```

---

### 15. 示例代码库

#### 15.1 速度控制示例

```cpp
#include <SimpleFOC.h>

MagneticSensorSPI sensor = MagneticSensorSPI(10, 14, 0x3FFF);
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);
BLDCMotor motor = BLDCMotor(11);

void setup() {
    Serial.begin(115200);

    sensor.init();
    motor.linkSensor(&sensor);

    driver.voltage_power_supply = 12;
    driver.init();
    motor.linkDriver(&driver);

    motor.controller = MotionControlType::velocity;
    motor.PID_velocity.P = 0.2;
    motor.PID_velocity.I = 20;
    motor.velocity_limit = 50;

    motor.init();
    motor.initFOC();
}

void loop() {
    motor.loopFOC();
    motor.move(10);  // 目标速度：10 rad/s
}
```

---

#### 15.2 位置控制示例

```cpp
motor.controller = MotionControlType::angle;
motor.P_angle.P = 20;
motor.velocity_limit = 20;

void loop() {
    motor.loopFOC();
    motor.move(3.14);  // 目标位置：π弧度
}
```

---

#### 15.3 扭矩控制示例

```cpp
motor.controller = MotionControlType::torque;
motor.torque_controller = TorqueControlType::voltage;

void loop() {
    motor.loopFOC();
    motor.move(5);  // 目标电压：5V
}
```

---

#### 15.4 开环速度控制示例

```cpp
motor.controller = MotionControlType::velocity_openloop;
motor.voltage_limit = 3;

void loop() {
    motor.move(5);  // 目标速度：5 rad/s（无传感器）
}
```

---

### 16. 硬件兼容性

#### 16.1 支持的开发板

| 开发板 | PWM频率 | 电流检测 | 状态 |
|--------|---------|----------|------|
| Arduino UNO | 4/32 kHz | ✅ | 完全支持 |
| Arduino Mega | 4/32 kHz | ✅ | 完全支持 |
| Arduino Due | 可配置 | ✅ | 完全支持 |
| ESP32 | 可配置 | ✅ | 完全支持 |
| ESP8266 | 可配置 | ❌ | 仅无电流检测 |
| STM32 Nucleo | 可配置 | ✅ | 完全支持 |
| STM32 Bluepill | 可配置 | ✅ | 完全支持 |
| Teensy 3.x/4.x | 可配置 | ✅ | 完全支持 |
| SAMD21/51 | 可配置 | ✅ | 完全支持 |
| Raspberry Pi Pico | 可配置 | ✅ | 完全支持 |

---

#### 16.2 支持的驱动板

- **SimpleFOCShield** - 官方开发板
- **SimpleFOCMini** - 小型版本
- **L6234 Breakout Board**
- **DRV8302/DRV8305** - TI驱动芯片
- **HMBGC V2.2** - 云台控制器
- **X-NUCLEO-IHM07M1** - ST官方板
- **自定义3PWM/6PWM驱动器**

---

### 17. 资源链接

- 📖 **官方文档**: https://docs.simplefoc.com/
- 💻 **GitHub仓库**: https://github.com/simplefoc/Arduino-FOC
- 💬 **社区论坛**: https://community.simplefoc.com/
- 🛒 **官方商店**: https://simplefoc.com/shop
- 📺 **视频教程**: YouTube搜索 "SimpleFOC"

---

### 18. 版本信息

**当前最新版本**: v2.3.5 (2024)

**主要更新：**
- ✅ ESP32 C6 MCPWM支持
- ✅ 混合步进电机支持
- ✅ 电机特性测量功能
- ✅ SAMD21低侧电流检测
- ✅ RP2350支持
- ✅ STM32 H7低侧电流检测

---

## 📝 总结

Arduino-FOC库提供了完整的FOC控制解决方案，主要包括：

1. **核心类**：BLDCMotor、StepperMotor
2. **驱动器**：BLDCDriver3PWM、BLDCDriver6PWM
3. **传感器**：Encoder、MagneticSensor、HallSensor
4. **电流检测**：InlineCurrentSense、LowsideCurrentSense
5. **控制器**：PID、P控制器、低通滤波器
6. **通信**：Commander、监控、调试

**关键函数调用顺序：**
1. 创建对象 → 2. 初始化硬件 → 3. 链接组件 → 4. 配置参数 → 5. motor.init() → 6. motor.initFOC() → 7. 循环调用 loopFOC() 和 move()

**性能关键点：**
- `loopFOC()` 需要高频调用（>1kHz）
- PWM频率推荐20kHz
- 合理配置PID参数
- 使用调试工具排查问题

---

**文档编写日期**: 2025-11-27
**适用库版本**: Arduino-FOC v2.3.x
**作者**: AI Assistant

