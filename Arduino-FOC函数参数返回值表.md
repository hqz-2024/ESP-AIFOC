# Arduino-FOC 函数参数和返回值详细对照表

## 📖 目录
1. [BLDCMotor 类](#bldcmotor-类)
2. [驱动器类](#驱动器类)
3. [传感器类](#传感器类)
4. [电流检测类](#电流检测类)
5. [控制器类](#控制器类)
6. [通信类](#通信类)

---

## BLDCMotor 类

### 构造函数
| 函数签名 | 参数说明 | 返回值 | 示例 |
|---------|---------|--------|------|
| `BLDCMotor(int pp)` | `pp`: 极对数 | - | `BLDCMotor motor(11);` |
| `BLDCMotor(int pp, float R)` | `pp`: 极对数<br>`R`: 相电阻(Ω) | - | `BLDCMotor motor(11, 10.5);` |
| `BLDCMotor(int pp, float R, float KV)` | `pp`: 极对数<br>`R`: 相电阻(Ω)<br>`KV`: KV值(rpm/V) | - | `BLDCMotor motor(11, 10.5, 120);` |

### 初始化方法
| 函数签名 | 参数说明 | 返回值 | 说明 |
|---------|---------|--------|------|
| `void init()` | 无 | void | 初始化电机硬件 |
| `int initFOC()` | 无 | `int`<br>1=成功<br>0=失败 | 初始化FOC算法，对齐传感器 |
| `int initFOC(float zero_offset, Direction dir)` | `zero_offset`: 电气零点偏移(rad)<br>`dir`: 传感器方向 | `int`<br>1=成功<br>0=失败 | 跳过对齐，使用已知参数 |

### 链接方法
| 函数签名 | 参数说明 | 返回值 | 说明 |
|---------|---------|--------|------|
| `void linkSensor(Sensor* sensor)` | `sensor`: 传感器对象指针 | void | 链接位置传感器 |
| `void linkDriver(BLDCDriver* driver)` | `driver`: 驱动器对象指针 | void | 链接驱动器 |
| `void linkCurrentSense(CurrentSense* cs)` | `cs`: 电流传感器对象指针 | void | 链接电流传感器 |

### 实时控制方法
| 函数签名 | 参数说明 | 返回值 | 说明 |
|---------|---------|--------|------|
| `void loopFOC()` | 无 | void | FOC算法循环，需高频调用(>1kHz) |
| `void move()` | 无 | void | 运动控制，使用motor.target |
| `void move(float target)` | `target`: 目标值<br>- 扭矩模式: 电压(V)或电流(A)<br>- 速度模式: 速度(rad/s)<br>- 位置模式: 角度(rad) | void | 运动控制，指定目标值 |
| `void enable()` | 无 | void | 使能电机 |
| `void disable()` | 无 | void | 禁用电机 |

### 监控和调试方法
| 函数签名 | 参数说明 | 返回值 | 说明 |
|---------|---------|--------|------|
| `void useMonitoring(Stream& serial)` | `serial`: 串口对象引用 | void | 启用监控功能 |
| `void monitor()` | 无 | void | 输出监控信息到串口 |

### 电机特性测量
| 函数签名 | 参数说明 | 返回值 | 说明 |
|---------|---------|--------|------|
| `void characteriseMotor(float voltage)` | `voltage`: 测试电压(V) | void | 测量相电阻和电感，结果输出到串口 |

---

## 驱动器类

### BLDCDriver3PWM

#### 构造函数
| 函数签名 | 参数说明 | 返回值 | 示例 |
|---------|---------|--------|------|
| `BLDCDriver3PWM(int A, int B, int C)` | `A,B,C`: A/B/C相PWM引脚 | - | `BLDCDriver3PWM driver(9, 5, 6);` |
| `BLDCDriver3PWM(int A, int B, int C, int EN)` | `A,B,C`: A/B/C相PWM引脚<br>`EN`: 使能引脚 | - | `BLDCDriver3PWM driver(9, 5, 6, 8);` |
| `BLDCDriver3PWM(int A, int B, int C, int EA, int EB, int EC)` | `A,B,C`: A/B/C相PWM引脚<br>`EA,EB,EC`: 各相使能引脚 | - | `BLDCDriver3PWM driver(9, 5, 6, 8, 7, 6);` |

#### 方法
| 函数签名 | 参数说明 | 返回值 | 说明 |
|---------|---------|--------|------|
| `int init()` | 无 | `int`<br>1=成功<br>0=失败 | 初始化驱动器硬件 |
| `void enable()` | 无 | void | 使能驱动器 |
| `void disable()` | 无 | void | 禁用驱动器 |
| `void setPwm(float Ua, float Ub, float Uc)` | `Ua,Ub,Uc`: A/B/C相电压(V) | void | 设置三相电压 |
| `void setPhaseState(PhaseState sa, PhaseState sb, PhaseState sc)` | `sa,sb,sc`: 相状态<br>`_ACTIVE` 或 `_HIGH_Z` | void | 设置相状态（需各相独立使能引脚） |

### BLDCDriver6PWM

#### 构造函数
| 函数签名 | 参数说明 | 返回值 | 示例 |
|---------|---------|--------|------|
| `BLDCDriver6PWM(int AH, int AL, int BH, int BL, int CH, int CL)` | `XH`: X相高侧PWM<br>`XL`: X相低侧PWM | - | `BLDCDriver6PWM driver(9, 10, 5, 6, 3, 11);` |
| `BLDCDriver6PWM(int AH, int AL, int BH, int BL, int CH, int CL, int EN)` | 同上 + `EN`: 使能引脚 | - | `BLDCDriver6PWM driver(9, 10, 5, 6, 3, 11, 8);` |

方法与 BLDCDriver3PWM 相同。

---

## 传感器类

### Encoder (编码器)

#### 构造函数
| 函数签名 | 参数说明 | 返回值 | 示例 |
|---------|---------|--------|------|
| `Encoder(int A, int B, int cpr)` | `A,B`: A/B通道引脚<br>`cpr`: 每转脉冲数 | - | `Encoder sensor(2, 3, 2048);` |
| `Encoder(int A, int B, int cpr, int I)` | 同上 + `I`: 索引引脚 | - | `Encoder sensor(2, 3, 2048, A0);` |

#### 方法
| 函数签名 | 参数说明 | 返回值 | 说明 |
|---------|---------|--------|------|
| `void init()` | 无 | void | 初始化编码器 |
| `void enableInterrupts(void (*doA)(), void (*doB)())` | `doA,doB`: 中断回调函数指针 | void | 启用A/B通道中断 |
| `void enableInterrupts(void (*doA)(), void (*doB)(), void (*doI)())` | 同上 + `doI`: 索引中断回调 | void | 启用A/B/索引中断 |
| `void handleA()` | 无 | void | A通道中断处理函数 |
| `void handleB()` | 无 | void | B通道中断处理函数 |
| `void handleIndex()` | 无 | void | 索引中断处理函数 |
| `float getAngle()` | 无 | `float`: 角度(rad) | 获取当前角度 |
| `float getVelocity()` | 无 | `float`: 速度(rad/s) | 获取当前速度 |
| `void update()` | 无 | void | 更新传感器值（在loop中调用） |

### MagneticSensorSPI (SPI磁传感器)

#### 构造函数
| 函数签名 | 参数说明 | 返回值 | 示例 |
|---------|---------|--------|------|
| `MagneticSensorSPI(int cs, float bits, int reg)` | `cs`: 片选引脚<br>`bits`: 位分辨率<br>`reg`: 角度寄存器地址 | - | `MagneticSensorSPI sensor(10, 14, 0x3FFF);` |

#### 方法
| 函数签名 | 参数说明 | 返回值 | 说明 |
|---------|---------|--------|------|
| `void init()` | 无 | void | 初始化传感器 |
| `float getAngle()` | 无 | `float`: 角度(rad) | 获取角度 |
| `float getVelocity()` | 无 | `float`: 速度(rad/s) | 获取速度 |

### MagneticSensorI2C (I2C磁传感器)

#### 构造函数
| 函数签名 | 参数说明 | 返回值 | 示例 |
|---------|---------|--------|------|
| `MagneticSensorI2C(uint8_t addr)` | `addr`: I2C地址 | - | `MagneticSensorI2C sensor(0x36);` |

方法与 MagneticSensorSPI 相同。

### HallSensor (霍尔传感器)

#### 构造函数
| 函数签名 | 参数说明 | 返回值 | 示例 |
|---------|---------|--------|------|
| `HallSensor(int A, int B, int C, int pp)` | `A,B,C`: 霍尔A/B/C引脚<br>`pp`: 电机极对数 | - | `HallSensor sensor(2, 3, 4, 11);` |

#### 方法
| 函数签名 | 参数说明 | 返回值 | 说明 |
|---------|---------|--------|------|
| `void init()` | 无 | void | 初始化传感器 |
| `void enableInterrupts(void (*doA)(), void (*doB)(), void (*doC)())` | `doA,doB,doC`: 中断回调 | void | 启用中断 |
| `void handleA()` | 无 | void | A通道中断处理 |
| `void handleB()` | 无 | void | B通道中断处理 |
| `void handleC()` | 无 | void | C通道中断处理 |
| `float getAngle()` | 无 | `float`: 角度(rad) | 获取角度 |
| `float getVelocity()` | 无 | `float`: 速度(rad/s) | 获取速度 |

---

## 电流检测类

### InlineCurrentSense (在线电流检测)

#### 构造函数
| 函数签名 | 参数说明 | 返回值 | 示例 |
|---------|---------|--------|------|
| `InlineCurrentSense(float R, float gain, int A, int B)` | `R`: 分流电阻(Ω)<br>`gain`: 放大增益<br>`A,B`: ADC引脚 | - | `InlineCurrentSense cs(0.01, 50, A0, A2);` |
| `InlineCurrentSense(float R, float gain, int A, int B, int C)` | 同上 + `C`: C相ADC引脚 | - | `InlineCurrentSense cs(0.01, 50, A0, A1, A2);` |
| `InlineCurrentSense(float mVpA, int A, int B)` | `mVpA`: mV/A比率<br>`A,B`: ADC引脚 | - | `InlineCurrentSense cs(185.0, A0, A2);` |
| `InlineCurrentSense(float mVpA, int A, int B, int C)` | 同上 + `C`: C相ADC引脚 | - | `InlineCurrentSense cs(185.0, A0, A1, A2);` |

#### 方法
| 函数签名 | 参数说明 | 返回值 | 说明 |
|---------|---------|--------|------|
| `int init()` | 无 | `int`<br>1=成功<br>0=失败 | 初始化电流传感器 |
| `void linkDriver(BLDCDriver* driver)` | `driver`: 驱动器指针 | void | 链接驱动器 |
| `int driverAlign(float voltage)` | `voltage`: 对齐电压(V) | `int`<br>0=失败<br>1=成功无变化<br>2=引脚重配<br>3=增益反转<br>4=引脚和增益都改 | 与驱动器对齐 |
| `PhaseCurrent_s getPhaseCurrents()` | 无 | `PhaseCurrent_s`<br>结构体{a, b, c} | 获取三相电流(A) |
| `float getDCCurrent()` | 无 | `float`: 电流幅值(A) | 获取DC电流幅值 |
| `float getDCCurrent(float angle)` | `angle`: 电气角度(rad) | `float`: 带符号电流(A) | 获取带符号DC电流 |
| `DQCurrent_s getFOCCurrents(float angle)` | `angle`: 电气角度(rad) | `DQCurrent_s`<br>结构体{d, q} | 获取DQ轴电流(A) |

### LowsideCurrentSense (低侧电流检测)

构造函数和方法与 InlineCurrentSense 相同。

---

## 控制器类

### PIDController (PID控制器)

#### 方法
| 函数签名 | 参数说明 | 返回值 | 说明 |
|---------|---------|--------|------|
| `float operator()(float error)` | `error`: 误差值 | `float`: 控制输出 | 计算PID输出 |
| `void reset()` | 无 | void | 重置积分项 |

**属性：**
- `float P` - 比例增益
- `float I` - 积分增益
- `float D` - 微分增益
- `float output_ramp` - 输出斜坡限制(值/秒)
- `float limit` - 输出限制

### PController (P控制器)

#### 方法
| 函数签名 | 参数说明 | 返回值 | 说明 |
|---------|---------|--------|------|
| `float operator()(float error)` | `error`: 误差值 | `float`: 控制输出 | 计算P输出 |

**属性：**
- `float P` - 比例增益
- `float output_ramp` - 输出斜坡限制
- `float limit` - 输出限制

### LowPassFilter (低通滤波器)

#### 方法
| 函数签名 | 参数说明 | 返回值 | 说明 |
|---------|---------|--------|------|
| `float operator()(float x)` | `x`: 输入值 | `float`: 滤波后的值 | 低通滤波 |

**属性：**
- `float Tf` - 滤波时间常数(秒)

---

## 通信类

### Commander (命令接口)

#### 构造函数
| 函数签名 | 参数说明 | 返回值 | 示例 |
|---------|---------|--------|------|
| `Commander(Stream& serial)` | `serial`: 串口对象引用 | - | `Commander cmd(Serial);` |

#### 方法
| 函数签名 | 参数说明 | 返回值 | 说明 |
|---------|---------|--------|------|
| `void add(char id, void (*callback)(char*), const char* label)` | `id`: 命令ID字符<br>`callback`: 回调函数<br>`label`: 标签(可选) | void | 添加自定义命令 |
| `void motor(BLDCMotor* motor, char* cmd)` | `motor`: 电机指针<br>`cmd`: 命令字符串 | void | 电机完整配置命令 |
| `void motion(BLDCMotor* motor, char* cmd)` | `motor`: 电机指针<br>`cmd`: 命令字符串 | void | 运动控制命令 |
| `void scalar(float* value, char* cmd)` | `value`: 变量指针<br>`cmd`: 命令字符串 | void | 标量变量命令 |
| `void pid(PIDController* pid, char* cmd)` | `pid`: PID控制器指针<br>`cmd`: 命令字符串 | void | PID配置命令 |
| `void lpf(LowPassFilter* lpf, char* cmd)` | `lpf`: 滤波器指针<br>`cmd`: 命令字符串 | void | 滤波器配置命令 |
| `void run()` | 无 | void | 运行命令解析器 |

### SimpleFOCDebug (调试工具)

#### 静态方法
| 函数签名 | 参数说明 | 返回值 | 说明 |
|---------|---------|--------|------|
| `static void enable(Stream* serial)` | `serial`: 串口指针 | void | 启用详细调试输出 |
| `static void disable()` | 无 | void | 禁用调试输出 |

---

## 数据结构

### PhaseCurrent_s (相电流结构)
```cpp
struct PhaseCurrent_s {
    float a;  // A相电流(A)
    float b;  // B相电流(A)
    float c;  // C相电流(A)，可能为0
};
```

### DQCurrent_s (DQ电流结构)
```cpp
struct DQCurrent_s {
    float d;  // D轴电流(A) - 磁通电流
    float q;  // Q轴电流(A) - 扭矩电流
};
```

### Direction (方向枚举)
```cpp
enum Direction {
    CW,   // 顺时针
    CCW   // 逆时针
};
```

### PhaseState (相状态枚举)
```cpp
enum PhaseState {
    _ACTIVE,          // 激活
    _HIGH_Z,          // 高阻态
    _HIGH_IMPEDANCE   // 高阻态(同_HIGH_Z)
};
```

---

**文档版本**: v1.0  
**适用库版本**: Arduino-FOC v2.3.x  
**更新日期**: 2025-11-27
