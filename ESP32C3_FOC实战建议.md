# ESP32-C3 FOC 实战建议和优化方案

## 🎯 你的项目现状

根据你的 `platformio.ini` 配置：
- **MCU**: ESP32-C3 (160MHz, 无FPU)
- **Flash**: 4MB
- **框架**: Arduino
- **库**: SimpleFOC (最新版)

---

## ⚠️ ESP32-C3 的核心限制

### 1. 硬件限制
```
❌ 无FPU - FOC算法中的三角函数计算慢5-10倍
❌ 无MCPWM - 只能用LEDC模块，不支持6PWM
❌ ADC无法与定时器同步 - 低侧电流检测困难
⚠️ 只有6个PWM通道 - 最多控制2个电机（3PWM模式）
⚠️ ADC通道少 - 电流检测通道有限
```

### 2. 性能限制
```
loopFOC() 执行时间：
- 电压模式: ~150-200 μs
- DC电流模式: ~250-300 μs  
- FOC电流模式: ~350-400 μs

最大控制频率：
- 电压模式: ~3-5 kHz
- FOC电流模式: ~1.5-2 kHz
```

---

## ✅ 推荐的应用方案

### 方案1: 电压模式 + 编码器（最推荐）

**适用场景：**
- 云台电机控制
- 机械臂关节控制
- 位置/速度控制
- 不需要精确扭矩控制

**硬件配置：**
```
电机: BLDC电机（11极对，<100W）
驱动器: 3PWM驱动（如L6234、DRV8313）
传感器: 增量编码器（2048 PPR）或磁传感器（AS5600）
电流检测: 无（使用电压模式）
```

**代码示例：**
```cpp
#include <SimpleFOC.h>

// 编码器
Encoder encoder = Encoder(2, 3, 2048);
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}

// 驱动器 (PWM引脚: GPIO 4, 5, 6, 使能: GPIO 7)
BLDCDriver3PWM driver = BLDCDriver3PWM(4, 5, 6, 7);

// 电机 (11极对)
BLDCMotor motor = BLDCMotor(11);

void setup() {
    Serial.begin(115200);
    
    // 初始化编码器
    encoder.init();
    encoder.enableInterrupts(doA, doB);
    motor.linkSensor(&encoder);
    
    // 初始化驱动器
    driver.voltage_power_supply = 12;
    driver.pwm_frequency = 20000;  // 20kHz PWM
    driver.init();
    motor.linkDriver(&driver);
    
    // 配置电机 - 电压模式速度控制
    motor.controller = MotionControlType::velocity;
    motor.torque_controller = TorqueControlType::voltage;
    
    // PID参数（需要根据实际电机调整）
    motor.PID_velocity.P = 0.2;
    motor.PID_velocity.I = 20;
    motor.PID_velocity.D = 0;
    motor.LPF_velocity.Tf = 0.01;
    
    // 限制参数
    motor.voltage_limit = 6;      // 限制电压，避免过流
    motor.velocity_limit = 20;    // 速度限制 (rad/s)
    
    // 初始化
    motor.init();
    motor.initFOC();
    
    Serial.println("Motor ready!");
}

void loop() {
    motor.loopFOC();        // FOC算法
    motor.move(10);         // 目标速度: 10 rad/s (约95 RPM)
    
    // 每秒输出一次状态
    static unsigned long last_print = 0;
    if(millis() - last_print > 1000) {
        Serial.print("Speed: ");
        Serial.print(motor.shaft_velocity);
        Serial.print(" rad/s, Angle: ");
        Serial.println(motor.shaft_angle);
        last_print = millis();
    }
}
```

**优点：**
- ✅ 不需要电流传感器，成本低
- ✅ 性能足够（loopFOC ~150μs）
- ✅ 代码简单，易于调试
- ✅ 适合大多数应用

**缺点：**
- ❌ 无法精确控制扭矩
- ❌ 负载变化时性能下降

---

### 方案2: 电压模式 + WiFi控制

**适用场景：**
- 远程控制电机
- IoT应用
- 手机APP控制

**额外代码：**
```cpp
#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "your_wifi_ssid";
const char* password = "your_wifi_password";

WebServer server(80);
float target_velocity = 0;

void handleSetSpeed() {
    if(server.hasArg("speed")) {
        target_velocity = server.arg("speed").toFloat();
        server.send(200, "text/plain", "Speed set to " + String(target_velocity));
    } else {
        server.send(400, "text/plain", "Missing speed parameter");
    }
}

void setup() {
    // ... 电机初始化代码 ...
    
    // WiFi连接
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    
    // Web服务器
    server.on("/setspeed", handleSetSpeed);
    server.begin();
}

void loop() {
    motor.loopFOC();
    motor.move(target_velocity);
    server.handleClient();  // 处理Web请求
}
```

**使用方法：**
```
浏览器访问: http://192.168.1.100/setspeed?speed=10
设置目标速度为10 rad/s
```

---

### 方案3: 在线电流检测 + FOC电流模式（高级）

**适用场景：**
- 需要精确扭矩控制
- 高性能应用
- 预算充足

**硬件配置：**
```
电流传感器: 
- 霍尔电流传感器（ACS712, 185mV/A）
- 或在线分流电阻 + 运放（INA240, gain=20）
```

**代码示例：**
```cpp
// 在线电流检测 (A相: GPIO 0, B相: GPIO 1)
// 使用ACS712-5A (185 mV/A)
InlineCurrentSense current_sense = InlineCurrentSense(185.0, A0, A1);

void setup() {
    // ... 传感器和驱动器初始化 ...
    
    // 初始化电流传感器
    current_sense.init();
    current_sense.linkDriver(&driver);
    motor.linkCurrentSense(&current_sense);
    
    // FOC电流模式
    motor.controller = MotionControlType::torque;
    motor.torque_controller = TorqueControlType::foc_current;
    
    // 电流PID参数
    motor.PID_current_q.P = 5;
    motor.PID_current_q.I = 300;
    motor.PID_current_d.P = 5;
    motor.PID_current_d.I = 300;
    motor.LPF_current_q.Tf = 0.005;
    motor.LPF_current_d.Tf = 0.005;
    
    motor.current_limit = 2;  // 2A限制
    
    motor.init();
    motor.initFOC();
}

void loop() {
    motor.loopFOC();
    motor.move(1.0);  // 目标电流: 1A
}
```

**注意事项：**
- ⚠️ loopFOC()执行时间 ~350μs，控制频率降低
- ⚠️ 需要调整motion_downsample降低运动控制频率
- ⚠️ 成本增加（电流传感器 ¥20-40）

---

## 🚀 性能优化技巧

### 1. 降低运动控制频率

```cpp
// FOC算法需要高频，但运动控制可以低频
motor.motion_downsample = 5;  // 运动控制降采样5倍

// 这样loopFOC()每次都执行，但move()每5次执行一次
```

### 2. 优化编译选项

你的platformio.ini已经配置了优化：
```ini
build_flags =
    -Os                     ; 优化代码大小（也能提升速度）
    -ffunction-sections
    -fdata-sections
    -Wl,--gc-sections       ; 移除未使用代码
```

**进一步优化：**
```ini
build_flags =
    -O3                     ; 最高速度优化（会增加代码大小）
    -ffast-math             ; 快速数学运算（牺牲精度）
```

### 3. 使用查找表代替三角函数

SimpleFOC已经内置了查找表优化，但你可以进一步优化：
```cpp
// 在setup()中
motor.foc_modulation = FOCModulationType::SinePWM;  // 使用查找表
// 而不是实时计算
```

### 4. 减少串口输出

```cpp
// ❌ 不要在loop()中频繁打印
void loop() {
    motor.loopFOC();
    Serial.println(motor.shaft_velocity);  // 这会严重降低性能！
}

// ✅ 使用定时输出
void loop() {
    motor.loopFOC();
    
    static unsigned long last_print = 0;
    if(millis() - last_print > 100) {  // 每100ms输出一次
        Serial.println(motor.shaft_velocity);
        last_print = millis();
    }
}
```

### 5. 使用核心绑定

```cpp
// ESP32-C3是单核，但可以设置任务优先级
void setup() {
    // 设置loop()任务优先级
    vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);
}
```

---

## 📊 引脚分配建议

### ESP32-C3 可用引脚

```
PWM输出 (LEDC):
- GPIO 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10

ADC输入:
- GPIO 0 (ADC1_CH0)
- GPIO 1 (ADC1_CH1)
- GPIO 2 (ADC1_CH2)
- GPIO 3 (ADC1_CH3)
- GPIO 4 (ADC1_CH4)

编码器输入 (支持中断):
- 任意GPIO

注意：
- GPIO 18, 19 用于USB，不要占用
- GPIO 12, 13 用于JTAG调试
```

### 推荐引脚分配

```cpp
// 3PWM驱动器
#define PWM_A  4
#define PWM_B  5
#define PWM_C  6
#define ENABLE 7

// 编码器
#define ENC_A  8
#define ENC_B  9

// 电流检测（如果使用）
#define CURR_A A0  // GPIO 0
#define CURR_B A1  // GPIO 1

// 磁传感器SPI（如果使用）
#define SPI_CS   10
#define SPI_MOSI 7
#define SPI_MISO 2
#define SPI_SCK  6
```

---

## 🐛 常见问题和解决方案

### 问题1: initFOC() 失败

**原因：**
- 传感器未正确连接
- 电机无法自由转动
- 电源电压不足

**解决：**
```cpp
// 启用调试
SimpleFOCDebug::enable(&Serial);

// 降低对齐电压
motor.voltage_sensor_align = 2;  // 默认3V，降低到2V

// 增加对齐时间
motor.velocity_index_search = 0.5;  // 降低搜索速度
```

### 问题2: 电机抖动

**原因：**
- PID参数不合适
- 电源纹波大
- loopFOC()频率不够

**解决：**
```cpp
// 降低P增益
motor.PID_velocity.P = 0.1;  // 从0.2降到0.1

// 增加滤波
motor.LPF_velocity.Tf = 0.02;  // 从0.01增加到0.02

// 降低电压限制
motor.voltage_limit = 4;  // 从6V降到4V
```

### 问题3: 速度不稳定

**原因：**
- 负载变化
- 电压模式无法补偿负载

**解决：**
```cpp
// 增加I增益
motor.PID_velocity.I = 50;  // 从20增加到50

// 或使用电流模式（如果有电流传感器）
motor.torque_controller = TorqueControlType::dc_current;
```

---

## 📈 性能测试建议

### 测试loopFOC()执行时间

```cpp
void loop() {
    unsigned long start = micros();
    motor.loopFOC();
    unsigned long duration = micros() - start;
    
    static unsigned long max_duration = 0;
    if(duration > max_duration) {
        max_duration = duration;
        Serial.print("Max loopFOC time: ");
        Serial.print(max_duration);
        Serial.println(" us");
    }
    
    motor.move();
}
```

**期望结果：**
- 电压模式: <200 μs
- FOC电流模式: <400 μs

---

## 🎯 总结建议

### 对于你的ESP32-C3项目：

✅ **可以做的：**
1. 电压模式FOC控制（推荐）
2. 速度/位置闭环控制
3. WiFi/BLE远程控制
4. 小功率电机（<100W）
5. 低速应用（<2000 RPM）

⚠️ **有挑战的：**
1. FOC电流模式（性能受限）
2. 高速电机（>3000 RPM）
3. 多电机控制（最多2个）

❌ **不建议做的：**
1. 低侧电流检测
2. 6PWM驱动
3. 高性能伺服控制
4. 工业级应用

### 如果性能不够，考虑：
1. 升级到ESP32-S3 (有FPU，性能提升3倍)
2. 或迁移到STM32G431 (专业电机控制MCU)

---

**祝你的ESP32-C3 FOC项目成功！** 🚀
