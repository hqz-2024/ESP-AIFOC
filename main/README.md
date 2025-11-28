# ESP32 FOC 电机控制系统

## 📁 项目结构

```
main/
├── main.ino                    # 主程序入口
├── user_config.h               # 用户配置文件（所有参数配置）
├── README.md                   # 项目说明文档
└── src/                        # 源代码模块
    ├── motor_control.h         # 电机控制头文件
    ├── motor_control.cpp       # 电机控制实现
    ├── wifi_manager.h          # WiFi管理头文件
    ├── wifi_manager.cpp        # WiFi管理实现
    ├── web_server.h            # Web服务器头文件
    └── web_server.cpp          # Web服务器实现
```

---

## 🔧 模块说明

### 1. user_config.h - 配置文件
**所有系统参数都在这里配置**

包含的配置项：
- WiFi配置（SSID、密码）
- 串口配置（波特率）
- 编码器配置（引脚、PPR）
- 驱动器配置（PWM引脚、电压、频率）
- 电机配置（极对数）
- PID参数（P、I、D、滤波）
- 限制参数（电压、速度）
- Web界面配置（刷新间隔）

**修改参数只需编辑这个文件！**

---

### 2. motor_control - 电机控制模块

**功能：**
- 编码器初始化和中断处理
- 驱动器初始化
- 电机FOC算法
- PID速度控制
- 目标速度设置和限制

**主要接口：**
```cpp
bool init()                          // 初始化电机系统
void loopFOC()                       // FOC循环（高频调用）
void move(float target)              // 运动控制
float getVelocity()                  // 获取当前速度
float getAngle()                     // 获取当前角度
void setTargetVelocity(float vel)    // 设置目标速度
```

---

### 3. wifi_manager - WiFi管理模块

**功能：**
- WiFi连接管理
- 连接状态检查
- IP地址获取

**主要接口：**
```cpp
bool connect()           // 连接WiFi
IPAddress getIP()        // 获取IP地址
bool isConnected()       // 检查连接状态
```

---

### 4. web_server - Web服务器模块

**功能：**
- Web控制界面
- 实时显示电机状态
- 目标速度设置
- 自动刷新

**主要接口：**
```cpp
void begin()             // 启动Web服务器
void handleClient()      // 处理客户端请求
```

**Web路由：**
- `/` - 主页（控制面板）
- `/setspeed?speed=X` - 设置速度

---

## ⚙️ 快速配置指南

### 修改WiFi设置
编辑 `user_config.h`：
```cpp
#define WIFI_SSID           "你的WiFi名称"
#define WIFI_PASSWORD       "你的WiFi密码"
```

### 修改引脚配置
编辑 `user_config.h`：
```cpp
// 编码器引脚
#define ENCODER_PIN_A       2
#define ENCODER_PIN_B       3

// 驱动器PWM引脚
#define DRIVER_PWM_A        4
#define DRIVER_PWM_B        5
#define DRIVER_PWM_C        6
#define DRIVER_ENABLE       7
```

### 修改电机参数
编辑 `user_config.h`：
```cpp
#define MOTOR_POLE_PAIRS    7      // 你的电机极对数
#define DRIVER_VOLTAGE      12.0f   // 电源电压
```

### 调整PID参数
编辑 `user_config.h`：
```cpp
#define PID_VELOCITY_P      0.2f
#define PID_VELOCITY_I      20.0f
#define PID_VELOCITY_D      0.0f
```

### 修改速度限制
编辑 `user_config.h`：
```cpp
#define MOTOR_VELOCITY_LIMIT    20.0f   // 最大速度 (rad/s)
```

---

## 🚀 使用方法

### 1. 编译上传
1. 打开 `main.ino`
2. 修改 `user_config.h` 中的配置
3. 编译并上传到ESP32

### 2. 查看串口
打开串口监视器（115200波特率），查看：
- 系统初始化信息
- WiFi连接状态
- IP地址
- 实时电机状态

### 3. Web控制
浏览器访问ESP32的IP地址，例如：
```
http://192.168.1.100
```

在网页上：
- 查看当前速度、角度
- 输入目标速度
- 点击"确定更新"

---

## 📊 代码架构优势

### ✅ 模块化设计
- 每个功能独立封装
- 易于维护和扩展
- 代码清晰易读

### ✅ 配置集中管理
- 所有参数在 `user_config.h`
- 修改配置不需要改代码
- 避免魔法数字

### ✅ 面向对象
- 使用C++类封装
- 接口清晰
- 便于复用

### ✅ 易于扩展
想添加新功能？
1. 在 `src/` 创建新模块
2. 在 `user_config.h` 添加配置
3. 在 `main.ino` 中调用

---

## 🔍 常见问题

### Q: 如何修改网页刷新间隔？
A: 编辑 `user_config.h`：
```cpp
#define WEB_REFRESH_INTERVAL    10000   // 单位：毫秒
```

### Q: 如何修改串口输出间隔？
A: 编辑 `user_config.h`：
```cpp
#define STATUS_PRINT_INTERVAL   1000    // 单位：毫秒
```

### Q: 如何添加新的Web页面？
A: 在 `web_server.cpp` 中：
1. 添加处理函数
2. 在 `begin()` 中注册路由

### Q: 如何更换传感器类型？
A: 修改 `motor_control.h` 和 `motor_control.cpp`，将 `Encoder` 替换为其他传感器类型（如 `MagneticSensorSPI`）

---

## 📝 开发建议

### 添加新功能的步骤：
1. 在 `user_config.h` 添加相关配置宏
2. 在 `src/` 创建新的 `.h` 和 `.cpp` 文件
3. 在 `main.ino` 中包含头文件并调用
4. 保持模块独立性

### 代码风格：
- 使用有意义的变量名
- 添加必要的注释
- 保持函数简短
- 遵循现有代码风格

---

**祝你使用愉快！** 🎉

