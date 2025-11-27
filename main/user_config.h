#ifndef USER_CONFIG_H
#define USER_CONFIG_H

// ==================== WiFi配置 ====================
#define WIFI_SSID           "联域科技"
#define WIFI_PASSWORD       "lykj987654321"
#define WEB_SERVER_PORT     80

// ==================== 串口配置 ====================
#define SERIAL_BAUDRATE     115200

// ==================== 传感器配置 ====================
// 传感器类型选择：0=编码器(Encoder), 1=AS5600磁传感器(I2C)
#define SENSOR_TYPE         1

// 编码器配置（仅当SENSOR_TYPE=0时使用）
#define ENCODER_PIN_A       2
#define ENCODER_PIN_B       3
#define ENCODER_PPR         2048        // 编码器每转脉冲数

// AS5600配置（仅当SENSOR_TYPE=1时使用）
#define AS5600_I2C_ADDRESS  0x36        // AS5600默认I2C地址
#define I2C_SDA_PIN         8          // ESP32 I2C SDA引脚
#define I2C_SCL_PIN         9          // ESP32 I2C SCL引脚

// ==================== 驱动器配置 ====================
#define DRIVER_PWM_A        5
#define DRIVER_PWM_B        6
#define DRIVER_PWM_C        7
#define DRIVER_ENABLE       10
#define DRIVER_VOLTAGE      12.0f       // 电源电压 (V)
#define DRIVER_PWM_FREQ     20000       // PWM频率 (Hz)

// ==================== 电机配置 ====================
#define MOTOR_POLE_PAIRS    11          // 电机极对数

// ==================== 控制参数 ====================
// PID速度控制参数
#define PID_VELOCITY_P      0.2f
#define PID_VELOCITY_I      20.0f
#define PID_VELOCITY_D      0.0f
#define LPF_VELOCITY_TF     0.01f       // 速度低通滤波时间常数

// 限制参数
#define MOTOR_VOLTAGE_LIMIT     6.0f    // 电压限制 (V)
#define MOTOR_VELOCITY_LIMIT    20.0f   // 速度限制 (rad/s)

// ==================== Web界面配置 ====================
#define WEB_REFRESH_INTERVAL    10000   // 网页自动刷新间隔 (ms)
#define STATUS_PRINT_INTERVAL   1000    // 串口状态输出间隔 (ms)

#endif // USER_CONFIG_H

