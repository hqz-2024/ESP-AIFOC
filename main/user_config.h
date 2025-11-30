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
#define ENCODER_PIN_A       8
#define ENCODER_PIN_B       9
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
#define DRIVER_VOLTAGE      11.0f       // 电源电压 (V)
#define DRIVER_PWM_FREQ     20000       // PWM频率 (Hz)

// ==================== 电机配置 ====================
#define MOTOR_POLE_PAIRS    7          // 电机极对数

// ==================== 电流传感器配置 ====================
// 电流传感器类型：0=无电流传感器, 1=在线电流传感器(inline), 2=低侧电流传感器(lowside)
#define CURRENT_SENSE_TYPE  1

// 在线电流传感器配置（CURRENT_SENSE_TYPE=1）
// 注意：SimpleFOC支持2相电流检测（A和B相），C相可以通过计算得出（Ic = -Ia - Ib）
#define CURRENT_SENSE_A     3           // A相电流检测引脚（ADC）
#define CURRENT_SENSE_B     4           // B相电流检测引脚（ADC）
#define CURRENT_SENSE_SHUNT_R   0.01f   // 采样电阻阻值（欧姆）
#define CURRENT_SENSE_AMP_GAIN  20.0f   // 运放增益

// 低侧电流传感器配置（CURRENT_SENSE_TYPE=2）
// 注意：SimpleFOC支持2相电流检测（A和B相），C相可以通过计算得出（Ic = -Ia - Ib）
#define CURRENT_SENSE_LOW_A 1           // A相低侧电流检测引脚（ADC）
#define CURRENT_SENSE_LOW_B 2           // B相低侧电流检测引脚（ADC）

// ==================== 控制模式配置 ====================
// 初始控制模式：0=速度控制(velocity), 1=位置控制(angle), 2=扭矩控制(torque)
// 注意：运行时可通过Web界面动态切换控制模式，此处仅为启动时的初始模式
#define CONTROL_MODE        0

// ==================== 控制参数 ====================
// PID速度控制参数
#define PID_VELOCITY_P      0.9f        // 比例系数（增大以提高响应）
#define PID_VELOCITY_I      5.4f       // 积分系数（恢复积分作用）
#define PID_VELOCITY_D      0.0f
#define LPF_VELOCITY_TF     0.1f       // 速度低通滤波时间常数（减小以提高响应）

// PID位置控制参数
#define PID_ANGLE_P         7.0f       // 增大位置环P
#define PID_ANGLE_I         0.0f        // 位置环通常不需要积分
#define PID_ANGLE_D         0.0f
#define LPF_ANGLE_TF        0.1f       // 位置低通滤波时间常数

// PID电流控制参数（FOC电流模式）
#define PID_CURRENT_Q_P     1.0f        // Q轴电流P（增大）
#define PID_CURRENT_Q_I     3.0f      // Q轴电流I（恢复强积分）
#define PID_CURRENT_Q_D     0.0f
#define PID_CURRENT_D_P     1.0f        // D轴电流P（增大）
#define PID_CURRENT_D_I     3.0f      // D轴电流I（恢复强积分）
#define PID_CURRENT_D_D     0.0f
#define LPF_CURRENT_TF      0.5f      // 电流低通滤波时间常数

// 限制参数
#define MOTOR_VOLTAGE_LIMIT     11.0f   // 电压限制 (V)
#define MOTOR_VELOCITY_LIMIT    50.0f   // 速度限制 (rad/s)
#define MOTOR_CURRENT_LIMIT     3.0f    // 电流限制 (A) - 增大到3A

// ==================== Web界面配置 ====================
#define WEB_REFRESH_INTERVAL    10000   // 网页自动刷新间隔 (ms)
#define STATUS_PRINT_INTERVAL   1000    // 串口状态输出间隔 (ms)

#endif // USER_CONFIG_H

