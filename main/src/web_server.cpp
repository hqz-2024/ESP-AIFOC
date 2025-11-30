#include "web_server.h"

// 静态成员初始化
WebServerManager* WebServerManager::instance = nullptr;

// 构造函数
WebServerManager::WebServerManager(MotorControl* motorCtrl)
    : server(WEB_SERVER_PORT),
      motorControl(motorCtrl)
{
    instance = this;
}

// 静态包装函数
void WebServerManager::handleRootStatic() {
    if (instance) {
        instance->handleRoot();
    }
}

void WebServerManager::handleSetSpeedStatic() {
    if (instance) {
        instance->handleSetSpeed();
    }
}

void WebServerManager::handleSetAngleStatic() {
    if (instance) {
        instance->handleSetAngle();
    }
}

void WebServerManager::handleSetTorqueStatic() {
    if (instance) {
        instance->handleSetTorque();
    }
}

void WebServerManager::handleSetModeStatic() {
    if (instance) {
        instance->handleSetMode();
    }
}

void WebServerManager::handleSetControlStatic() {
    if (instance) {
        instance->handleSetControl();
    }
}

// 初始化Web服务器
void WebServerManager::begin() {
    server.on("/", handleRootStatic);
    server.on("/setspeed", handleSetSpeedStatic);
    server.on("/setangle", handleSetAngleStatic);
    server.on("/settorque", handleSetTorqueStatic);
    server.on("/setmode", handleSetModeStatic);
    server.on("/setcontrol", handleSetControlStatic);  // 统一控制接口
    server.begin();
    Serial.println("Web server started!");
}

// 处理客户端请求
void WebServerManager::handleClient() {
    server.handleClient();
}

// 主页处理
void WebServerManager::handleRoot() {
    int mode = motorControl->getControlMode();
    String modeNames[] = {"速度控制", "位置控制", "扭矩控制"};

    String html = "<!DOCTYPE html><html><head>";
    html += "<meta charset='UTF-8'>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<title>ESP32 FOC电机控制</title>";
    html += "<style>";
    html += "body { font-family: Arial, sans-serif; max-width: 700px; margin: 30px auto; padding: 20px; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); }";
    html += ".container { background: white; padding: 30px; border-radius: 15px; box-shadow: 0 10px 30px rgba(0,0,0,0.3); }";
    html += "h1 { color: #333; text-align: center; margin-bottom: 10px; }";
    html += ".mode-badge { text-align: center; margin: 10px 0 20px 0; }";
    html += ".badge { display: inline-block; padding: 8px 20px; background: #667eea; color: white; border-radius: 20px; font-size: 14px; font-weight: bold; }";
    html += ".status { background: #e3f2fd; padding: 15px; border-radius: 8px; margin: 20px 0; }";
    html += ".status p { margin: 8px 0; font-size: 15px; }";
    html += ".mode-selector { margin: 20px 0; padding: 15px; background: #fff3e0; border-radius: 8px; }";
    html += ".mode-buttons { display: flex; gap: 10px; margin-top: 10px; }";
    html += ".mode-btn { flex: 1; padding: 10px; font-size: 14px; border: 2px solid #ff9800; background: white; color: #ff9800; border-radius: 5px; cursor: pointer; }";
    html += ".mode-btn.active { background: #ff9800; color: white; }";
    html += ".control { margin: 20px 0; padding: 15px; background: #f5f5f5; border-radius: 8px; }";
    html += "label { display: block; margin-bottom: 8px; font-weight: bold; color: #555; }";
    html += "input[type='number'], select { width: 100%; padding: 12px; font-size: 16px; border: 2px solid #ddd; border-radius: 5px; box-sizing: border-box; }";
    html += "button { width: 100%; padding: 15px; font-size: 18px; background: #4CAF50; color: white; border: none; border-radius: 5px; cursor: pointer; margin-top: 10px; }";
    html += "button:hover { background: #45a049; }";
    html += ".info { color: #666; font-size: 13px; margin-top: 5px; }";
    html += "</style>";
    html += "</head><body>";
    html += "<div class='container'>";
    html += "<h1>🎛️ ESP32 FOC电机控制</h1>";
    html += "<div class='mode-badge'><span class='badge'>" + modeNames[mode] + "</span></div>";

    // 显示控制权状态
    MotorControl::ControlSource source = motorControl->getControlSource();
    String controlSourceName[] = {"无", "Web端", "串口上位机"};
    String controlColor[] = {"#999", "#4CAF50", "#ff9800"};
    html += "<div style='text-align:center; margin:10px 0;'>";
    html += "<span style='color:" + controlColor[source] + "; font-weight:bold;'>";
    html += "🎮 当前控制权：" + controlSourceName[source];
    html += "</span></div>";

    // 状态显示
    html += "<div class='status'>";
    html += "<p><strong>📊 当前速度:</strong> " + String(motorControl->getVelocity(), 2) + " rad/s</p>";
    html += "<p><strong>📐 当前角度:</strong> " + String(motorControl->getAngle(), 2) + " rad (" + String(motorControl->getAngle() * 57.2958, 1) + "°)</p>";

#if CURRENT_SENSE_TYPE > 0
    html += "<p><strong>⚡ 电流 Q:</strong> " + String(motorControl->getCurrentA(), 2) + " A</p>";
#endif

    if (mode == 0) {
        html += "<p><strong>🎯 目标速度:</strong> " + String(motorControl->getTargetVelocity(), 2) + " rad/s</p>";
    } else if (mode == 1) {
        html += "<p><strong>🎯 目标角度:</strong> " + String(motorControl->getTargetAngle(), 2) + " rad (" + String(motorControl->getTargetAngle() * 57.2958, 1) + "°)</p>";
    } else if (mode == 2) {
        html += "<p><strong>🎯 目标扭矩:</strong> " + String(motorControl->getTargetTorque(), 2);
#if CURRENT_SENSE_TYPE > 0
        html += " A</p>";
#else
        html += " V</p>";
#endif
    }
    html += "</div>";

    // 统一的控制表单
    html += "<div class='control'>";
    html += "<form action='/setcontrol' method='GET'>";

    // 模式选择
    html += "<div style='margin-bottom: 20px;'>";
    html += "<label for='mode'>🎛️ 控制模式:</label>";
    html += "<select id='mode' name='mode' onchange='updateInputField()' style='margin-bottom: 10px;'>";
    html += "<option value='0'" + String(mode == 0 ? " selected" : "") + ">速度控制 (Velocity)</option>";
    html += "<option value='1'" + String(mode == 1 ? " selected" : "") + ">位置控制 (Angle)</option>";
    html += "<option value='2'" + String(mode == 2 ? " selected" : "") + ">扭矩控制 (Torque)</option>";
    html += "</select>";
    html += "</div>";

    // 速度控制输入
    html += "<div id='velocity-input' style='display:" + String(mode == 0 ? "block" : "none") + ";'>";
    html += "<label for='velocity'>🎯 目标转速 (rad/s):</label>";
    html += "<input type='number' id='velocity' name='velocity' step='0.1' value='" + String(motorControl->getTargetVelocity(), 1) + "'>";
    html += "<p class='info'>范围: -" + String(MOTOR_VELOCITY_LIMIT, 0) + " ~ " + String(MOTOR_VELOCITY_LIMIT, 0) + " rad/s</p>";
    html += "</div>";

    // 位置控制输入
    html += "<div id='angle-input' style='display:" + String(mode == 1 ? "block" : "none") + ";'>";
    html += "<label for='angle'>🎯 目标角度 (rad):</label>";
    html += "<input type='number' id='angle' name='angle' step='0.1' value='" + String(motorControl->getTargetAngle(), 1) + "'>";
    html += "<p class='info'>提示: 1 rad ≈ 57.3°, 2π rad = 360°, 6.28 rad = 360°</p>";
    html += "</div>";

    // 扭矩控制输入
    html += "<div id='torque-input' style='display:" + String(mode == 2 ? "block" : "none") + ";'>";
#if CURRENT_SENSE_TYPE > 0
    html += "<label for='torque'>🎯 目标扭矩 (A):</label>";
    html += "<input type='number' id='torque' name='torque' step='0.1' value='" + String(motorControl->getTargetTorque(), 1) + "'>";
    html += "<p class='info'>范围: -" + String(MOTOR_CURRENT_LIMIT, 1) + " ~ " + String(MOTOR_CURRENT_LIMIT, 1) + " A (FOC电流模式)</p>";
#else
    html += "<label for='torque'>🎯 目标扭矩 (V):</label>";
    html += "<input type='number' id='torque' name='torque' step='0.1' value='" + String(motorControl->getTargetTorque(), 1) + "'>";
    html += "<p class='info'>范围: -" + String(MOTOR_VOLTAGE_LIMIT, 1) + " ~ " + String(MOTOR_VOLTAGE_LIMIT, 1) + " V (电压模式)</p>";
#endif
    html += "</div>";

    html += "<button type='submit'>✓ 应用设置</button>";
    html += "</form>";
    html += "</div>";

    html += "</div>";

    // JavaScript
    html += "<script>";
    // 根据模式切换显示对应的输入框
    html += "function updateInputField() {";
    html += "  var mode = document.getElementById('mode').value;";
    html += "  document.getElementById('velocity-input').style.display = (mode == '0') ? 'block' : 'none';";
    html += "  document.getElementById('angle-input').style.display = (mode == '1') ? 'block' : 'none';";
    html += "  document.getElementById('torque-input').style.display = (mode == '2') ? 'block' : 'none';";
    html += "}";
    // 自动刷新
    html += "setTimeout(function(){ location.reload(); }, " + String(WEB_REFRESH_INTERVAL) + ");";
    html += "</script>";
    html += "</body></html>";

    server.send(200, "text/html", html);
}

// 设置速度处理
void WebServerManager::handleSetSpeed() {
    if (server.hasArg("speed")) {
        float velocity = server.arg("speed").toFloat();
        motorControl->setTargetVelocity(velocity);

        String html = "<!DOCTYPE html><html><head>";
        html += "<meta charset='UTF-8'>";
        html += "<meta http-equiv='refresh' content='1;url=/'>";
        html += "<style>";
        html += "body { font-family: Arial, sans-serif; text-align: center; padding: 50px; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); }";
        html += ".message { background: white; padding: 30px; border-radius: 10px; display: inline-block; box-shadow: 0 5px 15px rgba(0,0,0,0.3); }";
        html += "h2 { color: #4CAF50; }";
        html += "</style>";
        html += "</head><body>";
        html += "<div class='message'>";
        html += "<h2>✓ 速度设置成功!</h2>";
        html += "<p>目标速度: <strong>" + String(motorControl->getTargetVelocity(), 2) + " rad/s</strong></p>";
        html += "<p>正在返回...</p>";
        html += "</div>";
        html += "</body></html>";

        server.send(200, "text/html", html);
    } else {
        server.send(400, "text/plain", "Missing speed parameter");
    }
}

// 设置角度处理
void WebServerManager::handleSetAngle() {
    if (server.hasArg("angle")) {
        float angle = server.arg("angle").toFloat();
        motorControl->setTargetAngle(angle);

        String html = "<!DOCTYPE html><html><head>";
        html += "<meta charset='UTF-8'>";
        html += "<meta http-equiv='refresh' content='1;url=/'>";
        html += "<style>";
        html += "body { font-family: Arial, sans-serif; text-align: center; padding: 50px; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); }";
        html += ".message { background: white; padding: 30px; border-radius: 10px; display: inline-block; box-shadow: 0 5px 15px rgba(0,0,0,0.3); }";
        html += "h2 { color: #4CAF50; }";
        html += "</style>";
        html += "</head><body>";
        html += "<div class='message'>";
        html += "<h2>✓ 角度设置成功!</h2>";
        html += "<p>目标角度: <strong>" + String(motorControl->getTargetAngle(), 2) + " rad (" + String(motorControl->getTargetAngle() * 57.2958, 1) + "°)</strong></p>";
        html += "<p>正在返回...</p>";
        html += "</div>";
        html += "</body></html>";

        server.send(200, "text/html", html);
    } else {
        server.send(400, "text/plain", "Missing angle parameter");
    }
}

// 设置扭矩处理
void WebServerManager::handleSetTorque() {
    if (server.hasArg("torque")) {
        float torque = server.arg("torque").toFloat();
        motorControl->setTargetTorque(torque);

        String html = "<!DOCTYPE html><html><head>";
        html += "<meta charset='UTF-8'>";
        html += "<meta http-equiv='refresh' content='1;url=/'>";
        html += "<style>";
        html += "body { font-family: Arial, sans-serif; text-align: center; padding: 50px; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); }";
        html += ".message { background: white; padding: 30px; border-radius: 10px; display: inline-block; box-shadow: 0 5px 15px rgba(0,0,0,0.3); }";
        html += "h2 { color: #4CAF50; }";
        html += "</style>";
        html += "</head><body>";
        html += "<div class='message'>";
        html += "<h2>✓ 扭矩设置成功!</h2>";
        html += "<p>目标扭矩: <strong>" + String(motorControl->getTargetTorque(), 2);
#if CURRENT_SENSE_TYPE > 0
        html += " A</strong></p>";
#else
        html += " V</strong></p>";
#endif
        html += "<p>正在返回...</p>";
        html += "</div>";
        html += "</body></html>";

        server.send(200, "text/html", html);
    } else {
        server.send(400, "text/plain", "Missing torque parameter");
    }
}

// 设置模式处理
void WebServerManager::handleSetMode() {
    if (server.hasArg("mode")) {
        int mode = server.arg("mode").toInt();
        motorControl->setControlMode(mode);

        String modeNames[] = {"速度控制", "位置控制", "扭矩控制"};
        String html = "<!DOCTYPE html><html><head>";
        html += "<meta charset='UTF-8'>";
        html += "<meta http-equiv='refresh' content='1;url=/'>";
        html += "<style>";
        html += "body { font-family: Arial, sans-serif; text-align: center; padding: 50px; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); }";
        html += ".message { background: white; padding: 30px; border-radius: 10px; display: inline-block; box-shadow: 0 5px 15px rgba(0,0,0,0.3); }";
        html += "h2 { color: #ff9800; }";
        html += "</style>";
        html += "</head><body>";
        html += "<div class='message'>";
        html += "<h2>✓ 模式切换成功!</h2>";
        html += "<p>当前模式: <strong>" + modeNames[mode] + "</strong></p>";
        html += "<p>正在返回...</p>";
        html += "</div>";
        html += "</body></html>";

        server.send(200, "text/html", html);
    } else {
        server.send(400, "text/plain", "Missing mode parameter");
    }
}

// 统一控制处理（模式+参数）
void WebServerManager::handleSetControl() {
    if (!server.hasArg("mode")) {
        server.send(400, "text/plain", "Missing mode parameter");
        return;
    }

    // 检查控制权限
    if (!motorControl->checkControlPermission(MotorControl::CONTROL_WEB)) {
        String html = "<!DOCTYPE html><html><head>";
        html += "<meta charset='UTF-8'>";
        html += "<title>控制权限被占用</title></head><body>";
        html += "<h1 style='color:red;'>⚠️ 控制权限被占用</h1>";
        html += "<p>当前控制权被<strong>串口上位机</strong>占用</p>";
        html += "<p>Web端仅可查看数据，无法控制电机</p>";
        html += "<p><a href='/'>返回主页</a></p>";
        html += "</body></html>";
        server.send(403, "text/html", html);
        return;
    }

    int mode = server.arg("mode").toInt();
    String modeNames[] = {"速度控制", "位置控制", "扭矩控制"};
    String resultMsg = "";

    // 切换控制模式
    motorControl->setControlMode(mode);

    // 根据模式设置对应的参数
    switch (mode) {
        case 0:  // 速度控制
            if (server.hasArg("velocity")) {
                float velocity = server.arg("velocity").toFloat();
                motorControl->setTargetVelocity(velocity);
                resultMsg = "目标速度: <strong>" + String(motorControl->getTargetVelocity(), 2) + " rad/s</strong>";
            } else {
                resultMsg = "模式已切换，请设置目标速度";
            }
            break;

        case 1:  // 位置控制
            if (server.hasArg("angle")) {
                float angle = server.arg("angle").toFloat();
                motorControl->setTargetAngle(angle);
                resultMsg = "目标角度: <strong>" + String(motorControl->getTargetAngle(), 2) + " rad (" + String(motorControl->getTargetAngle() * 57.2958, 1) + "°)</strong>";
            } else {
                resultMsg = "模式已切换，请设置目标角度";
            }
            break;

        case 2:  // 扭矩控制
            if (server.hasArg("torque")) {
                float torque = server.arg("torque").toFloat();
                motorControl->setTargetTorque(torque);
                resultMsg = "目标扭矩: <strong>" + String(motorControl->getTargetTorque(), 2);
#if CURRENT_SENSE_TYPE > 0
                resultMsg += " A</strong>";
#else
                resultMsg += " V</strong>";
#endif
            } else {
                resultMsg = "模式已切换，请设置目标扭矩";
            }
            break;

        default:
            server.send(400, "text/plain", "Invalid mode");
            return;
    }

    // 返回成功页面
    String html = "<!DOCTYPE html><html><head>";
    html += "<meta charset='UTF-8'>";
    html += "<meta http-equiv='refresh' content='1;url=/'>";
    html += "<style>";
    html += "body { font-family: Arial, sans-serif; text-align: center; padding: 50px; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); }";
    html += ".message { background: white; padding: 30px; border-radius: 10px; display: inline-block; box-shadow: 0 5px 15px rgba(0,0,0,0.3); }";
    html += "h2 { color: #4CAF50; }";
    html += "</style>";
    html += "</head><body>";
    html += "<div class='message'>";
    html += "<h2>✓ 设置成功!</h2>";
    html += "<p>控制模式: <strong>" + modeNames[mode] + "</strong></p>";
    html += "<p>" + resultMsg + "</p>";
    html += "<p>正在返回...</p>";
    html += "</div>";
    html += "</body></html>";

    server.send(200, "text/html", html);
}

