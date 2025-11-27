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

// 初始化Web服务器
void WebServerManager::begin() {
    server.on("/", handleRootStatic);
    server.on("/setspeed", handleSetSpeedStatic);
    server.begin();
    Serial.println("Web server started!");
}

// 处理客户端请求
void WebServerManager::handleClient() {
    server.handleClient();
}

// 主页处理
void WebServerManager::handleRoot() {
    String html = "<!DOCTYPE html><html><head>";
    html += "<meta charset='UTF-8'>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<title>ESP32 电机控制</title>";
    html += "<style>";
    html += "body { font-family: Arial, sans-serif; max-width: 600px; margin: 50px auto; padding: 20px; background: #f0f0f0; }";
    html += ".container { background: white; padding: 30px; border-radius: 10px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }";
    html += "h1 { color: #333; text-align: center; }";
    html += ".status { background: #e3f2fd; padding: 15px; border-radius: 5px; margin: 20px 0; }";
    html += ".status p { margin: 8px 0; font-size: 16px; }";
    html += ".control { margin: 20px 0; }";
    html += "label { display: block; margin-bottom: 8px; font-weight: bold; color: #555; }";
    html += "input[type='number'] { width: 100%; padding: 12px; font-size: 16px; border: 2px solid #ddd; border-radius: 5px; box-sizing: border-box; }";
    html += "button { width: 100%; padding: 15px; font-size: 18px; background: #4CAF50; color: white; border: none; border-radius: 5px; cursor: pointer; margin-top: 10px; }";
    html += "button:hover { background: #45a049; }";
    html += "button:active { background: #3d8b40; }";
    html += ".info { color: #666; font-size: 14px; margin-top: 10px; }";
    html += "</style>";
    html += "</head><body>";
    html += "<div class='container'>";
    html += "<h1>🔧 ESP32 电机控制面板</h1>";
    html += "<div class='status'>";
    html += "<p><strong>当前速度:</strong> " + String(motorControl->getVelocity(), 2) + " rad/s</p>";
    html += "<p><strong>当前角度:</strong> " + String(motorControl->getAngle(), 2) + " rad</p>";
    html += "<p><strong>目标速度:</strong> " + String(motorControl->getTargetVelocity(), 2) + " rad/s</p>";
    html += "</div>";
    html += "<div class='control'>";
    html += "<form action='/setspeed' method='GET'>";
    html += "<label for='speed'>设置目标转速 (rad/s):</label>";
    html += "<input type='number' id='speed' name='speed' step='0.1' value='" + String(motorControl->getTargetVelocity(), 1) + "' required>";
    html += "<p class='info'>范围: -" + String(MOTOR_VELOCITY_LIMIT, 0) + " ~ " + String(MOTOR_VELOCITY_LIMIT, 0) + " rad/s</p>";
    html += "<button type='submit'>✓ 确定更新</button>";
    html += "</form>";
    html += "</div>";
    html += "</div>";
    html += "<script>";
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
        
        // 返回成功页面
        String html = "<!DOCTYPE html><html><head>";
        html += "<meta charset='UTF-8'>";
        html += "<meta http-equiv='refresh' content='1;url=/'>";
        html += "<style>";
        html += "body { font-family: Arial, sans-serif; text-align: center; padding: 50px; background: #f0f0f0; }";
        html += ".message { background: white; padding: 30px; border-radius: 10px; display: inline-block; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }";
        html += "h2 { color: #4CAF50; }";
        html += "</style>";
        html += "</head><body>";
        html += "<div class='message'>";
        html += "<h2>✓ 设置成功!</h2>";
        html += "<p>目标速度已更新为: <strong>" + String(motorControl->getTargetVelocity(), 2) + " rad/s</strong></p>";
        html += "<p>正在返回主页...</p>";
        html += "</div>";
        html += "</body></html>";
        
        server.send(200, "text/html", html);
    } else {
        server.send(400, "text/plain", "Missing speed parameter");
    }
}

