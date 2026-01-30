#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <ArduinoJson.h>

// ========== PIN MAPPING FROM YOUR DIAGRAM ==========
#define MOTOR1_PWM 4
#define MOTOR1_IN1 5
#define MOTOR1_IN2 6
#define MOTOR2_PWM 7
#define MOTOR2_IN1 15
#define MOTOR2_IN2 16
#define MOTOR3_PWM 17
#define MOTOR3_IN1 18
#define MOTOR3_IN2 8
#define MOTOR4_PWM 9
#define MOTOR4_IN1 10
#define MOTOR4_IN2 11
#define STBY_PIN 12
#define SDA_PIN 1
#define SCL_PIN 2
#define LED_PIN 38

// WiFi
const char* ssid = "DRONE";
const char* password = "12345678";

// PWM Channels
#define PWM_CH_M1 0
#define PWM_CH_M2 1
#define PWM_CH_M3 2
#define PWM_CH_M4 3
#define PWM_FREQ 1000
#define PWM_RES 8

// Globals
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
float pitch = 0, roll = 0, yaw = 0;
uint16_t motorVals[4] = {0, 0, 0, 0};
bool armed = false;
unsigned long lastTelemetry = 0;
float throttle = 0, yaw_input = 0, pitch_input = 0, roll_input = 0;
float Kp = 2.0, Ki = 0.0, Kd = 0.5;
float pitch_err, roll_err, pitch_integral=0, roll_integral=0, last_pitch_err=0, last_roll_err=0;

// Forward declaration (or define functions before use)
void readMPU();
void calculateAngles();
void updateFlightController();
void setMotor(int num, int speed);
void stopMotors();
void sendTelemetry();
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, 
               AwsEventType type, void *arg, uint8_t *data, size_t len);
void processCommand(char* json);

// HTML Content - DEFINED BEFORE SETUP
const char HTML_CONTENT[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>Drone Control</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; touch-action: none; user-select: none; }
        body { background: #000; color: #fff; font-family: -apple-system, sans-serif; overflow: hidden; height: 100vh; }
        .horizon { position: absolute; top: 0; left: 0; width: 100%; height: 100%; background: linear-gradient(to bottom, #1e3c72 50%, #2a5298 50%); transform-origin: center; }
        .crosshair { position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%); width: 40px; height: 40px; border: 2px solid rgba(0,255,0,0.8); border-radius: 50%; pointer-events: none; }
        .crosshair::after { content: ''; position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%); width: 4px; height: 4px; background: #0f0; border-radius: 50%; }
        .status { position: absolute; top: 10px; left: 10px; right: 10px; display: flex; justify-content: space-between; z-index: 10; background: rgba(0,0,0,0.7); padding: 10px; border-radius: 10px; }
        .arm-btn { padding: 10px 20px; background: #300; border: 2px solid #f00; color: #f00; border-radius: 5px; font-weight: bold; cursor: pointer; }
        .arm-btn.armed { background: #030; border-color: #0f0; color: #0f0; }
        .telemetry { font-family: monospace; font-size: 12px; }
        .sticks { position: absolute; bottom: 20px; left: 0; right: 0; height: 200px; display: flex; justify-content: space-between; padding: 0 20px; }
        .stick { width: 140px; height: 140px; border: 2px solid rgba(255,255,255,0.3); border-radius: 50%; position: relative; touch-action: none; background: rgba(255,255,255,0.05); }
        .knob { position: absolute; top: 50%; left: 50%; width: 50px; height: 50px; background: radial-gradient(circle, #fff, #00d4ff); border-radius: 50%; transform: translate(-50%, -50%); pointer-events: none; }
        .motor-bars { position: absolute; bottom: 160px; left: 50%; transform: translateX(-50%); display: flex; gap: 5px; }
        .m-bar { width: 20px; height: 60px; background: #333; border-radius: 3px; position: relative; overflow: hidden; }
        .m-fill { position: absolute; bottom: 0; left: 0; right: 0; background: linear-gradient(to top, #0f0, #ff0, #f00); transition: height 0.1s; height: 0%; }
    </style>
</head>
<body>
    <div class="horizon" id="horizon"></div>
    <div class="crosshair"></div>
    <div class="status">
        <button class="arm-btn" id="armBtn" onclick="toggleArm()">DISARMED</button>
        <div class="telemetry">
            <div>Pitch: <span id="p">0</span>° Roll: <span id="r">0</span>°</div>
            <div>Motor%: <span id="mot">0</span></div>
        </div>
    </div>
    <div class="motor-bars">
        <div class="m-bar"><div class="m-fill" id="m1"></div></div>
        <div class="m-bar"><div class="m-fill" id="m2"></div></div>
        <div class="m-bar"><div class="m-fill" id="m3"></div></div>
        <div class="m-bar"><div class="m-fill" id="m4"></div></div>
    </div>
    <div class="sticks">
        <div class="stick" id="left"><div class="knob"></div></div>
        <div class="stick" id="right"><div class="knob"></div></div>
    </div>
    <script>
        let ws, armed = false;
        const knobs = {left: document.querySelector('#left .knob'), right: document.querySelector('#right .knob')};
        const sticks = {left: {x:0, y:0}, right: {x:0, y:0}};
        
        function connect() {
            ws = new WebSocket('ws://' + location.host + '/ws');
            ws.onopen = () => console.log('Connected');
            ws.onmessage = (e) => {
                const d = JSON.parse(e.data);
                document.getElementById('p').textContent = d.pitch.toFixed(1);
                document.getElementById('r').textContent = d.roll.toFixed(1);
                document.getElementById('horizon').style.transform = `rotate(${-d.roll}deg) translateY(${d.pitch * 3}px)`;
                document.getElementById('m1').style.height = (d.m1/2.55) + '%';
                document.getElementById('m2').style.height = (d.m2/2.55) + '%';
                document.getElementById('m3').style.height = (d.m3/2.55) + '%';
                document.getElementById('m4').style.height = (d.m4/2.55) + '%';
                document.getElementById('mot').textContent = Math.round((d.m1+d.m2+d.m3+d.m4)/10.2) + '%';
            };
            ws.onclose = () => setTimeout(connect, 1000);
        }
        function toggleArm() {
            armed = !armed;
            document.getElementById('armBtn').textContent = armed ? 'ARMED' : 'DISARMED';
            document.getElementById('armBtn').classList.toggle('armed', armed);
            ws.send(JSON.stringify({cmd:'arm', state:armed}));
        }
        function setupStick(id, name) {
            const el = document.getElementById(id);
            let active = false, rect, cx, cy;
            const update = (e) => {
                const clientX = e.touches ? e.touches[0].clientX : e.clientX;
                const clientY = e.touches ? e.touches[0].clientY : e.clientY;
                let dx = (clientX - cx) / 45;
                let dy = (clientY - cy) / 45;
                const dist = Math.sqrt(dx*dx + dy*dy);
                if (dist > 1) { dx /= dist; dy /= dist; }
                sticks[name].x = dx;
                sticks[name].y = dy;
                knobs[name].style.transform = `translate(calc(-50% + ${dx*45}px), calc(-50% + ${dy*45}px))`;
                send();
            };
            const start = (e) => {
                active = true;
                e.preventDefault();
                rect = el.getBoundingClientRect();
                cx = rect.left + rect.width/2;
                cy = rect.top + rect.height/2;
                update(e);
            };
            const end = () => {
                active = false;
                sticks[name].x = sticks[name].y = 0;
                knobs[name].style.transform = `translate(-50%, -50%)`;
                send();
            };
            el.addEventListener('touchstart', start);
            el.addEventListener('mousedown', start);
            window.addEventListener('touchmove', (e) => { if(active) { e.preventDefault(); update(e); }});
            window.addEventListener('mousemove', (e) => active && update(e));
            window.addEventListener('touchend', end);
            window.addEventListener('mouseup', end);
        }
        function send() {
            if (!ws || ws.readyState !== 1) return;
            ws.send(JSON.stringify({cmd:'stick', l_x: sticks.left.x, l_y: sticks.left.y, r_x: sticks.right.x, r_y: sticks.right.y}));
        }
        setupStick('left', 'left');
        setupStick('right', 'right');
        connect();
    </script>
</body>
</html>
)rawliteral";

void setup() {
  Serial.begin(115200);
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP: ");
  Serial.println(IP);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B); 
  Wire.write(0);
  Wire.endTransmission(true);

  ledcSetup(PWM_CH_M1, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_M2, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_M3, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_M4, PWM_FREQ, PWM_RES);
  
  ledcAttachPin(MOTOR1_PWM, PWM_CH_M1);
  ledcAttachPin(MOTOR2_PWM, PWM_CH_M2);
  ledcAttachPin(MOTOR3_PWM, PWM_CH_M3);
  ledcAttachPin(MOTOR4_PWM, PWM_CH_M4);

  pinMode(MOTOR1_IN1, OUTPUT); pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR2_IN1, OUTPUT); pinMode(MOTOR2_IN2, OUTPUT);
  pinMode(MOTOR3_IN1, OUTPUT); pinMode(MOTOR3_IN2, OUTPUT);
  pinMode(MOTOR4_IN1, OUTPUT); pinMode(MOTOR4_IN2, OUTPUT);
  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, HIGH);
  pinMode(LED_PIN, OUTPUT);
  
  stopMotors();

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", HTML_CONTENT);
  });
  server.begin();
}

void loop() {
  ws.cleanupClients();
  
  static unsigned long lastMPU = 0;
  if (millis() - lastMPU > 4) {
    lastMPU = millis();
    readMPU();
    calculateAngles();
    if (armed) updateFlightController();
    else stopMotors();
  }
  
  if (millis() - lastTelemetry > 100) {
    lastTelemetry = millis();
    sendTelemetry();
  }
}

void readMPU() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)0x68, (uint8_t)14, (uint8_t)true);
  
  int16_t ax = (Wire.read()<<8)|Wire.read();
  int16_t ay = (Wire.read()<<8)|Wire.read();
  int16_t az = (Wire.read()<<8)|Wire.read();
  Wire.read(); Wire.read();
  int16_t gx = (Wire.read()<<8)|Wire.read();
  int16_t gy = (Wire.read()<<8)|Wire.read();
  int16_t gz = (Wire.read()<<8)|Wire.read();
  
  static float gyroXoff = 0, gyroYoff = 0;
  static bool calib = false, calibDone = false;
  static long sumX = 0, sumY = 0;
  static int count = 0;
  
  if (!calibDone) {
    sumX += gx; sumY += gy;
    if (++count > 100) {
      gyroXoff = sumX / 100.0;
      gyroYoff = sumY / 100.0;
      calibDone = true;
      Serial.println("MPU Calibrated");
    }
    return;
  }
  
  float gyroX = (gx - gyroXoff) / 131.0;
  float gyroY = (gy - gyroYoff) / 131.0;
  
  float accPitch = atan2(-ax, sqrt((float)ay*ay + (float)az*az)) * 180/PI;
  float accRoll = atan2(ay, az) * 180/PI;
  
  pitch = 0.98 * (pitch + gyroX * 0.004) + 0.02 * accPitch;
  roll = 0.98 * (roll + gyroY * 0.004) + 0.02 * accRoll;
  yaw += (gz/131.0) * 0.004;
}

void calculateAngles() {
  // Integrated with readMPU above
}

void updateFlightController() {
  pitch_err = pitch_input * 30 - pitch;
  roll_err = roll_input * 30 - roll;
  
  pitch_integral += pitch_err * 0.004;
  roll_integral += roll_err * 0.004;
  pitch_integral = constrain(pitch_integral, -50, 50);
  roll_integral = constrain(roll_integral, -50, 50);
  
  float pitch_diff = (pitch_err - last_pitch_err) / 0.004;
  float roll_diff = (roll_err - last_roll_err) / 0.004;
  last_pitch_err = pitch_err;
  last_roll_err = roll_err;
  
  float pitch_out = Kp * pitch_err + Ki * pitch_integral + Kd * pitch_diff;
  float roll_out = Kp * roll_err + Ki * roll_integral + Kd * roll_diff;
  float yaw_out = yaw_input * 100;
  
  int m1 = (throttle * 255) - pitch_out + roll_out - yaw_out;
  int m2 = (throttle * 255) - pitch_out - roll_out + yaw_out;
  int m3 = (throttle * 255) + pitch_out + roll_out + yaw_out;
  int m4 = (throttle * 255) + pitch_out - roll_out - yaw_out;
  
  motorVals[0] = constrain(m1, 0, 255);
  motorVals[1] = constrain(m2, 0, 255);
  motorVals[2] = constrain(m3, 0, 255);
  motorVals[3] = constrain(m4, 0, 255);
  
  setMotor(1, motorVals[0]);
  setMotor(2, motorVals[1]);
  setMotor(3, motorVals[2]);
  setMotor(4, motorVals[3]);
}

void setMotor(int num, int speed) {
  int pwmCh, in1, in2;
  switch(num) {
    case 1: pwmCh = PWM_CH_M1; in1 = MOTOR1_IN1; in2 = MOTOR1_IN2; break;
    case 2: pwmCh = PWM_CH_M2; in1 = MOTOR2_IN1; in2 = MOTOR2_IN2; break;
    case 3: pwmCh = PWM_CH_M3; in1 = MOTOR3_IN1; in2 = MOTOR3_IN2; break;
    case 4: pwmCh = PWM_CH_M4; in1 = MOTOR4_IN1; in2 = MOTOR4_IN2; break;
    default: return;
  }
  if (speed > 10) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    ledcWrite(pwmCh, speed);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(pwmCh, 0);
  }
}

void stopMotors() {
  for (int i = 1; i <= 4; i++) setMotor(i, 0);
  memset(motorVals, 0, sizeof(motorVals));
}

void sendTelemetry() {
  StaticJsonDocument<256> doc;
  doc["pitch"] = pitch;
  doc["roll"] = roll;
  doc["yaw"] = fmod(yaw, 360);
  doc["armed"] = armed;
  doc["m1"] = motorVals[0];
  doc["m2"] = motorVals[1];
  doc["m3"] = motorVals[2];
  doc["m4"] = motorVals[3];
  
  char buf[256];
  size_t len = serializeJson(doc, buf);
  ws.textAll(buf, len);
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, 
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      data[len] = 0;
      processCommand((char*)data);
    }
  }
}

void processCommand(char* json) {
  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, json);
  if (err) return;
  
  const char* cmd = doc["cmd"];
  if (strcmp(cmd, "stick") == 0) {
    throttle = (doc["l_y"] | 0.0) * -1;
    throttle = (throttle + 1) / 2;
    yaw_input = doc["l_x"] | 0.0;
    pitch_input = doc["r_y"] | 0.0;
    roll_input = doc["r_x"] | 0.0;
  }
  else if (strcmp(cmd, "arm") == 0) {
    armed = doc["state"] | false;
    if (!armed) stopMotors();
    digitalWrite(LED_PIN, armed ? HIGH : LOW);
  }
}