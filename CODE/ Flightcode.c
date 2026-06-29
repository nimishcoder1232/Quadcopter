#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <SparkFun_BMI270_Arduino_Library.h>


BMI270 imu;

typedef struct {
  float throttle;
  float x;
  float y;
  float z;
  bool  arm;
} ControlPacket;

ControlPacket input;
volatile unsigned long lastPacketTime = 0;

// ── Config ────────────────────────────────────────────────────────────────
const int ESC_PINS[]  = {42, 4, 5, 2};
const int NUM_ESCS    = 4;
const int PWM_FREQ    = 50;
const int PWM_BITS    = 12;
const int PWM_MIN_US  = 1000;
const int PWM_MAX_US  = 2000;

// ── Helpers ───────────────────────────────────────────────────────────────
uint32_t usToDuty(int pulseUs) {
    return (uint32_t)(pulseUs * 4095.0f / 20000.0f);
}

void setESC(int pin, int pulseUs) {
    pulseUs = constrain(pulseUs, PWM_MIN_US, PWM_MAX_US);
    ledcWrite(pin, usToDuty(pulseUs));
}




// ============== ORIENTATION (QUATERNION) ==============
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
float angleX = 0, angleY = 0, angleZ = 0;

const float DEG2RAD = 0.017453292f;
const float RAD2DEG = 57.295780f;

float mahonyKp = 2.0f;
float mahonyKi = 0.02f;
float ixCorr = 0, iyCorr = 0, izCorr = 0;

void updateOrientation(float dt) {
  imu.getSensorData();
  float ax = imu.data.accelX;
  float ay = imu.data.accelY;
  float az = imu.data.accelZ;
  float gx = (imu.data.gyroX - gyroBiasX) * DEG2RAD;
  float gy = (imu.data.gyroY - gyroBiasY) * DEG2RAD;
  float gz = (imu.data.gyroZ - gyroBiasZ) * DEG2RAD;

  float accelNorm = sqrtf(ax*ax + ay*ay + az*az);
  if (accelNorm > 0.0001f) {
    ax /= accelNorm; ay /= accelNorm; az /= accelNorm;

    float vx = 2.0f * (q1*q3 - q0*q2);
    float vy = 2.0f * (q0*q1 + q2*q3);
    float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    float ex = ay*vz - az*vy;
    float ey = az*vx - ax*vz;
    float ez = ax*vy - ay*vx;

    float trust = 1.0f - fabsf(accelNorm - 1.0f);
    trust = constrain(trust, 0.0f, 1.0f);

    ixCorr += mahonyKi * ex * dt * trust;
    iyCorr += mahonyKi * ey * dt * trust;
    izCorr += mahonyKi * ez * dt * trust;

    gx += mahonyKp * ex * trust + ixCorr;
    gy += mahonyKp * ey * trust + iyCorr;
    gz += mahonyKp * ez * trust + izCorr;
  }

  float qDot0 = 0.5f * (-q1*gx - q2*gy - q3*gz);
  float qDot1 = 0.5f * ( q0*gx + q2*gz - q3*gy);
  float qDot2 = 0.5f * ( q0*gy - q1*gz + q3*gx);
  float qDot3 = 0.5f * ( q0*gz + q1*gy - q2*gx);

  q0 += qDot0 * dt;
  q1 += qDot1 * dt;
  q2 += qDot2 * dt;
  q3 += qDot3 * dt;

  float qNorm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 /= qNorm; q1 /= qNorm; q2 /= qNorm; q3 /= qNorm;

  angleX = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)) * RAD2DEG;
  angleY = asinf(constrain(2.0f*(q0*q2 - q1*q3), -1.0f, 1.0f)) * RAD2DEG;
  angleZ = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3)) * RAD2DEG;
}

// ============== PID ==============
float KpX = 0.7, KiX = 0.005, KdX = 0.06;
float KpY = 0.7, KiY = 0.005, KdY = 0.06;
float KpZ = 0.3;

float pidIntegX = 0, pidIntegY = 0;
float lastErrX = 0, lastErrY = 0;
float dFiltX = 0, dFiltY = 0;
const float D_FILTER_ALPHA = 0.3f;

float pid(float setpoint, float measured, float &integ, float &lastErr, float &dFilt,
           float kp, float ki, float kd, float dt) {
  float err = setpoint - measured;
  integ += err * dt;
  integ = constrain(integ, -50.0f, 50.0f);
  float dRaw = (err - lastErr) / dt;
  dFilt += D_FILTER_ALPHA * (dRaw - dFilt);
  lastErr = err;
  return kp*err + ki*integ + kd*dFilt;
}

// ============== ESP-NOW ==============
void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(ControlPacket)) return;
  memcpy(&input, data, sizeof(input));
  lastPacketTime = millis();
}

// ============== SETUP ==============
unsigned long lastTime;


void setup() {
    Serial.begin(115200);

    for (int i = 0; i < NUM_ESCS; i++) {
        ledcAttach(ESC_PINS[i], PWM_FREQ, PWM_BITS);
        ledcWrite(ESC_PINS[i], usToDuty(PWM_MIN_US));
    }

    Serial.println("Arming – keep props off!");
    delay(3000);
    Serial.println("Armed.");

    Wire.begin();

  if (imu.beginI2C() != BMI2_OK) {
    Serial.println("BMI270 not found - check wiring");
    while (1) delay(10);
  }
  Serial.println("Calibrating gyro - keep the frame still...");
  long n = 0;
  double sumX = 0, sumY = 0, sumZ = 0;
  unsigned long calStart = millis();
  while (millis() - calStart < 1500) {
    imu.getSensorData();
    sumX += imu.data.gyroX;
    sumY += imu.data.gyroY;
    sumZ += imu.data.gyroZ;
    n++;
    delay(2);
  }
  gyroBiasX = sumX / n;
  gyroBiasY = sumY / n;
  gyroBiasZ = sumZ / n;
  Serial.println("Gyro calibration done.");

  WiFi.mode(WIFI_STA);

  Serial.println("Setting WiFi mode...");
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (1) delay(10);
  }

  Serial.println("Initializing ESP-NOW...");


  esp_now_register_recv_cb(onReceive);

  Serial.println("Registering callback...");

  Serial.print("This board's MAC (give it to the transmitter sketch): ");
  Serial.println(WiFi.macAddress());

  lastTime = micros();
  lastPacketTime = millis();
    
}

// ── Loop ──────────────────────────────────────────────────────────────────
void loop() {
  unsigned long now = micros();
  float dt = (now - lastTime) * 1e-6f;
  lastTime = now;
  if (dt <= 0 || dt > 0.5f) dt = 0.002f;

  updateOrientation(dt);

/*
  if (millis() - lastPacketTime > 100) {
    input.arm = false;
  }
*/
  if (!input.arm) {
    setESC(ESC_PINS[0], 1000);
    setESC(ESC_PINS[1], 1000);
    setESC(ESC_PINS[2], 1000);
    setESC(ESC_PINS[3], 1000);
    
    pidIntegX = 0; pidIntegY = 0;
    Serial.println("ABORTED");
    return;
  }

  float outX = -(pid(-input.x, angleY, pidIntegX, lastErrX, dFiltX, KpX, KiX, KdX, dt));
  float outY = -(pid(input.y, angleX, pidIntegY, lastErrY, dFiltY, KpY, KiY, KdY, dt));
  float outZ = KpZ * (input.z - (imu.data.gyroZ - gyroBiasZ));


  float t = input.throttle * 100;

  // quad-X mixing
  float m1 = t + outY + outX - outZ;
  float m2 = t + outY - outX + outZ;
  float m3 = t - outY + outX + outZ;
  float m4 = t - outY - outX - outZ;

  int us1 = constrain(1000 + 10 * m1, 1000, 2000);
  int us2 = constrain(1000 + 10 * m2, 1000, 2000);
  int us3 = constrain(1000 + 10 * m3, 1000, 2000);
  int us4 = constrain(1000 + 10 * m4, 1000, 2000);  
  
  
  setESC(ESC_PINS[0], us1);
  setESC(ESC_PINS[1], us2);
  setESC(ESC_PINS[2], us3);
  setESC(ESC_PINS[3], us4);
}