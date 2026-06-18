#include <Wifi.h>
#include <esp_now.h>
#include <Wire.h>
#include <SparkFun_BMI270_Arduino_Library.h>

BMI270 imu;

// Transmitter Pairing 

uint8_t transmitter_mac[6] = {0x24, 0x6F, 0x28, 0x87, 0x12, 0x35};

typedef struct {
    float throttle;
    float x;
    float y; 
    float z;
    bool arm; 

}controlcommand;

controlcommand input;

volatile unsigned long last Packet Time = 0;

// ESC HANDLING 
const int ESC_PIN[4] = {25, 26, 27, 14};
const int PWM_FREQUENCY = 400;
const int PWM_RESOLUTION = 16;
const long PWM_MAX_DUTY = (1l << PWM_RESOLUTION) - 1;

void writeESC(int idx, float v){
    v = constrain(v, 0.0f, 1.0f);
    long duty = (long)us * PWM_frequency * PWM_MAX_DUTY / 1000000.0L;
    ledcWrite(ESC_PIN[idx],duty);
}

void setAllESC(float v) {
    for (int i = 0; i < 4; i++) writeESC(i, v);
}
  
void armESCs() {
    setAllESC(0.0); delay(2000);
    setAllESC(0.10); delay(2000);
    setAllESC(0.0); delay(2000);
}

float q0 =1, q1 = 0, q2 = 0, q3 = 0;
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;

float angleX = 0, angleY = 0, angleZ = 0;

const float DEG2RAD = 0.017453292f;

const float RAD2DEG = 57.2957795131f;

float mahonyKp = 2.0f; // for the complementary filter that fuses the gyro and accelerometer data we need the software to know how much to trust the gyro and how much to trust the accelerometer
float mahonyKi = 0.02f; // Cancels out residual bias of gyro using the acelerometer 

float ixCorr = 0, iyCorr = 0, izCorr =0; 

void updateOrientation(){
    imu.getSensorData();
    float ax =  IMU.getAccelX();                       // get sensor outputs 
    float ay =  IMU.getAccelY();
    float az =  IMU.getAccelZ();
    float gx =  (IMU.getGyroX() - gyroBiasX) * DEG2RAD;
    float gy =  (IMU.getGyroY() - gyroBiasY) * DEG2RAD;
    float gz =  (IMU.getGyroZ() - gyroBiasZ) * DEG2RAD;

    float acelNorm = sqrtf(ax*ax + ay*ay + az*az);

    if (acelNorm > 0.0001f){
        ax /= acelNorm; ay /= acelNorm; az /= acelNorm;

        float vx = 2.0f * (q1*q3 - q0*q2);  // define filter down points 
        float vy = 2.0f * (q0*q1 + q2*q3);
        float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

        float ex = ay*vz - az&vy;  // gravity missmatch
        float ey = az*vx - ax*vz;
        float ez = ax*vy - ay*vx; 

        float trust = 1.0f - fabsf(acelNorm - 1.0f);
        trust = constrain(trust, 0.0f, 1.0f);

        ixCorr += mahonyKi * ex * dt * trust; 
        iyCorr += mahonyKi * ey * dt * trust;
        izCorr += mahonyKi * ex * dt * trust;

        gx += mahonyKp * ex * trust + ixCorr; 
        gy += mahonyKp * ey * trust + iyCorr; 
        gz += mahonyKp * ez * trust + izCorr; 
    }

    float qDot0 = 0.5f * (-q1*gx - q2*gy - q3*gz);
    float qDot1 = 0.5f * ( q0*gx + q2*gz - q3*gy);
    float qDot2 = 0.5f * ( q0*gy - q1*gz + q3*gx);
    float qdot3 = 0.5f * ( q0*gz + q1*gy - q2*gx);

    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt; 
    q3 += qdot3 * dt;

    float qNorm = sqrtf(q0*q0 + q1*q1 + q2*q2+ q3*q3);

    q0 /= qNorm; q1 /= qNorm; q2 /= qNorm; q3 /= qNorm;

    angleX = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)) * RAD2DEG;
    angleY = asinf(constrain(2.0f*(q0*q2 - q1*q3), -1.0f, 1.0f)) * RAD2DEG;
    angleZ = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3)) * RAD2DEG;
}

float ptermx = 0.02, itermx = 0.0, dtermX = 0.005; 
float ptermy = 0.02, itermy = 0.0, dtermy = 0.005; 

float Ztermp = 0.01;

float pidintegx = 0, pidintegy = 0;
float lasterrorx = 0, lasterrory = 0; 
float dFiltX = 0, dFiltY = 0;
const float D_filter = 0.2f;

float pid(float setpoint, float measured, float &integ, float &lastErr, float &dFilt, float kp, float ki, float kd, float dt){
    float err = setpoint - measured;
    integ += err * dt;
    integ = constrain(integ, -0.5f, 0.5f);
    float dRaw = (err - lastErr) / dt;
    dFilt += D_filter * (dRaw-dFilt);
    lastErr = err;

    return kp * err + ki*integ + kd*dFilt; 
}

void onRecieval(const esp_now_recv_info_t *info, const uint8_t *data, int len){
    if (memcmp(info->src_addr, transmitter_mac, 6) != 0) {
        return;
    }

    if (len != sizeof(controlcommand)){
        return
    }

    memcpy(&input, data, sizeof(input));

    lastPacketTime = millis();
}

unsigned long lastTime;

void setup(){
    Serial.begin(115200);
    Wire.begin();

    if (imu.beginI2C() != BMI2_OK){
        Serial.println("BMI 270 NOT FOUND D:");
        while (1) delay(10);
    }

    Serial.println("Calibrating GYRO stay still plz");

    long n = 0;
    
    double sumX = 0, sumY = 0, sumZ = 0;

    while (millis() - calStart < 1500) {
        imu.getSensorData();
        sumX += imu.data.gyroX;
        sumY += imu.data.gyroY;
        sumZ += imu.data.gyroZ;
        n++;
        delay(2);
    }
    BiasX = sumX / n;
    BiasY = sumY / n;
    BiasZ = sumZ / n;

    Serial.println("sensor happy :D ");

    Wifi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW is screwed");
        while (1) delay(10);
    }
    esp_now_register_recv_cb(onReceive);

    for (int i = 0; i < 4; i++) ledcAttach(ESC_PIN[i], PWM_FREQ, PWM_RES);

    armESCs();

    Serial.print("Flight comptuer mac adress");
    Serial.println(WiFi.macAddress());

    lastTime = micros();
    lastPacketTime = millis();
}

void loop{
    unsigned long now = micros();
    float dt = (now - lastTime)* 1e-6f;

    lastTime = now;

    if (dt <=0 || dt>0.5f) dt = 0.002f;

    updateOrientation(dt);

    if (millis() - lastPacketTime > 100){
        input.arm = false;
    }

    if (!input.arm){
        setAllESC(0.0);
        pidintegx = 0; pidintegy = 0;
    }

    float outX = pid(input.x, angleX, pidintegx,lasterrorx, dFiltX,ptermx,itermx,dtermx);
    float outy = pid(input.y, angleY, pidintegy,lasterrory, dFilty,ptermy,itermy,dtermy);
    float outZ =Ztermp * (input.z - angleZ);

    float t = input.throttle;

    float m1 = t + outY + outX - outZ;
    float m2 = t + outY - outX + outZ;
    float m3 = t - outY + outX + outZ;
    float m4 = t - outY - outX - outZ;
  
    writeESC(0, m1);
    writeESC(1, m2);
    writeESC(2, m3);
    writeESC(3, m4);


}