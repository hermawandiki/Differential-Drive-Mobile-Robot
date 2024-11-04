#include <Arduino_FreeRTOS.h>
#include <ArduinoJson.h>
#include <util/atomic.h>
#include <Ultrasonic.h>
#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"

// MOTOR
#define PWM_R 11
#define IN1_R 10
#define IN2_R 9
#define IN1_L 8
#define IN2_L 7
#define PWM_L 6

// ENCODER
#define ENCA_L 2
#define ENCB_L 4
#define ENCA_R 3
#define ENCB_R 5

// SENSOR
#define TRIG_LEFT   36
#define ECHO_LEFT   34
#define TRIG_RIGHT  44
#define ECHO_RIGHT  42
#define TRIG_FRONT  40
#define ECHO_FRONT  38
#define INTERRUPT_PIN   23

// WHEEL CONFIGURATION
#define WHEEL_DIAMETER 0.068 // in meters
#define WHEEL_RADIUS (WHEEL_DIAMETER / 2.0)
#define WHEELBASE 0.18 // in meters

// STATE MODE
#define TELEOPERATION   1
#define AUTONOMOUS      2
#define WALL_FOLLOWING  3
#define POINT_TO_POINT  4

// ULTRASONIC
Ultrasonic US_Left (TRIG_LEFT,  ECHO_LEFT);
Ultrasonic US_Right(TRIG_RIGHT, ECHO_RIGHT);
Ultrasonic US_Front(TRIG_FRONT, ECHO_FRONT);
JsonDocument doc;

// IMU
MPU6050 mpu;
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float euler[3];
float ypr[3];
int Yaw, Pitch, Roll;
volatile bool mpuInterrupt = false;
void dmpDataReady() { mpuInterrupt = true; }

char buffer[50];
uint8_t state = 0;
bool isRun = false;

volatile int encoderL, encoderR;
int posReadL, posReadR;
int prevPosReadL, prevPosReadR;
int deltaPosL, deltaPosR;
float speedActualL, speedActualR;
float speedSetpointL, speedSetpointR;

float errorL, prevErrorL;
float integralL, derivativeL;
float controlL, velocityL;
float KpL = 10;
float KiL = 0;
float KdL = 0;
float errorR, prevErrorR;
float integralR, derivativeR;
float controlR, velocityR;
float KpR = 10;
float KiR = 0;
float KdR = 0;

int us_kiri, us_depan, us_kanan;
int enc_kiri, enc_kanan;
float yaw, pitch, roll;
float pos_x, pos_y, pos_w;

float dt = 0.08;  // 80ms
int dtDelay = dt * 1000;
float vy, omega;
long waktu, _waktu = 0;
int iterasi, interval;

void TaskMotorControl (void *pvParameters);
void TaskMainProgram  (void *pvParameters);
void TaskKirimSerial  (void *pvParameters);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  pinMode(PWM_L, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  pinMode(IN1_L, OUTPUT);
  pinMode(IN2_L, OUTPUT);
  pinMode(IN1_R, OUTPUT);
  pinMode(IN2_R, OUTPUT);
  pinMode(ENCA_L, INPUT_PULLUP);
  pinMode(ENCB_L, INPUT_PULLUP);
  pinMode(ENCA_R, INPUT_PULLUP);
  pinMode(ENCB_R, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCA_L), HandleENC_L, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_R), HandleENC_R, RISING);
  
  initMPU();
  
  xTaskCreate(TaskMotorControl, "Kinematika & PID", 1024, NULL, 1, NULL);
  xTaskCreate(TaskMainProgram, "Main Program", 1024, NULL, 1, NULL);
  xTaskCreate(TaskKirimSerial, "Kirim Data GUI", 1024, NULL, 1, NULL);
}

void loop(){
  // null
}
//
void initMPU(){
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  if(devStatus == 0){
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else{}
}
//
void HandleENC_L() {
  digitalRead(ENCB_L) ? encoderL++ : encoderL--;
}
//
void HandleENC_R() {
  digitalRead(ENCB_R) ? encoderR++ : encoderR--;
}
//
void driveMotor(int pwm1, int pwm2) {
  if (pwm1 > 0) {
    digitalWrite(IN1_L, LOW);
    digitalWrite(IN2_L, HIGH);
  } else {
    digitalWrite(IN1_L, HIGH);
    digitalWrite(IN2_L, LOW);
  }

  if (pwm2 > 0) {
    digitalWrite(IN1_R, LOW);
    digitalWrite(IN2_R, HIGH);
  } else {
    digitalWrite(IN1_R, HIGH);
    digitalWrite(IN2_R, LOW);
  }

  pwm1 = constrain(abs(pwm1), 0, 255);
  pwm2 = constrain(abs(pwm2), 0, 255);

  analogWrite(PWM_L, pwm1);
  analogWrite(PWM_R, pwm2);
}
//
void kinematic(float vy, float omega) {
  // Konversi vy (mm/s) menjadi m/s
  vy = vy / 1000.0;
  // Konversi w (deg/s) menjadi rad/s
  omega = omega * M_PI / 180.0;

  // Hitung kecepatan roda kiri (v_l) dan roda kanan (v_r) dalam m/s
  float vl = vy + (WHEELBASE / 2.0) * omega;
  float vr = vy - (WHEELBASE / 2.0) * omega;

  // Konversi dari m/s ke rps
  float rpsl = vl / (2.0 * M_PI * WHEEL_RADIUS);
  float rpsr = vr / (2.0 * M_PI * WHEEL_RADIUS);

  // Kirim ke PID
  int pwmL = PID_L(rpsl);
  int pwmR = PID_R(-rpsr);
  driveMotor(pwmL, pwmR);
}
//
int PID_L(float setpoint) {
  int out;
  // PID
  errorL = setpoint - speedActualL;
  integralL += errorL * dt;
  derivativeL = (errorL - prevErrorL) / dt;
  controlL = (KpL * errorL) + (KiL * integralL) + (KdL * derivativeL);
  // PWM
  velocityL = velocityL + controlL;
  out = constrain(velocityL, -255, 255);
  prevErrorL = errorL;

  return out;
}
//
int PID_R(float setpoint) {
  int out;
  // PID
  errorR = setpoint - speedActualR;
  integralR += errorR * dt;
  derivativeR = (errorR - prevErrorR) / dt;
  controlR = (KpR * errorR) + (KiR * integralR) + (KdR * derivativeR);
  // PWM
  velocityR = velocityR + controlR;
  out = constrain(velocityR, -255, 255);
  prevErrorR = errorR;

  return out;
}
//
void TaskMotorControl(void *pvParameters) {
  for (;;) {
    noInterrupts();
    posReadL = encoderL;
    posReadR = encoderR;
    kinematic(vy, omega);
    interrupts();
    deltaPosL = posReadL - prevPosReadL;
    deltaPosR = posReadR - prevPosReadR;
    prevPosReadL = posReadL;
    prevPosReadR = posReadR;
    speedActualL = deltaPosL / dt / 320;
    speedActualR = deltaPosR / dt / 320;
    kinematic(vy, omega);
    vTaskDelay(dtDelay / portTICK_PERIOD_MS);
  }
}
//
void TaskMainProgram(void *pvParameters) {
  float _vy, _omega;
  for (;;) {
    if (Serial.available() > 0) {
      char data = Serial.read();

      switch(data){
        case 'T' : state = TELEOPERATION;   isRun = false;  break;
        case 'A' : state = AUTONOMOUS;      isRun = false;  break;
        case 'W' : state = WALL_FOLLOWING;  isRun = false;  break;
        case 'P' : state = POINT_TO_POINT;  isRun = false;  break;
        case 'M' : isRun = true;                            break;
        case 'N' : isRun = false;                           break;
        case 'I' : _vy = Serial.parseFloat();               break;
        case 'K' : _vy = -(Serial.parseFloat());            break;
        case 'J' : _omega = -(Serial.parseFloat());         break;
        case 'L' : _omega = Serial.parseFloat();            break;
        case 'X' : _vy = 0; _omega = 0;                     break;
        default  :                                          break;
      }
      
      switch (state) {
        case TELEOPERATION :
          robotVelocity(_vy, _omega);
          break;
        case AUTONOMOUS :
          runAutonomous();
          break;
        case WALL_FOLLOWING :
          wallFollowing();
          break;
        case POINT_TO_POINT :
          break;
        default :
          break;
      }
    }

//    Serial.print(state);
//    Serial.print(" ");
//    Serial.println(isRun);
    vTaskDelay(dtDelay / portTICK_PERIOD_MS);
  }
}
//
void TaskKirimSerial(void *pvParameters){
  for(;;){
    if(!dmpReady) return;
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      pitch = (ypr[2] * 180 / M_PI + 180) + 0.5;
      roll  = (ypr[1] * 180 / M_PI + 180) + 0.5;
      yaw   = (ypr[0] * 180 / M_PI + 180) + 0.5;
    }
    int us_depan = US_Front.read();
    int us_kiri  = US_Left.read();
    int us_kanan = US_Right.read();
    
//    doc["us_kiri"]   = us_kiri;
//    doc["us_depan"]  = us_depan;
//    doc["us_kanan"]  = us_kanan;
//    doc["yaw"]       = yaw;
//    doc["pitch"]     = pitch;
//    doc["roll"]      = roll;
//    doc["encoder_l"] = encoderL;
//    doc["encoder_r"] = encoderR;
//    doc["pos_x"]     = pos_x;
//    doc["pos_y"]     = pos_y;
//    doc["pos_w"]     = pos_w;
//    
//    char jsonBuffer[256];
//    serializeJson(doc, jsonBuffer);
//    Serial.println(jsonBuffer);

    sprintf(buffer, "#U%dU%dU%dE%dE%d", us_kiri, us_depan, us_kanan, encoderL, encoderR);
    Serial.print(buffer);
    String dataF = 'Y' + String(yaw,2) + 'P'+ String(pitch,2) + 'R' + String(roll,2) + 'X' + String(pos_x,2) + 'Y' + String(pos_y,2) + 'W' + String(pos_w,2);
    Serial.println(dataF);
    
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}
//
void robotVelocity(float y, float w){
  vy    = y;
  omega = w;
}
//
void runAutonomous(){
  if (!isRun) {
    robotVelocity(0, 0);
    iterasi = 0;
    _waktu = 0;
  }
  else {
    waktu = millis();
    if (waktu - _waktu >= interval) {
    _waktu = waktu;
    
    switch (iterasi) {
      case 0 :
        robotVelocity(100, 0);
        interval = 2000;
        iterasi = 1;
        break;
      case 1 :
        robotVelocity(0, 90);
        interval = 1000;
        iterasi = 0;
        break;
      }
    }
  }
}
//
void wallFollowing(){
  
}
//
