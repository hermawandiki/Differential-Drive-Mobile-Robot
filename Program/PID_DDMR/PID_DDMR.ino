#include <Arduino_FreeRTOS.h>
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
#define TRIG_LEFT   44
#define ECHO_LEFT   42
#define TRIG_RIGHT  36
#define ECHO_RIGHT  34
#define TRIG_FRONT  40
#define ECHO_FRONT  38
#define INTERRUPT_PIN   23

// WHEEL CONFIGURATION
#define WHEEL_DIAMETER 0.068 // in meters
#define WHEEL_RADIUS (WHEEL_DIAMETER / 2.0)
#define WHEELBASE 0.18 // in meters

Ultrasonic US_Left (TRIG_LEFT,  ECHO_LEFT);
Ultrasonic US_Right(TRIG_RIGHT, ECHO_RIGHT);
Ultrasonic US_Front(TRIG_FRONT, ECHO_FRONT);

char buffer[100];

volatile int encoderL, encoderR;
int posReadL, posReadR;
int prevPosReadL, prevPosReadR;
int deltaPosL, deltaPosR;
float speedActualL, speedActualR;
float speedSetpointL, speedSetpointR;

float errorL, prevErrorL;
float integralL, derivativeL;
float controlL, velocityL;
float KpL = 20;
float KiL = 0;
float KdL = 0;
float errorR, prevErrorR;
float integralR, derivativeR;
float controlR, velocityR;
float KpR = 20;
float KiR = 0;
float KdR = 0;

float dt = 0.08;  // 80ms
int dtDelay = dt * 1000;
float vy, omega;

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

void TaskSpeedL     (void *pvParameters);
void TaskSpeedR     (void *pvParameters);
void TaskRobotRun   (void *pvParamters);
void TaskMainProgram(void *pvParamters);

void setup() {
  Serial.begin(9600);
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
  pinMode(INTERRUPT_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA_L), HandleENC_L, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_R), HandleENC_R, RISING);

  initMPU();

  xTaskCreate(TaskSpeedL, "Baca Speed L", 512, NULL, 1, NULL);
  xTaskCreate(TaskSpeedR, "Baca Speed R", 512, NULL, 1, NULL);
  xTaskCreate(TaskRobotRun, "Run Kinematic", 512, NULL, 1, NULL);
  xTaskCreate(TaskMainProgram, "BMain Control", 512, NULL, 1, NULL);
}
//
void loop() {
}
//
void HandleENC_L() {
  digitalRead(ENCB_L) ? encoderL++ : encoderR--;
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
void kinematic(float vy, float omega){
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
void TaskSpeedL(void *pvParameters){
  for(;;){
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      posReadL = encoderL;
    }
    deltaPosL = posReadL - prevPosReadL;
    prevPosReadL = posReadL;
    speedActualL = deltaPosL / dt / 320;
    vTaskDelay(dtDelay/portTICK_PERIOD_MS);
  }
}
//
void TaskSpeedR(void *pvParameters){
  for(;;){
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      posReadR = encoderR;
    }
    deltaPosR = posReadR - prevPosReadR;
    prevPosReadR = posReadR;
    speedActualR = deltaPosR / dt / 320;
    vTaskDelay(dtDelay/portTICK_PERIOD_MS);
  }
}
//
int PID_L(float setpoint){
  int out;
  // PID
  errorL = setpoint - speedActualL;
  integralL += errorL * dt;
  derivativeL = (errorL - prevErrorL) / dt;
  controlL = (KpL * errorL) + (KiL * integralL) + (KdL * derivativeL);
  // PWM
  velocityL = velocityL + controlL;
  out = min(max(velocityL, -255), 255);
  prevErrorL = errorL;

  return out;
}
//
int PID_R(float setpoint){
  int out;
  // PID
  errorR = setpoint - speedActualR;
  integralR += errorR * dt;
  derivativeR = (errorR - prevErrorR) / dt;
  controlR = (KpR * errorR) + (KiR * integralR) + (KdR * derivativeR);
  // PWM
  velocityR = velocityR + controlR;
  out = min(max(velocityR, -255), 255);
  prevErrorR = errorR;

  return out;
}
//
void TaskRobotRun(void *pvParameters){
  for(;;){
    kinematic(vy, omega);
    vTaskDelay(dtDelay/portTICK_PERIOD_MS);
  }
}
//
void TaskMainProgram(void *pvParameters){
  for(;;){
    if(Serial.available()>0){
      String input = Serial.readStringUntil('\n');
      vy    = input.toFloat();
      omega = 0;
    }
    
    if(!dmpReady) return;
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      Pitch = (ypr[2] * 180 / M_PI + 180) + 0.5;
      Roll  = (ypr[1] * 180 / M_PI + 180) + 0.5;
      Yaw   = (ypr[0] * 180 / M_PI + 180) + 0.5;
    }
    
    int Depan = US_Front.read();
    int Kiri  = US_Left.read();
    int Kanan = US_Right.read();
    
//    sprintf(buffer, "%d  %d  %d  \n", Yaw, Pitch, Roll);
//    sprintf(buffer, "%d  %d  %d  \n", Kiri, Depan, Kanan);
//    Serial.print(buffer);

//    Serial.print(speedActualL);
//    Serial.print("  ");
//    Serial.println(speedActualR);
    
    vTaskDelay(1/portTICK_PERIOD_MS);
  }
}
