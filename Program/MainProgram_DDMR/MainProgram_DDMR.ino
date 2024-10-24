#include <Ultrasonic.h>
#include <Encoder.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// Sensor
#define Trig_Kiri   44
#define Echo_Kiri   42
#define Trig_Kanan  36
#define Echo_Kanan  34
#define Trig_Depan  40
#define Echo_Depan  38
#define ENCA_L      2
#define ENCB_L      4
#define ENCA_R      3
#define ENCB_R      5

// Aktuator
#define INA_L       8
#define INB_L       7
#define PWML        6
#define INA_R       10
#define INB_R       9 
#define PWMR        11

Ultrasonic US_Kiri(Trig_Kiri, Echo_Kiri);
Ultrasonic US_Kanan(Trig_Kanan, Echo_Kanan);
Ultrasonic US_Depan(Trig_Depan, Echo_Depan);

Encoder ENCL(ENCA_L, ENCB_L);
Encoder ENCR(ENCA_R, ENCB_R);

MPU6050 mpu;
#define INTERRUPT_PIN 19
#define LED_PIN 13
bool blinkState = false;
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
volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}
unsigned int Yaw, Pitch, Roll;

void motor(int pwm1, int pwm2){
  if(pwm1>0){
    digitalWrite(INA_L, LOW);
    digitalWrite(INB_L, HIGH);
  }else{
    digitalWrite(INA_L, HIGH);
    digitalWrite(INB_L, LOW);
  }

  if(pwm2>0){
    digitalWrite(INA_R, HIGH);
    digitalWrite(INB_R, LOW);
  }else{
    digitalWrite(INA_R, LOW);
    digitalWrite(INB_R, HIGH);
  }

  pwm1 = constrain(abs(pwm1), 0, 255);
  pwm2 = constrain(abs(pwm2), 0, 255);
  
  analogWrite(PWML, pwm1);
  analogWrite(PWMR, pwm2);
}

void setup() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
  #endif

  Serial.begin(115200);
  for(uint8_t i=6; i<=11; i++){
    pinMode(i, OUTPUT);
  }

  delay(2000);
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {}
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  if (!dmpReady) return;
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    Pitch = (int)(((ypr[2] * 180 / M_PI) + 180.0) + 0.5); // depan + belakang - 
    Roll  = (int)(((ypr[1] * 180 / M_PI) + 180.0) + 0.5); // kanan + kiri -
    Yaw   = (int)(((ypr[0] * 180 / M_PI) + 180.0) + 0.5); // kanan + kiri -
  }
}