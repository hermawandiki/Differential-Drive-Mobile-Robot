#include <Ultrasonic.h>
#include <Encoder.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// Sensor
#define Trig_Kiri 44
#define Echo_Kiri 42
#define Trig_Kanan 36
#define Echo_Kanan 34
#define Trig_Depan 40
#define Echo_Depan 38
#define ENCA_L 2
#define ENCB_L 4
#define ENCA_R 3
#define ENCB_R 5
// Aktuator
#define INA_L 8
#define INB_L 7
#define PWML 6
#define INA_R 10
#define INB_R 9 
#define PWMR 11

#define Kiri 1
#define Kanan 2
#define CW 1
#define CCW -1
#define Stop 0

Ultrasonic US_Kiri(Trig_Kiri, Echo_Kiri);
Ultrasonic US_Kanan(Trig_Kanan, Echo_Kanan);
Ultrasonic US_Depan(Trig_Depan, Echo_Depan);

Encoder ENCL(ENCA_L, ENCB_L);
Encoder ENCR(ENCA_R, ENCB_R);

MPU6050 mpu;
#define INTERRUPT_PIN 19  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13       // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;
// MPU control/status vars
bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer
Quaternion q;         // [w, x, y, z]         quaternion container
VectorFloat gravity;  // [x, y, z]            gravity vector
float euler[3];  // [psi, theta, phi]    Euler angle container
float ypr[3];    // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}
unsigned int Yaw, Pitch, Roll;

int error, lastError;
double Kp = 10.0;
double Ki = 0;
double Kd = 0;

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
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
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
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);  // 1688 factory default for my test chip
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    // Serial.println(F("Enabling DMP...")); coba
    mpu.setDMPEnabled(true);

    //  // enable Arduino interrupt detection
    // Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    // Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    // Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    //       Serial.println(F("DMP ready! Waiting for first interrupt...")); //coba1
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    //    Serial.print(F("DMP Initialization failed (code ")); coba
    //   Serial.print(devStatus); coba
    //  Serial.println(F(")")); coba
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  // error = 180 - Yaw;
  // int P = Kp * error;
  // int I = Ki * (error + lastError);
  // int D = Kd * (error - lastError);
  // lastError = error;
  // int outPID = constrain(P+I+D, -100, 100);
  // Serial.println(outPID);

  // motor(50+outPID, 50+(-outPID));
  // while(US_Depan.read()<10){
  //   motor(0,0);
  // }

  // Serial.print("Kiri : ");
  // Serial.print(US_Kiri.read());
  // Serial.print("\t");
  // Serial.print("Depan : ");
  // Serial.print(US_Depan.read());
  // Serial.print("\t");
  // Serial.print("Kanan : ");
  // Serial.print(US_Kanan.read());
  
  Serial.print("\t");
  Serial.print("ENCL : ");
  Serial.print(ENCL.read());
  Serial.print("\t");
  Serial.print("ENCR : ");
  Serial.print(ENCR.read());
  
  Serial.println();

  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    unsigned int cbuff;
    uint8_t ceksum = 0;

    Pitch = (int)(((ypr[2] * 180 / M_PI) + 180.0) + 0.5); // depan + belakang - 
    Roll = (int)(((ypr[1] * 180 / M_PI) + 180.0) + 0.5); // kanan + kiri -
    Yaw = (int)(((ypr[0] * 180 / M_PI) + 180.0) + 0.5); // kanan + kiri -

    // Serial.print(Yaw);
    // Serial.print('\t');
    // Serial.print(Pitch);
    // Serial.print('\t');
    // Serial.print(Roll);
    // Serial.print('\n');
  }
}