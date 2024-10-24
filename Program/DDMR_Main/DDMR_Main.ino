#include "Arduino.h"
#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <util/atomic.h>
#include <Ultrasonic.h>
#include <Encoder.h>
#include "myDefinition.h"

Ultrasonic US_Left (TRIG_LEFT,  ECHO_LEFT);
Ultrasonic US_Right(TRIG_RIGHT, ECHO_RIGHT);
Ultrasonic US_Front(TRIG_FRONT, ECHO_FRONT);

Encoder ENC_Left (ENCA_L, ENCB_L);
Encoder ENC_Right(ENCA_R, ENCB_R);

char buffer[50] = "";
volatile long encL, encR;

void setup(){
  Serial.begin(9600);
}

void loop(){
  Serial.print(ENC_Left.read());
  Serial.print(" ");
  Serial.println(ENC_Right.read());
}
