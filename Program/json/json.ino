#include <ArduinoJson.h>

int us_kiri, us_depan, us_kanan;
int Yaw, Pitch, Roll;
volatile int encoderL, encoderR;
float KpL = 10;
float KiL = 0;
float KdL = 0;
float KpR = 10;
float KiR = 0;
float KdR = 0;

void setup(){
  Serial.begin(115200);
  while(!Serial) continue;
}

void loop(){  
  StaticJsonDocument<256> doc;
  us_kiri = random(10,100);
  us_depan = random(10,100);
  us_kanan = random(10,100);
  
  Yaw = random(180,190);
  Pitch = random(180,190);
  Roll = random(180,190);
  
  encoderL = random(2000,2050);
  encoderR = random(2000,2050);
  
  doc["usl"]    = us_kiri;
  doc["usf"]    = us_depan;
  doc["usr"]    = us_kanan;
  doc["yaw"]    = Yaw;
  doc["pitch"]  = Pitch;
  doc["roll"]   = Roll;
  doc["encl"]   = encoderL;
  doc["encr"]   = encoderR;
  doc["kpl"]    = KpL;
  doc["kil"]    = KiL;
  doc["kdl"]    = KdL;
  doc["kpr"]    = KpR;
  doc["kir"]    = KiR;
  doc["kdr"]    = KdR;
  
  char jsonBuffer[256];
  serializeJson(doc, jsonBuffer);
  Serial.println(jsonBuffer);

  delay(100);
}
