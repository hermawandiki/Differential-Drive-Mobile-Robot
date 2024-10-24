#include <ArduinoJson.h>
JsonDocument doc;

char buffer[100];
int us_kiri, us_depan, us_kanan;
int enc_kiri, enc_kanan;
float yaw, pitch, roll;
float pos_x, pos_y, pos_w;

void setup(){
  Serial.begin(115200);
  while(!Serial) continue;
}

void loop(){
  us_kiri     = random(100,120);
  us_depan    = random(100,120);
  us_kanan    = random(100,120);  
  enc_kiri    = random(1000,2000);
  enc_kanan   = random(1000,2000);
  yaw       = random(-90.0,90.0);
  pitch     = random(-15.0,15.0);
  roll      = random(-15.0,15.0);
  pos_x     = random(500.0,600.0);
  pos_y     = random(500.0,600.0);
  pos_w     = random(-90.0,90.0);
  
  doc["us_kiri"]  = us_kiri;
  doc["us_depan"] = us_depan;
  doc["us_kanan"] = us_kanan;
  
  doc["yaw"] = yaw;
  doc["pitch"] = pitch;
  doc["roll"] = roll;
  
  doc["encoder_l"] = enc_kiri;
  doc["encoder_r"] = enc_kanan;
  
  doc["pos_x"] = pos_x;
  doc["pos_y"] = pos_y;
  doc["pos_w"] = pos_w;
  
  // Serialisasi objek JSON ke string dan kirim ke serial
  char jsonBuffer[256];
  serializeJson(doc, jsonBuffer);

  Serial.println(jsonBuffer);

  // Tunggu 1 detik sebelum mengirim data lagi
  delay(100);
}
