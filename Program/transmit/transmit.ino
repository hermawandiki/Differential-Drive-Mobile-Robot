char buffer[100];
int us_kiri, us_depan, us_kanan;
int enc_kiri, enc_kanan;
float yaw, pitch, roll;
float pos_x, pos_y, pos_w;

void setup() {
  Serial.begin(115200);
}

void loop() {
  us_kiri     = random(100,120);
  us_depan    = random(100,120);
  us_kanan    = random(100,120);  
  enc_kiri    = random(1000,2000);
  enc_kanan   = random(1000,2000);
  yaw       = random(-180.0,180.0);
  pitch     = random(-180.0,180.0);
  roll      = random(-180.0,180.0);
  pos_x     = random(500.0,600.0);
  pos_y     = random(500.0,600.0);
  pos_w     = random(-90.0,90.0);

  sprintf(buffer, "U%dU%dU%dE%dE%d", us_kiri, us_depan, us_kanan, enc_kiri, enc_kanan);
  Serial.print(buffer);
  String dataF = 'Y' + String(yaw,2) + 'P' + String(pitch,2) + 'R' + String(roll,2) + 'X' + String(pos_x,2) + 'Y' + String(pos_y,2) + 'W' + String(pos_w,2);
  Serial.println(dataF);
  delay(100);
}
