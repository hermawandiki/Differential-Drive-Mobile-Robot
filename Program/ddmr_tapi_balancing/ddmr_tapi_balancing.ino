// MOTOR
#define PWM_L           15
#define IN1_L           2
#define IN2_L           4
#define IN1_R           16
#define IN2_R           17
#define PWM_R           5

// ENCODER
#define ENCA_L          13
#define ENCB_L          25
#define ENCA_R          33
#define ENCB_R          27

// WHEEL CONFIGURATION
#define WHEEL_DIAMETER 0.068 // in meters
#define WHEEL_RADIUS (WHEEL_DIAMETER / 2.0)
#define WHEELBASE 0.18 // in meters

// STATE MODE
#define TELEOPERATION   1
#define AUTONOMOUS      2
#define WALL_FOLLOWING  3
#define POINT_TO_POINT  4

char buffer[100];
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

float dt = 0.08;  // 80ms
int dtDelay = dt * 1000;
float vy, omega;

TaskHandle_t HandleMotorControl = NULL;
TaskHandle_t HandleMainProgram = NULL;

void setup() {
  Serial.begin(9600);
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

  xTaskCreatePinnedToCore(TaskMotorControl, "Kinematik & PID", 1024, NULL, 1, &HandleMotorControl, 0);
  xTaskCreatePinnedToCore(TaskMainProgram, "Main Program", 1024, NULL, 1, &HandleMainProgram, 0);
}

void loop() {
  //
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
      String data = Serial.readStringUntil('\n');

      if (data == "T")               { state = TELEOPERATION;  isRun = false; }
      else if (data == "A")          { state = AUTONOMOUS;     isRun = false; }
      else if (data == "W")          { state = WALL_FOLLOWING; isRun = false; }
      else if (data == "P")          { state = POINT_TO_POINT; isRun = false; }
      else if (data == "M")          { isRun = true;                          }
      else if (data == "N")          { isRun = false;                         }
      else if (data.startsWith("I")) { _vy = data.substring(1).toInt();       }
      else if (data.startsWith("K")) { _vy = -(data.substring(1).toInt());    }
      else if (data.startsWith("J")) { _omega = -(data.substring(1).toInt()); }
      else if (data.startsWith("L")) { _omega = data.substring(1).toInt();    }
      else {
        _vy     = 0;
        _omega  = 0;
        vy      = 0;
        omega   = 0;
      }
    }

    switch (state) {
      case TELEOPERATION :
        vy    = _vy;
        omega = _omega;
        break;
      case AUTONOMOUS :
        if(isRun) autonomous();
        else break;
        break;
      case WALL_FOLLOWING :
        break;
      case POINT_TO_POINT :
        break;
      default :
        break;
    }

//    Serial.print(speedActualL);
//    Serial.print("  ");
//    Serial.println(speedActualR);
//    Serial.print(state);
//    Serial.print(" ");
//    Serial.println(isRun);
    vTaskDelay(dtDelay / portTICK_PERIOD_MS);
  }
}
//
void autonomous(){
  vy    = 100;
  omega = 0;
  delay(10000);
  vy    = 0;
  omega = 0;
  isRun = false;
}
