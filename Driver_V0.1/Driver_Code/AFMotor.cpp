// Adafruit Motor shield library
// copyright Adafruit Industries LLC, 2009
// this code is public domain, enjoy!


#if (ARDUINO >= 100)
  #include "Arduino.h"
#endif

#include "AFMotor.h"



static uint8_t latch_state;
unsigned short motor1_pwm, motor2_pwm, motor3_pwm, motor4_pwm;


#if (MICROSTEPS == 8)
uint8_t microstepcurve[] = {0, 50, 98, 142, 180, 212, 236, 250, 255};
#elif (MICROSTEPS == 16)
uint8_t microstepcurve[] = {0, 25, 50, 74, 98, 120, 141, 162, 180, 197, 212, 225, 236, 244, 250, 253, 255};
#endif

AFMotorController::AFMotorController(void) {
    TimerInitalized = false;
}

void AFMotorController::enable(void) {
  pinMode(MOTORLATCH, OUTPUT);
  pinMode(MOTORENABLE, OUTPUT);
  pinMode(MOTORDATA, OUTPUT);
  pinMode(MOTORCLK, OUTPUT);

  latch_state = 0;

  latch_tx();  // "reset"

  digitalWrite(MOTORENABLE, LOW);
}


void AFMotorController::latch_tx(void) {
  uint8_t i;

  //LATCH_PORT &= ~_BV(LATCH);
  digitalWrite(MOTORLATCH, LOW);

  //SER_PORT &= ~_BV(SER);
  digitalWrite(MOTORDATA, LOW);

  for (i=0; i<8; i++) {
    //CLK_PORT &= ~_BV(CLK);
    digitalWrite(MOTORCLK, LOW);

    if (latch_state & _BV(7-i)) {
      //SER_PORT |= _BV(SER);
      digitalWrite(MOTORDATA, HIGH);
    } else {
      //SER_PORT &= ~_BV(SER);
      digitalWrite(MOTORDATA, LOW);
    }
    //CLK_PORT |= _BV(CLK);
    digitalWrite(MOTORCLK, HIGH);
  }
  //LATCH_PORT |= _BV(LATCH);
  digitalWrite(MOTORLATCH, HIGH);
}

static AFMotorController MC;

/******************************************
               MOTORS
******************************************/
inline void initPWM1() {
    motor1_pwm = 1023;
    pinMode(16, OUTPUT);
    analogWrite(16, motor1_pwm);
}

inline void setPWM1(uint8_t s) {
  motor1_pwm = s;
  analogWrite(16, motor1_pwm);
}

inline void initPWM2() {
    motor2_pwm = 1023;
    pinMode(14, OUTPUT);
    analogWrite(14, motor2_pwm);
}

inline void setPWM2(uint8_t s) {
  motor2_pwm = s;
  analogWrite(14, motor2_pwm);
}

inline void initPWM3() {
    motor3_pwm = 1023;
    pinMode(13, OUTPUT);
    analogWrite(13, motor3_pwm);
}

inline void setPWM3(uint8_t s) {
  motor3_pwm = s;
  analogWrite(13, motor3_pwm);
}



inline void initPWM4() {
    motor4_pwm = 1023;
    pinMode(12, OUTPUT);
    analogWrite(12, motor4_pwm);
}

inline void setPWM4(uint8_t s) {
  motor4_pwm = s;
  analogWrite(12, motor4_pwm);
}

AF_DCMotor::AF_DCMotor(uint8_t num) {
  motornum = num;

  MC.enable();

  switch (num) {
  case 1:
    latch_state &= ~_BV(MOTOR1_A) & ~_BV(MOTOR1_B); // set both motor pins to 0
    MC.latch_tx();
    initPWM1();
    break;
  case 2:
    latch_state &= ~_BV(MOTOR2_A) & ~_BV(MOTOR2_B); // set both motor pins to 0
    MC.latch_tx();
    initPWM2();
    break;
  case 3:
    latch_state &= ~_BV(MOTOR3_A) & ~_BV(MOTOR3_B); // set both motor pins to 0
    MC.latch_tx();
    initPWM3();
    break;
  case 4:
    latch_state &= ~_BV(MOTOR4_A) & ~_BV(MOTOR4_B); // set both motor pins to 0
    MC.latch_tx();
    initPWM4();
    break;
  }
}

void AF_DCMotor::run(uint8_t cmd) {
  uint8_t a, b;
  switch (motornum) {
  case 1:
    a = MOTOR1_A; b = MOTOR1_B; break;
  case 2:
    a = MOTOR2_A; b = MOTOR2_B; break;
  case 3:
    a = MOTOR3_A; b = MOTOR3_B; break;
  case 4:
    a = MOTOR4_A; b = MOTOR4_B; break;
  default:
    return;
  }
  
  switch (cmd) {
  case FORWARD:
    latch_state |= _BV(a);
    latch_state &= ~_BV(b); 
    MC.latch_tx();
    break;
  case BACKWARD:
    latch_state &= ~_BV(a);
    latch_state |= _BV(b); 
    MC.latch_tx();
    break;
  case RELEASE:
    latch_state &= ~_BV(a);     // A and B both low
    latch_state &= ~_BV(b); 
    MC.latch_tx();
    break;
  }
}

void AF_DCMotor::setSpeed(uint8_t speed) {
  switch (motornum) {
  case 1:
    setPWM1(speed); break;
  case 2:
    setPWM2(speed); break;
  case 3:
    setPWM3(speed); break;
  case 4:
    setPWM4(speed); break;
  }
}


