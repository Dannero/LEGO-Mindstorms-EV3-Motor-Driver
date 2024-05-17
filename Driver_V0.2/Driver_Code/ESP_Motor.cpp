/*
* File:   ESP32_Motor.cpp
*
* Desc:   Classes, methods and functions to initialize and control
*         Power Function motor via *ledc* functions, based on the Adafruit
*         Motor Shield library (https://www.arduino.cc/reference/en/libraries/adafruit-motor-shield-library/)
*
* Author: Daniel Blaško
*/


#if (ARDUINO >= 100)
  #include "Arduino.h"
#endif
#include "ESP_Motor.h"

uint8_t motor1_pwm, motor2_pwm, motor3_pwm, motor4_pwm;

// Initialize motors, set up PWM channels for both motor pins
inline void Init_PWM1() {
    motor1_pwm = 0;
    ledcSetup(PWM_CHANNEL_1A, MOTOR_PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_1B, MOTOR_PWM_FREQ, PWM_RESOLUTION);
  
    ledcAttachPin(MOTOR_1A, PWM_CHANNEL_1A);
    ledcAttachPin(MOTOR_1B, PWM_CHANNEL_1B);
}

inline void Init_PWM2() {
    motor2_pwm = 0;
    ledcSetup(PWM_CHANNEL_2A, MOTOR_PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_2B, MOTOR_PWM_FREQ, PWM_RESOLUTION);
  
    ledcAttachPin(MOTOR_2A, PWM_CHANNEL_2A);
    ledcAttachPin(MOTOR_2B, PWM_CHANNEL_2B);
}

inline void Init_PWM3() {
    motor3_pwm = 0;
    ledcSetup(PWM_CHANNEL_3A, MOTOR_PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_3B, MOTOR_PWM_FREQ, PWM_RESOLUTION);
  
    ledcAttachPin(MOTOR_3A, PWM_CHANNEL_3A);
    ledcAttachPin(MOTOR_3B, PWM_CHANNEL_3B);
}

inline void Init_PWM4() {
    motor4_pwm = 0;
    ledcSetup(PWM_CHANNEL_4A, MOTOR_PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_4B, MOTOR_PWM_FREQ, PWM_RESOLUTION);
  
    ledcAttachPin(MOTOR_4A, PWM_CHANNEL_4A);
    ledcAttachPin(MOTOR_4B, PWM_CHANNEL_4B);
}

// Motor class constructor
Motor::Motor(uint8_t num) {
  motor_num = num;

  switch (num) {
  case 1:
    Init_PWM1();
    break;
  case 2:
    Init_PWM2();
    break;
  case 3:
    Init_PWM3();
    break;
  case 4:
    Init_PWM4();
    break;
  }
}

// Generate PWM signals to run motors
void Motor::Run(uint8_t cmd) {
  uint8_t a, b;
  switch (motor_num) {
  case 1:
    a = PWM_CHANNEL_1A; 
    b = PWM_CHANNEL_1B; 
    pwm = motor1_pwm;
    break;

  case 2:
    a = PWM_CHANNEL_2A; 
    b = PWM_CHANNEL_2B; 
    pwm = motor2_pwm;
    break;

  case 3:
    a = PWM_CHANNEL_3A; 
    b = PWM_CHANNEL_3B; 
    pwm = motor3_pwm;
    break;

  case 4:
    a = PWM_CHANNEL_4A; 
    b = PWM_CHANNEL_4B; 
    pwm = motor4_pwm;
    break;

  default:
    return;
  }
  
  switch (cmd) {
  case FORWARD:
    ledcWrite(a, pwm);
    ledcWrite(b, 0);
    break;
  case BACKWARD:
    ledcWrite(a, 0);
    ledcWrite(b, pwm);
    break;
  case RELEASE:
    ledcWrite(a, 0);
    ledcWrite(b, 0);
    break;
  }
}

// Set duty cycle for motor PWM signals
void Motor::Set_Speed(uint8_t speed) {
  switch (motor_num) {
  case 1:
    motor1_pwm = speed;
    break;
  case 2:
    motor2_pwm = speed;
    break;
  case 3:
    motor3_pwm = speed;
    break;
  case 4:
    motor4_pwm = speed;
    break;
  }
}


