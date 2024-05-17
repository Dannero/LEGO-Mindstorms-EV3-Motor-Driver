/*
* File:   ESP32_Motor.h
*
* Desc:   Header file for the ESP32_Motor.cpp file
*
* Author: Daniel Blaško
*/

#ifndef _ESP_Motor_h_
#define _ESP_Motor_h_

// Motor GPIO pins
#define MOTOR_1A 33
#define MOTOR_1B 32
#define MOTOR_2A 26
#define MOTOR_2B 25
#define MOTOR_3A 12
#define MOTOR_3B 27
#define MOTOR_4A 16
#define MOTOR_4B 4

// PWM channels for motor control
#define PWM_CHANNEL_1A 1
#define PWM_CHANNEL_1B 2
#define PWM_CHANNEL_2A 3
#define PWM_CHANNEL_2B 4
#define PWM_CHANNEL_3A 5
#define PWM_CHANNEL_3B 6
#define PWM_CHANNEL_4A 7
#define PWM_CHANNEL_4B 8
#define LED_CHANNEL 9

// PWM attribute values
#define MOTOR_PWM_FREQ 1200
#define PWM_RESOLUTION 4

// Motor run modes
#define FORWARD 1
#define BACKWARD 2
#define RELEASE 3

class Motor
{
 public:
  Motor(uint8_t motor_num);
  void Run(uint8_t);
  void Set_Speed(uint8_t);

 private:
  uint8_t motor_num, pwm;
};

#endif
