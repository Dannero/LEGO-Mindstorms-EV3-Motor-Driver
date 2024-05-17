/*
* File:   ESP32_Driver.h
*
* Desc:   Header file for the ESP32 variation of the motor driver code
*
* Author: Daniel Blaško
*/

#ifndef _ESP32_Driver_h_
#define _ESP32_Driver_h_

#include <SoftwareSerial.h>
#include <esp_adc_cal.h>
#include "ESP_Motor.h"

#define RX_PIN 22  //SoftwareSerial RX pin
#define TX_PIN 23  //SoftwareSerial TX pin
#define ADC_PIN 35 //Battery voltage measurement pin
#define LED_PIN 17 //Status LED pin

#define DEFAULT_BAUD_RATE 2400
#define COMM_BAUD_RATE 57600

#define HANDSHAKE_INTERVAL 80      // Interval in milliseconds
#define WRITE_INTERVAL 50          //UART write every 50ms 
#define BREAK_COND_TIME 500        //500ms UART break condition
#define NACK_TIMEOUT 1000          //Timeout for last received NACK
#define VOLTAGE_READ_INTERVAL 2000 //voltage readouts happen every 2 seconds 
#define LED_INTERVAL 300
#define LED_DUTY_CYCLE_LOW 51
#define LOW_BATTERY_TIMEOUT 10000  //10 seconds of LED flashing until the ESP enters deep sleep

#define ON_OFF_MASK 0x20           //Mask for motor on/off bit (bit 5)
#define POLARITY_MASK 0x10         //Mask for motor polarity bit (bit 4)
#define DUTY_CYCLE_MASK 0x0F       //Mask for motor duty cycle (bits 0-3)
#define EV3_WRITE_MASK 0x38        //Mask for EV3 Write message (getting incoming data byte length)

// Window size for moving average calculation of battery voltage
#define AVG_WINDOW_SIZE 10

// Connection states of the ESP
enum states {
  HANDSHAKE,
  CONNECTED,
  LOW_BATTERY
};

// Function for sending handshake data to the EV3 via SoftwareSerial
void Handshake(SoftwareSerial &soft_serial);
// Parse incoming message and set a motor accordingly
void setMotor(uint8_t val);
// Read battery voltage value and calculate average value, enter sleep mode if average drops below 10.8V
void voltage_control();
//Control the status LED based on the state of the driver;
void led_control();
//Calibrate ADC output
uint32_t readADC_Cal(int ADC_Raw);


#endif
