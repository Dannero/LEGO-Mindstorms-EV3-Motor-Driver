/*
* File:   ESP32_Driver.ino
*
* Desc:   ESP-WROOM32E Arduino framework program for communicating with a LEGO Mindstorms EV3 brick
*         using the LEGO UART Sensor Protocol and running motors PWM
*
* Author: Daniel Blaško
*/

#include "ESP32_Driver.h"

uint8_t state = 0;
bool break_cond_ongoing = false;
bool break_high_ongoing = false;
bool handshake_finished = false;
bool checksum_expected = false;

unsigned long current_millis = 0;
unsigned long previous_millis = 0;
unsigned long last_write_millis = 0;
unsigned long last_nack_millis = 0;
unsigned long break_cond_millis = 0;
unsigned long break_high_millis = 0;
unsigned long last_voltage_read_millis = 0;
unsigned long low_voltage_trigger_millis = 0;
unsigned long led_current_millis = 0;
unsigned long led_previous_millis = 0;

uint8_t val = 0;
uint8_t bytes_to_read;  //Set number of bytes to read from the WRITE EV3 Message (ref: https://sourceforge.net/p/lejos/wiki/UART%20Sensor%20Protocol/)
uint16_t voltage_readout = 0; //Value of the ADC pin readout
uint8_t led_duty_cycle = 0;

// Moving average variables
uint8_t avg_index = 0;
uint16_t avg_window[AVG_WINDOW_SIZE];

//Set up all four motors
Motor motor1(1);
Motor motor2(2);
Motor motor3(3);
Motor motor4(4);
//Array of motors (easier motor setting by just setting the motor at a parsed index)
Motor motors[4] = {motor1, motor2, motor3, motor4};
uint8_t motor_pwms[4] = {0,0,0,0};
uint8_t pwm_send_index = 0;

SoftwareSerial soft_serial(RX_PIN, TX_PIN);


void Handshake(SoftwareSerial &soft_serial) {
  //TYPE
  soft_serial.write(0x40);
  soft_serial.write(0x64); //type - 100
  soft_serial.write(0xDB); //XOR

  //MODES 
  soft_serial.write(0x49); //Mode 0
  soft_serial.write('\0'); 
  soft_serial.write('\0');
  soft_serial.write(0xB6); //XOR


  //Communication speed (Optional, if not specified, then stays at 2400 baud) 
  //57600 (Little endian)
  soft_serial.write(0x52);
  soft_serial.write('\0');
  soft_serial.write(0xE1);
  soft_serial.write('\0');
  soft_serial.write('\0');
  soft_serial.write(0x52 ^ '\0' ^ 0xE1 ^ '\0' ^ '\0' ^ 0xFF);

  //Mode 0 Name
  soft_serial.write(0x90);
  soft_serial.write('\0'); 
  soft_serial.write(0x75); //u
  soft_serial.write(0x61); //a
  soft_serial.write(0x72); //r
  soft_serial.write(0x74); //t
  soft_serial.write(0x90 ^ '\0' ^ 0x75 ^ 0x61 ^ 0x72 ^ 0x74 ^ 0xFF); //XOR

  // RAW value range (32bit floating point)
  soft_serial.write(0x98);
  soft_serial.write(0x01);
  soft_serial.write('\0');  //Min value: 0
  soft_serial.write('\0');
  soft_serial.write('\0');
  soft_serial.write('\0');
  soft_serial.write('\0'); //Max value: 255
  soft_serial.write('\0');
  soft_serial.write(0x7F);
  soft_serial.write(0x43);
  soft_serial.write(0x98 ^ 0x01 ^ '\0' ^ 0x7F ^ 0x43 ^ 0xFF); //XOR

  //FORMAT
  soft_serial.write(0x90);
  soft_serial.write(0x80);
  soft_serial.write(0x01); //1 item
  soft_serial.write('\0'); //Data type 0 - 8bit int
  soft_serial.write(0x03); //Number of digits to show
  soft_serial.write('\0'); //Number of decimals to show
  soft_serial.write(0x90 ^ 0x80 ^ 0x01 ^ '\0' ^ 0x03 ^ '\0' ^ 0xFF); //XOR

  //ACK
  soft_serial.write(0x04);
}


void setup() {
  //Pin OUTPUT mode for 500ms UART break condition
  pinMode(TX_PIN, OUTPUT); 
  soft_serial.begin(DEFAULT_BAUD_RATE);
  // Default state, trying to connect to EV3
  state = HANDSHAKE;
  // Set up LED PWM channel and pin
  ledcSetup(LED_CHANNEL, MOTOR_PWM_FREQ, 8);
  ledcAttachPin(LED_PIN, LED_CHANNEL);
}

void loop() {
  current_millis = millis();
  // Read battery voltage
  if (current_millis - last_voltage_read_millis > VOLTAGE_READ_INTERVAL) {
    voltage_control();  
    last_voltage_read_millis = current_millis;
  }

  led_control();

  
  // Start break and handshake procedure if ACK wasn't received within 80ms after the handshake
  if ((state == HANDSHAKE) && (!break_cond_ongoing) && (!break_high_ongoing) && (current_millis - previous_millis > HANDSHAKE_INTERVAL)) {
    digitalWrite(TX_PIN, LOW); //Begin 500ms UART break condition for EV3
    break_cond_millis = current_millis;
    break_cond_ongoing = true;
    handshake_finished = false;
  }
  
  // End break condition LOW, set pin to HIGH for 1ms
  if ((break_cond_ongoing) && (!break_high_ongoing) && (current_millis - break_cond_millis >= BREAK_COND_TIME)) {
    digitalWrite(TX_PIN, HIGH);
    current_millis = millis();
    break_high_millis = current_millis;
    break_cond_ongoing = false;
    break_high_ongoing = true;
  }

  // 1ms passed since TX HIGH, begin handshake
  if ((break_high_ongoing) && (!handshake_finished) && (current_millis - break_high_millis > 0)) {
    soft_serial.begin(DEFAULT_BAUD_RATE);
    Handshake(soft_serial);
    digitalWrite(TX_PIN, LOW);
    current_millis = millis();
    previous_millis = current_millis;
    break_high_ongoing = false;
    handshake_finished = true;
  }

  if (handshake_finished) {

    if(soft_serial.available()) {
        char received_char = soft_serial.read();

        switch(received_char) {
          //ACK received, begin regular communication
          case 0x04:
              current_millis = millis();
              if ((current_millis - previous_millis <= HANDSHAKE_INTERVAL)) {
                state = CONNECTED;
                // Change comm baud rate if specifying different than default comm speeed (comment the code if keeping default speed)
                soft_serial.begin(COMM_BAUD_RATE);
              }
              break;

          //NACK received, send information
          case 0x02: 
              //Send 1 byte + arbitrary byte for read compatibility
              soft_serial.write(0xC8);
              soft_serial.write(motor_pwms[pwm_send_index] + 16*pwm_send_index);
              soft_serial.write('\0');
              soft_serial.write(0xC8 ^ (motor_pwms[pwm_send_index] + 16*pwm_send_index) ^ '\0' ^ 0xFF);

              //Increment index for multiplexing
              pwm_send_index++;
              if (pwm_send_index == 4) {
                pwm_send_index = 0;
              }

              last_write_millis = current_millis;
              last_nack_millis = current_millis;
              break;

          //WRITE/Data/checksum message
          default: 
              // Ignore checksum byte
              if (checksum_expected) {
                checksum_expected = false;
              }

              // WRITE message
              else if ((bytes_to_read == 0) && (received_char >= 0x44) && (received_char <= 0x7C)) {
                received_char = received_char & EV3_WRITE_MASK; //AND mask to get length exponent
                received_char = received_char >> 3;
                bytes_to_read = pow(2, received_char); //Set incoming data length to 2^result
              }
              //Set Motor based on message byte, ignore if 0x11 (unknown) byte
              else if (bytes_to_read > 0) {
                  bytes_to_read--;
                  if (received_char != 0x11) {
                    setMotor(received_char);
                  }
                  // Prepare for checksum byte after parsing last payload byte
                  if (bytes_to_read == 0) {
                    checksum_expected = true;
                  }
              }
              break;
        }
    }

    //Write data independent of NACK message at set interval
    if ((current_millis - last_write_millis >= WRITE_INTERVAL) && (state == CONNECTED)) {
      // Uncomment if you want to send data independently in an interval of <1,100> ms
      // soft_serial.write(0xC8);
      // soft_serial.write(motor_pwms[pwm_send_index] + 16*pwm_send_index);
      // soft_serial.write('\0');
      // soft_serial.write(0xC8 ^ (motor_pwms[pwm_send_index] + 16*pwm_send_index) ^ '\0' ^ 0xFF);

      // //Increment index for multiplexing
      // pwm_send_index++;
      // if (pwm_send_index == 4) {
      //   pwm_send_index = 0;
      // }
      last_write_millis = current_millis;
    }

    //1000ms since last NACK --> sensor reset
    if ((current_millis - last_nack_millis >= NACK_TIMEOUT) && (state == CONNECTED)) {
      last_nack_millis = current_millis;
      state = HANDSHAKE;
    }

  }
}

//Parse read byte by the motor control protocol and set a motor accordingly
void setMotor(uint8_t val) {
    uint8_t motor_index = 0;
    bool motor_turn_on = false;
    bool polarity_forward = false;
    unsigned short int duty_cycle = 0;

    //Get bits of motor index (right shift to get bits 6 and 7 and return the value)
    motor_index = (val >> 6);

    //Get PWM duty cycle (combine with mask to get just the first 4 bits)
    duty_cycle = (val & DUTY_CYCLE_MASK);

    //Get ON/OFF state 
    if ((val & ON_OFF_MASK) != 0) {
      motor_turn_on = true;
    } 
    else {
      motor_turn_on = false;
    }
    //Get polarity
    if ((val & POLARITY_MASK) != 0) {
      polarity_forward = true;
    } 
    else {
      polarity_forward = false;
    }
    

    //Set specified motor to specified parameters if turned on, otherwise turn off
    if (motor_turn_on && duty_cycle != 0) {
        motors[motor_index].Set_Speed(duty_cycle);
        //Save motor PWM for data sendback
        motor_pwms[motor_index] = duty_cycle;
          if (polarity_forward) {
            motors[motor_index].Run(FORWARD);
          } else {
            motors[motor_index].Run(BACKWARD);
          }
    } 
    else {
        motors[motor_index].Run(RELEASE);
    }   
}

// Read battery voltage value and calculate average value
void voltage_control() {
  voltage_readout = readADC_Cal(analogRead(ADC_PIN));
  
  // Fill out the window at the beginning to avoid 0s skewing the average
  if (avg_index == 0) {
    for (uint8_t i = 0; i < AVG_WINDOW_SIZE; i++) {
      avg_window[i] = voltage_readout;
    }
  }


  //Calculate running average
  avg_window[avg_index] = voltage_readout;
  avg_index = (avg_index + 1) % AVG_WINDOW_SIZE;

  uint32_t sum = 0;
  for (uint8_t i = 0; i < AVG_WINDOW_SIZE; i++) {
    sum += avg_window[i];
  }
  uint32_t average = sum / AVG_WINDOW_SIZE;
  // Trigger shutdown procedure if battery voltage gets below threshold
  if (average <= 860 && state != LOW_BATTERY) { //860 --> 10.8V
    state = LOW_BATTERY;
    low_voltage_trigger_millis = millis();
  }

  // Check if 10 seconds have passed since low voltage trigger, enter deep sleep
  if (state == LOW_BATTERY) {
    current_millis = millis();
    if (current_millis - low_voltage_trigger_millis >= LOW_BATTERY_TIMEOUT) {
      esp_deep_sleep_start();
    }
  }
}

// Control GPIO LED
void led_control() {
  led_current_millis = millis();
  // Slow flashing
  if (state == HANDSHAKE) {
    // Switch duty cycle after timer reset
    if (led_current_millis - led_previous_millis > LED_INTERVAL) {
      if (led_duty_cycle == 0) {
        led_duty_cycle = 255;      
      }
      else {
        led_duty_cycle = 0;
      }
      ledcWrite(LED_CHANNEL, led_duty_cycle);
      led_previous_millis  = led_current_millis;
    }
  }
  // Low PWM LED on if connected to EV3
  else if (state == CONNECTED) {
    ledcWrite(LED_CHANNEL, LED_DUTY_CYCLE_LOW);
  }
  // Fast flashing if battery is low
  else if (state == LOW_BATTERY) {
    // Switch out duty cycle after timer reset
    if (led_current_millis - led_previous_millis > LED_INTERVAL/5) {
        if (led_duty_cycle == 0) {
          led_duty_cycle = 255;      
        }
        else {
          led_duty_cycle = 0;
        }
        ledcWrite(LED_CHANNEL, led_duty_cycle);
      led_previous_millis = led_current_millis;
    }
  }
}


//Calibrate ADC output
uint32_t readADC_Cal(int ADC_Raw)
{
  esp_adc_cal_characteristics_t adc_chars;
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  return(esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
}