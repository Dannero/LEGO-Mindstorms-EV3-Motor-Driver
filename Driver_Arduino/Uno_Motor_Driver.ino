#include <Wire.h>
#include <AFMotor.h>
#define SLAVE_ADDRESS 0x12


//Set up all four motors
    AF_DCMotor motor1(1);
    AF_DCMotor motor2(2);
    AF_DCMotor motor3(3);
    AF_DCMotor motor4(4);
//Array of motors (easier motor setting by just setting the motor at a parsed index)
AF_DCMotor motors[4] = {motor1, motor2, motor3, motor4};

//Mask for motor on/off bit (bit 5)
uint8_t on_off_mask = 0x20;
//Mask for motor polarity bit (bit 4)
uint8_t polarity_mask = 0x10;
//Mask for motor duty cycle (bits 0-3)
uint8_t duty_cycle_mask = 0x0F;

uint8_t val, flag=0;

void setup()
{
  //Listen to I2C communication on the defined slave device address
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
}

void loop()
{
  if(flag==1)
   {
    setMotor(val);
    flag=0;
   }
}

void receiveData(int byteCount)
{
  //Incoming message byte
  while(Wire.available()>0)
  {
    val = Wire.read();
    flag=1;
  }
}

//Parse read byte by the motor control protocol and set a motor accordingly
void setMotor(unsigned short int val) {
    unsigned short int motor_index = 0;
    bool motor_turn_on = false;
    bool polarity_forward = false;
    unsigned short int duty_cycle = 0;
    //Get bits of motor index (right shift to get bits 6 and 7 and return the value)
    motor_index = (val >> 6);
    //Get ON/OFF state 
    if ((val & on_off_mask) != 0) {
      motor_turn_on = true;
    } else {
      motor_turn_on = false;
    }
    //Get polarity
    if ((val & polarity_mask) != 0) {
      polarity_forward = true;
    } else {
      polarity_forward = false;
    }

    //Get PWM duty cycle (combine with mask to get just the first 4 bits)
    duty_cycle = (val & duty_cycle_mask);

    //Set specified motor to specified parameters if turned on, otherwise turn off
    if (motor_turn_on && duty_cycle != 0) {
        motors[motor_index].setSpeed((duty_cycle*255)/15);
          if (polarity_forward) {
            motors[motor_index].run(FORWARD);
          } else {
            motors[motor_index].run(BACKWARD);
          }
    } else {
        motors[motor_index].run(RELEASE);
    }   
}