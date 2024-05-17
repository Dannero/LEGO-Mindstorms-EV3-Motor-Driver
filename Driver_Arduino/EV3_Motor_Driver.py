#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.iodevices import I2CDevice
from pybricks.parameters import Port
import time

bus = I2CDevice(Port.S2, 0x12)  
slave_address = 0x12  

class MotorDriver:
    def __init__(self, bus, slave_address):
        self.bus = bus
        self.slave_address = slave_address

    #Function to set a motor's state(motor's index, on/off state, polarity, duty cycle)
    def setMotor(set_motor: int, set_on: bool, set_polarity: int, set_duty_cycle: int):
        # Incorrect function call handle
        if (set_motor < 0 or set_motor > 3):
            raise ValueError("Can only set motors 0 to 3")
        if (set_polarity != 0 and set_polarity != 1):
            raise ValueError("Polarity can only be set to 0 (Backward) or 1 (Forward)")
        if (set_duty_cycle < 0 or set_duty_cycle > 15):
            raise ValueError("Can only set motor duty cycle to values between 0 and 15")    

        send_byte = 0
        send_byte += set_motor << 6
        if (set_on):
            send_byte += 1 << 5
        send_byte += set_polarity << 4
        send_byte += set_duty_cycle

        #Send byte
        bus.write(slave_address, bytes([send_byte]))


    #Set all motors on to a single duty cycle and polarity
    def runAllMotors(set_polarity: int, set_duty_cycle: int):
        if (set_polarity != 0 and set_polarity != 1):
            raise Exception("Polarity can only be set to 0 (Backward) or 1 (Forward)")
        if (set_duty_cycle < 0 or set_duty_cycle > 15):
            raise Exception("Can only set motor duty cycle to values between 0 and 15")   
        
        #Motor Addresses (bits 6 and 7) + Motor turn on bit (bit 5)
        motor_bytes = [
            0 + (1 << 5),
            64 + (1 << 5),
            128 + (1 << 5),
            192 + (1 << 5)
        ]
        #Set polarity and duty cycle for motors
        for byte in motor_bytes:
            byte += set_polarity << 4
            byte += set_duty_cycle
            #Send byte
            bus.write(slave_address, bytes([byte]))

    #Stop all motors at once
    def stopAllMotors():
        bus.write(slave_address, bytes([0x00]))
        bus.write(slave_address, bytes([0x40]))
        bus.write(slave_address, bytes([0x80]))
        bus.write(slave_address, bytes([0xC0]))



MotorDriver.setMotor(0, 1, 1, 5)
time.sleep(0.3)
MotorDriver.setMotor(1, 0, 1, 7)
time.sleep(0.3)
MotorDriver.setMotor(2, 1, 1, 11)
time.sleep(0.3)
MotorDriver.setMotor(3, 0, 1, 15)

time.sleep(3)
MotorDriver.stopAllMotors()
time.sleep(2)
MotorDriver.runAllMotors(1, 15)
time.sleep(3)
MotorDriver.stopAllMotors()

