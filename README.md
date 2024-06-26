# LEGO Mindstorms EV3 Motor Driver

The following files are a part of my bachelor's thesis, in which I created custom motor driver PCBs with integrated microcontrollers, designed enclosures for these drivers, and created EV3-G blocks that allow the EV3 brick to communicate with the driver and carry out UART communication. 

---

### Driver_Arduino
This directory contains code for the [ev3dev](https://www.ev3dev.org/) OS and an Arduino Uno with an [Adafruit Motor Driver shield](https://learn.adafruit.com/adafruit-motor-shield/overview) and allows the EV3 with the ev3dev OS installed to control motors via I2C.

---

### Driver_V0.1
This directory contains the code for the first version of the custom driver in the `Driver_Code` directory, as well as PCB and 3D case files for manufacturing and printing. This PCB design is faulty and has some missing components, so I strongly advise to use the V0.2 driver.

---

### Driver_V0.2
Directory with source code and design files for the second iteration of the custom Driver, now more compact and able to run motors with higher performance, while being significantly more efficient in an idle state. 

---

### EV3-G_Blocks
Blocks for the *LEGO EV3 Lab Software* and their source code are situated in this directory. The `Thesis_Block.ev3b` allows the EV3 to communicate via the LEGO UART Sensor Protocol, built on the UART protocol, as well as control the motors connected to the motor driver via both I2C and the aforementioned UART-based protocols.

The `Dexter_Mod.ev3b` block is a modified version of the *Dexter Industries I2C Block*, available [here](https://github.com/DexterInd/EV3_Dexter_Industries_Sensors/blob/master/Dexter.ev3b). My modifications to this block fix the long-standing issue of the I2C communication functionality, where the maximum value of a sent byte is `0x7F`.

---

### Driver_Manual.pdf
This document contains all the necessary info to create the second version of the custom driver, get it up and running and use EV3-G block to control it.

---

### Future Work
The next iteration of the motor driver will take on a different approach, with the connection to the EV3 going wireless. Stay tuned! 



