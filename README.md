HomeSeer_Arduino_Plugin
=======================

This Plugin was written to interact with the Arduino range of I/O boards and Homeseer. The Plugin has the following functions: 

1. Boards can be connected with USB, Serial or Ethernet.  
2. Board pins can be assigned as Input, Analogue input, Output,  PWM, Servo or Onewire. 
3. The Plugin will auto generate the used devices in homeseer. 
4. All Inputs are switched Inputs that are on when pulled to ground.(no resistors are required) 
5. All outputs are High when On in Homeseer. 
6. All inputs and Outputs are updated on connection to homeseer. 
7. Up to 9 Arduino boards are supported. 
8. One Wire DS18S20, DS18B20 and DS1822 temperature sensors are supported. 
9. PWM fade times can be set from a device in Homeseer 
10. Calculations can be performed on the analogue inputs to display custom data. 
11. Analogue inputs can be inverted to accommodate various sensors. 
12. The board will Auto reconnect if reset while connected to HS. 
13. Servos can be set to a value for 0 to 180° from a HS device. 
14. An RGB device can be created from 3 pwm outputs 
15. There is an API for the board so you can send your own values from the board to Homeseer. 
