This repo contains libraries for AVR ATmega 16 & similar micro-controllers.
It contains function names same as that used in Arduino IDE

io.h is the main library containing all the basic functions for ATmega16 microcontroller
analogWrite1() for Atmega 2560:
1.PARAMETERS:
The parameters for this function are the pin and duty cycle.The pins for this function are PB5 and PB6 on Atmega and digital pin 11 and 12 on Arduino.For simplicity,pin parameter has only two values.These are 1 and 2.If 1 is given as parameter for pin,the duty cycle will be given to PB5/digital pin 11 and if it is 2 the duty cycle will be given toPB6/digital pin 12.
2.SYNTAX:
analogWrite1(pin,duty cycle);


analogWrite() for Atmega 16:
1.PARAMETERS:
The parameters for this function are the pin and duty cycle.The pins for this function are PD4 and PD5 on Atmega .For simplicity,pin parameter has only two values.These are 1 and 2.If 1 is given as parameter for pin,the duty cycle will be given to PD4 and if it is 2 the duty cycle will be given to PD5.
2.SYNTAX:
analogWrite(pin,duty cycle);

