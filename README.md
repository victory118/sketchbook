Arduino Sketchook
===============

This repository contains my Arduino sketches for various projects.

readTwoEncoders.ino
------------------------------

Two DC motor  encoders and two LS7184 quadrature encoders are set up to measure the encoder counts and direction of each DC motor. The measurements are sent to the Serial port to be displayed.

>Note: If you are using the USB for serial communication, you cannot use digital pins 0 (RX) and 1 (TX) for digital input or output. [Ref](https://www.arduino.cc/reference/en/language/functions/communication/serial/)

sfd