Arduino Sketchook
===============

This repository contains my Arduino sketches for various projects.

readEncoder.ino
------------------------------

One DC motor encoder and one LS7184 quadrature encoders are set up to measure the encoder counts and direction of the DC motor. The measurements are sent to the Serial port to be displayed.

>Note: If you are using the USB for serial communication, you cannot use digital pins 0 (RX) and 1 (TX) for digital input or output. [Ref.](https://www.arduino.cc/reference/en/language/functions/communication/serial/)

readTwoEncoders.ino
------------------------------

Same as readEncoder.ino, but using two of everything.

motor_driver.ino
-----------------------

These instructions are based on the tutorial [here](https://hackernoon.com/apply-coursera-control-of-mobile-robots-with-ros-and-rosbots-part-1-777a51f63617). First upload this sketch onto the Arduino.

Next start the **ROS Master** by opening a new terminal and running `roscore`. Next start the **rosserial client** application that forward your Arduino messages to the rest of **ROS** by opening a new terminal and running

```bash
rosrun rosserial_python serial_node.py /dev/ttyACM0
```
If necessary, replace `ttyACM0` with the correct serial port information by going to **Tools | Serial Port** from the Arduino IDE drop-down menu.
 >Note: There was an error when trying to run python scripts because of how the `PYTHONPATH` was configured. It turns out that because I had installed **Anaconda** after ROS, the Anaconda installation changed this configuration so ROS did not know where to locate some files. To get ROS working again, go into `.bashrc` and comment out the line `export PATH=$HOME/anaconda3/bin:$PATH`. When you go back to using Anaconda, just uncomment this line.

If you run `rosnode list` in the terminal, the output should be
```
/rosout
/uno_serial_node
```
If you run `rostopic list` in the terminal, the output should be
```
/diagnostics
/rosout
/rosout_agg
/wheel_power_left
/wheel_power_right
```
 
 

