# Arduino Sketchbook

---

This repository contains my Arduino sketches for various projects.

## readEncoder.ino

---

One DC motor encoder and one LS7184 quadrature encoders are set up to measure the encoder counts and direction of the DC motor. The measurements are sent to the Serial port to be displayed.

>Note: If you are using the USB for serial communication, you cannot use digital pins 0 (RX) and 1 (TX) for digital input or output. [Ref.](https://www.arduino.cc/reference/en/language/functions/communication/serial/)

## readTwoEncoders.ino

---

Same as readEncoder.ino, but using two of everything.

## motor_driver.ino

---

These instructions are based on the tutorial [here](https://hackernoon.com/apply-coursera-control-of-mobile-robots-with-ros-and-rosbots-part-1-777a51f63617). First upload this sketch onto the Arduino.

Next start the **ROS Master** by opening a new terminal and running `roscore`. Next start the **rosserial client** application that forwards your Arduino messages to the rest of **ROS** by opening a new terminal and running

```
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

 To make the left wheel spin forward at maximum speed, enter the command
```bash
rostopic pub -1 /wheel_power_left std_msgs/Float32 '{data: 1.0}'
```
Replacing `{data: 1.0}` with `{data: 0.0}` or `{data: -1.0}` should make the left wheel stop spinning or spin in the opposite direction at maximum speed, respectively. This message can take on values between -1.0 and 1.0 to map to a desired wheel power. The analogous commands can be sent to the right wheel by replacing `/wheel_power_left` with `/wheel_power_right`. Giving rosnode list a command in this case be understood as publishing a message of type `std_msgs/Float32` to a ROS topic (`/wheel_power_left` or `/wheel_power_right`), which is subscribed to by the rosnode called `/uno_serial_node`.

## readEncoder_ros.ino

---

This sketch combines that of **motor_driver.ino** and **readTwoEncoders.ino**. In this sketch, I created two publisher objects that publish to ROS topics called `/left_wheel_velocity` and `/right_wheel_velocity`. I also created two subscriber objects that subscribe to ROS topics called `/wheel_power_left` and `/wheel_power_right`. I can command a power to be delivered to each wheel by publishing a message on the terminal to the `/wheel_power_left` and `/wheel_power_right` topics, as explained in the **motor_driver.ino** section. Furthermore, I can read the left (or right) wheel velocity by going into a new terminal and running the command
```
rostopic echo /left_wheel_velocity
```
A tool for visualizing the active ROS nodes and topics can be run by entering the command
```
rqt_graph
```
To see the topics show up in `rqt_graph`, you will have to run `rostopic echo <some-ros-topic>`.

A useful tool for visualizing the wheel velocity measurements is a plotting GUI that can be run by entering the command
```
rqt_plot
```
### Mininum and maximum motor speed at no load

**Left motor:**

Minimum positive speed: 0.19/1.0 power measures 2.5 rad/s

Maximum positive speed: 1.0/1.0 power measures 26 rad/s

Minimum negative speed: -0.08/-1.0 power measures -1.6 rad/s

Maximum negative speed: -1.0/-1.0 power measures -25.3 rad/s

## farmaid_ros.ino

---

This sketch instantiates a Robot object containing specific attributes and methods that characterize its different components and behaviors, respectively. The purpose of this program is to receive high level linear and angular velocity commands from a supervisory controller in ROS and output low level control commands to the motors to allow the robot to achieve the desired linear and angular velocity. Another function of this program is to calculate the speed and odometry of the robot based on encoder measurements and publish them to ROS. This code is organized into various classes including: Robot, Motor, Encoder, and PidController.

The Robot class constructor takes as arguments two Motor objects, two Encoder objects, and two PidController objects. I instantiate these objects in the global workspace in order to test their functionality in isolation before they operate as one unit in the Robot object. Initially, I passed these objects to the Robot class constructor when I instantiated the Robot object. The Robot constructor then creates a copy of these objects and they become member attributes of the robot object. As a result, the global workspace still contains instances of the motor, encoder, and PID controller objects while the robot object also has copies of these objects as member attributes. If I plan to only use the robot object in the main program and don't need to test the motors, encoders, and PID controllers separately, then I should instead instantiate the objects directly inside the argument list when I call the Robot class constructor. Instantiating the robot object in this way takes up less memory because the motor, encoder, and PID controller objects only exist inside of the robot object. They do not exist in the global workspace.

### Controlling robot via ROS Teleop mode

Upload the farmaid.ino code into the Arduino.

Open a new terminal and start `roscore`.

Open another terminal and run the **rosserial client** application:

```
$ rosrun rosserial_python serial_node.py /dev/ttyACM0
```

Running this enables communication between the ROS serial node running on the laptop and the Arduino. If necessary, replace the name of the USB port `/dev/ttyACM0` with the correct one to which the Arduino is connected.

> Note: The `serial_node.py` executable will not run if your terminal is in an **environment** that conflicts with the ROS workspace. In my case, the ROS terminal was in an Anaconda environment which caused an error error when running the rosserial client. To fix this, you have to deactivate the Anaconda environment in the terminal by running: `conda deactivate`.

Next run the `teleop_twist_keyboard` ROS node to publish velocity commands to the `/cmd_vel` topic using the keyboard as the controller:
```
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
Pressing the appropriate keys will send velocity and angular velocity commands to the robot, which should cause it to move accordingly. To see the commanded velocity being published to the robot, run the command:
```
$ rostopic echo /cmd_vel
```