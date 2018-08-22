#ifndef _READ_ENCODER_ROS_
#define _READ_ENCODER_ROS_

#include <ros.h>
#include "Arduino.h"
#include <std_msgs/Float32.h>
#include <TimerOne.h>

ros::NodeHandle nh;

std_msgs::Float32 left_wheel_vel;
ros::Publisher left_wheel_vel_pub("/left_wheel_velocity", &left_wheel_vel);

std_msgs::Float32 right_wheel_vel;
ros::Publisher right_wheel_vel_pub("/right_wheel_velocity", &right_wheel_vel);

// left encoder variables
const int clkPinL = 2;
const int dirPinL = 8;
volatile int encoderCountL = 0;
volatile boolean changeFlagL = false;
volatile int countResetL = 0;

// right encoder variables
const int clkPinR = 3;
const int dirPinR = 9;
volatile int encoderCountR = 0;
volatile boolean changeFlagR = false;
volatile int countResetR = 0;

// other variables
const unsigned long sampleTime_us = 200000; // 200000us = 0.2s = 5Hz
const float sampleRate_hz = 1.0/(sampleTime_us/1e6);
const float pi = 3.1415;
const int ppr = 1920; // encoder pulses per revolution of motor

void timerIsr(void) {
  Timer1.detachInterrupt(); // stop the timer
  
  //Left motor speed [rad/sec] = (pulses/sample) * (rev/pulses) * (2pi rad/rev) * (samples/sec)
  left_wheel_vel.data = float(countResetL)*(1.0/ppr)*2.0*pi*sampleRate_hz;
  left_wheel_vel_pub.publish(&left_wheel_vel);
  
  //Right motor speed
  right_wheel_vel.data = float(countResetR)*(1.0/ppr)*2.0*pi*sampleRate_hz;
  right_wheel_vel_pub.publish(&right_wheel_vel);
  
  countResetL = 0;
  countResetR = 0;
  
  Timer1.attachInterrupt(timerIsr); // enable the timer
}

void encoderIntL() {
  if (digitalRead(dirPinL) == HIGH) {
    encoderCountL++;
    countResetL++;
  }
  else {
    encoderCountL--;
    countResetL--;
  }
  changeFlagL = true;
}

void encoderIntR() {
  if (digitalRead(dirPinR) == HIGH) {
    encoderCountR++;
    countResetR++;
  }
  else {
    encoderCountR--;
    countResetR--;
  }
  changeFlagR = true;
}

void setup() {
//  Serial.begin(9600);
  pinMode(clkPinL, INPUT);  
  pinMode(dirPinL, INPUT);
  attachInterrupt(0, encoderIntL, RISING);
  
  pinMode(clkPinR, INPUT);  
  pinMode(dirPinR, INPUT);
  attachInterrupt(1, encoderIntR, RISING);
  
  Timer1.initialize(sampleTime_us); //initialize timer1 with period sampleTime_us
  Timer1.attachInterrupt(timerIsr); //attaches timerIsr function every period
  
  nh.initNode();
  nh.advertise(left_wheel_vel_pub);
  nh.advertise(right_wheel_vel_pub);
  
}

void loop() {
//  if (changeFlagL) {
//    changeFlagL = false;
//    Serial.print("Left encoder count = ");
//    Serial.println(encoderCountL);
//  }
//  
//  if (changeFlagR) {
//    changeFlagR = false;
//    Serial.print("Right encoder count = ");
//    Serial.println(encoderCountR);
//  }
//  
//  Serial.print("Left encoder velocity (m/s) = ");
//  Serial.println(left_wheel_vel.data);
//  
//  Serial.print("Right encoder velocity (m/s) = ");
//  Serial.println(right_wheel_vel.data);
//  
//  Serial.print("Sample rate (Hz) = ");
//  Serial.println(sampleRate_hz);
//  
//  delay(50);
  
  nh.spinOnce();
  
  // Debug
//  const unsigned long sampleTime_us = 200000; // 200000us = 0.2s = 5Hz
//  const float sampleRate_hz = 1.0/sampleTime_us/1e6;
//  const float pi = 3.1415;
//  const int ppr = 1920; // encoder pulses per revolution of motor
  
//  Serial.print("sampleTime_us = ");
//  Serial.println(sampleTime_us);
//  
//  Serial.print("Sample rate (Hz) = ");
//  Serial.println(sampleRate_hz);
//  delay(1000);
  
}

#endif



