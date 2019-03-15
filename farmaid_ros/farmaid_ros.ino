#include <Arduino.h>
#include "differential_drive.h"
#include "drv8833_motor.h"
#include "encoder.h"
#include "pid_controller.h"

// Sampling rates
const float kControlFreq = 100; // Hz
const float kControlSampleTime = 1.0 / kControlFreq; // sec
const float kControlSampleTimeMicros = kControlSampleTime * 1e6; // microsec
unsigned long prev_control_time_micros = 0;

const float kRosFreq = 50; // Hz
const float kRosSampleTime = 1.0 / kRosFreq; // sec
const float kRosSampleTimeMs = kRosSampleTime * 1e6; // microsec
unsigned long prev_ros_time_micros = 0;

// Robot parameters
const float kWheelbase = 0.5; // meters
const float kWheelRadius = 0.1; // meters

// Motor hardware interface parameters
const int kLeftMotorPwmPin = 6;
const int kLeftMotorDirPin = 7;
const int kRightMotorPwmPin = 5;
const int kRightMotorDirPin = 4;
const float kMaxMotorSpeed = 20; // use the minimum of the two max motor speeds [rad/sec]

// Motor PID controller parameters
float p_gain = 0; // proportional gain
float i_gain = 0; // integral gain
float d_gain = 0; // derivative gain
float filt_const = 0; // first order low-pass filter constant; 0: pass-through, 0-1: low-pass, 1: one time-step delay

namespace Farmaid
{
    class Robot
    {
    public:
        Robot()
        : left_motor_(kLeftMotorPwmPin, kLeftMotorDirPin), right_motor_(kRightMotorPwmPin, kRightMotorDirPin),
          left_encoder_(kLeftEncClkPin, kLeftEncDirPin), right_encoder_(kRightEncClkPin, kRightEncDirPin),
          diff_drive_(kWheelbase, kWheelRadius, kMaxMotorSpeed)
        {
            left_motor_pid = PidController(p_gain, i_gain, d_gain, filt_const, kControlSampleTime);
            right_motor_pid = PidController(p_gain, i_gain, d_gain, filt_const, kControlSampleTime);
        }

    void Execute(float des_vel, float des_ang_vel)
    {

        // Update the encoder member variables with current encoder measurements
        left_encoder_.UpdateCount();
        right_encoder_.UpdateCount();

        // Calculate the current wheel velocities
        curr_left_wheel_vel = diff_drive_.ConvertAngVel2WheelVel(left_encoder_.get_ang_vel());
        curr_right_wheel_vel = diff_drive_.ConvertAngVel2WheelVel(right_encoder_.get_ang_vel());

        // Calculate the desired wheel velocities
        diff_drive_.UpdateDesWheelVel(des_vel, des_ang_vel);
        des_left_wheel_vel = diff_drive_.get_des_left_wheel_vel();
        des_right_wheel_vel = diff_drive_.get_des_right_wheel_vel();

        // Compute controller commands based on desired and current wheel velocities
        left_motor_command = left_motor_pid_.ComputeCommand(des_left_wheel_vel, curr_left_wheel_vel);
        right_motor_command = right_motor_pid_.ComputeCommand(des_right_wheel_vel, curr_right_wheel_vel);

        // Send controller commands to each motor
        left_motor_.set_command(left_motor_command);
        right_motor_.set_command(right_motor_command);
    }
        
    private:
        Motor left_motor_;
        Motor right_motor_;
        
        PidController left_motor_pid_;
        PidController right_motor_pid_;
        
        Encoder left_encoder_;
        Encoder right_encoder_;

        DifferentialDrive diff_drive_;
    };
};

void setup() {
  // put your setup code here, to run once:

  Farmaid::Robot robot; // instantiate robot

  unsigned long curr_time_micros = micros();

  // Attach encoder interrupts
  attachInterrupt(0, encoderIntL, RISING);
  attachInterrupt(1, encoderIntR, RISING);
  
  // Necessary for encoder interrupts to initialize
  delay(200);
}

void loop() {
  // put your main code here, to run repeatedly:

  curr_time_micros = micros();

  // Publish and subscribe to/from ROS master at this rate
  if (curr_time_micros - prev_ros_time_micros) >= kRosSampleTimeMicros:
  {
      nh.spinOnce();
      prev_ros_time_micros = curr_time_micros;
  }

  // Execute control loop at this rate
  if (curr_time_micros - prev_control_time_micros) >= kControlSampleTimeMicros:
  {
      // Get the desired velocity and angular velocity from the subscribed topic
      des_vel = getDesVel(); // TODO
      des_ang_vel = getDesAngVel(); // TODO
      
      robot.Execute(des_vel, des_ang_vel));
      prev_control_time_micros = curr_time_micros;
  }

}
