#include <Arduino.h>
#include "drv8833_motor.h"
#include "ls7184_encoder.h"
#include "pid_controller.h"
#include "motor_controller.h"
#include "robot.h"
#include "test.h"

unsigned long prev_ros_millis;
unsigned long prev_control_millis;
unsigned long prev_serial_millis;
unsigned long curr_millis;
const unsigned long ros_period = 40; // ROS communication period [millis]
const unsigned long control_period = 5; // control loop period [millis]
const unsigned long serial_period = 100; // period for printing to terminal for debugging [millis]

const float wheelbase = 0.14; // [m]
const float wheel_radius = 0.065 / 2; // [m]
const float no_load_rps = 0.4 / wheel_radius; // determined experimentally in [m/s] --> [rad/s]

// Initialize encoder parameter struct
// {byte clk_pin (byte), byte dir_pin, float counts_per_rev, float sample_period, float filt_tc}
Farmaid::EncoderParams left_encoder_p = {2, 8, 1920.0, control_period / 1000.0, 4.0 * control_period / 1000.0};
Farmaid::EncoderParams right_encoder_p = {3, 9, 1920.0, control_period / 1000.0, 4.0 * control_period / 1000.0};

// Initialize motor parameter struct
// {byte pwm_pin, byte dir_pin, unsigned int max_command, float no_load_rps}
Farmaid::MotorParams left_motor_p = {6, 7, 255, no_load_rps};
Farmaid::MotorParams right_motor_p = {5, 4, 255, no_load_rps};

// Initialize PID controller parameter struct
// {float p_gain, float i_gain, float d_gain, float filt_const, float sample_period}
Farmaid::PidParams left_pid_p = {1.0, 0.0, 0.0, 0.0, control_period / 1000.0};
Farmaid::PidParams right_pid_p = {1.0, 0.0, 0.0, 0.0, control_period / 1000.0};

// Initialize encoders, motors, and PID controllers
Farmaid::Encoder left_encoder = Farmaid::Encoder(left_encoder_p, &left_encoder_count);
Farmaid::Encoder right_encoder = Farmaid::Encoder(right_encoder_p, &right_encoder_count);
Farmaid::Motor left_motor = Farmaid::Motor(left_motor_p);
Farmaid::Motor right_motor = Farmaid::Motor(right_motor_p);
Farmaid::PidController left_pid = Farmaid::PidController(left_pid_p);
Farmaid::PidController right_pid = Farmaid::PidController(right_pid_p);

// Initialize motor controller objects for debugging motors individually
Farmaid::MotorController left_motor_controller = Farmaid::MotorController(left_encoder, left_motor, left_pid);
Farmaid::MotorController right_motor_controller = Farmaid::MotorController(right_encoder, right_motor, right_pid);

// Initialize robot
Farmaid::Robot robot = Farmaid::Robot(left_encoder, right_encoder,
                                      left_motor, right_motor,
                                      left_pid, right_pid,
                                      wheelbase, wheel_radius);

// TODO: Initialize ROS node handle, publishers, and subscribers

void setup() {

    // Attach encoder interrupts
    attachInterrupt(0, LeftEncoderInterrupt, RISING);
    attachInterrupt(1, RightEncoderInterrupt, RISING);

    Serial.begin(9600);       // initialize Serial Communication

    // Necessary for encoder interrupts to initialize
    delay(200);
}

void loop() {

    curr_millis = millis();

    // Test individual components
//    ReadEncoders(); // Passed!
//    Farmaid::TestEncoderClass(); // Passed!
//    Farmaid::TestMotorOpenLoop(left_motor, 1); // Passed!
//    Farmaid::TestMotorOpenLoop(left_motor, -1); // Passed!
//    Farmaid::TestMotorOpenLoop(right_motor, 1); // Passed!
//    Farmaid::TestMotorOpenLoop(right_motor, -1); // Passed!

    // Find deadband
//    left_motor.set_command(0.28);
//    left_motor.set_command(-0.11);
//    right_motor.set_command(0.35);
//    right_motor.set_command(-0.09);

    // Motor position control
    
//    left_pid.set_gains(7.5, 0.0, 0.25, 10.0 * control_period / 1000.0);
//    left_pid.set_max_deadband(0.15);
//    left_pid.set_min_deadband(-0.1);
//    Farmaid::TestMotorPositionControl(left_motor, left_encoder, left_pid, 1); // Passed!
//    Farmaid::TestMotorPositionControl(left_motor, left_encoder, left_pid, -1); // Passed!

//    right_pid.set_gains(7.0, 0.0, 0.25, 10.0 * control_period / 1000.0);
//    right_pid.set_max_deadband(0.18);
//    right_pid.set_min_deadband(-0.1);
//    Farmaid::TestMotorPositionControl(right_motor, right_encoder, right_pid, 1); // Passed!
//    Farmaid::TestMotorPositionControl(right_motor, right_encoder, right_pid, -1); // Passed!

    // Find maximum speed of each motor to add feedforward to PID controller
//    Farmaid::TestMaxSpeed(left_motor, left_encoder, 1); // 13.09 rad/s * 0.065/2 m = 0.425 m/s
//    Farmaid::TestMaxSpeed(left_motor, left_encoder, -1); // 12.44 rad/s * 0.065/2 m = 0.404 m/s
//    Farmaid::TestMaxSpeed(right_motor, right_encoder, 1); // 13.09 rad/s * 0.065/2 m = 0.425 m/s
//    Farmaid::TestMaxSpeed(right_motor, right_encoder, -1); // 13.09 rad/s * 0.065/2 m = 0.425 m/s

    // Motor velocity control

    // Step velocity commands with PID only
//    left_pid.set_gains(12.0, 3.0, 0.1, 10.0 * control_period / 1000.0);
//    left_pid.set_antiwindup(1.2); // Antiwindup makes it worse. Need to debug this.
//    Farmaid::TestMotorVelocityStep(left_motor, left_encoder, left_pid, 1); // Passed!
//    left_pid.set_gains(11.0, 2.5, 0.0, 10.0 * control_period / 1000.0);
//    Farmaid::TestMotorVelocityStep(left_motor, left_encoder, left_pid, -1); // Passed!

//    right_pid.set_gains(12.0, 3.0, 0.1, 10.0 * control_period / 1000.0);
//    Farmaid::TestMotorVelocityStep(right_motor, right_encoder, right_pid, 1); // Passed!
//    right_pid.set_gains(11.0, 2.0, 0.0, 10.0 * control_period / 1000.0);
//    Farmaid::TestMotorVelocityStep(right_motor, right_encoder, right_pid, -1); // Passed!

    // Step velocity commands with PID and feedforward
//    left_pid.set_gains(1.0, 0.0, 0.0, 10.0 * control_period / 1000.0);
//    left_pid.set_feedforward_flag(true);
//    Farmaid::TestMotorVelocityStep(left_motor, left_encoder, left_pid, 1); // Passed!
//    Farmaid::TestMotorVelocityStep(left_motor, left_encoder, left_pid, -1); // Passed!

//    right_pid.set_gains(2.0, 0.0, 0.0, 10.0 * control_period / 1000.0);
//    right_pid.set_feedforward_flag(true);
//    Farmaid::TestMotorVelocityStep(right_motor, right_encoder, right_pid, 1); // Passed!
//    Farmaid::TestMotorVelocityStep(right_motor, right_encoder, right_pid, -1); // Passed!

    // Sinusoidal velocity command with PID and feedforward
//    left_pid.set_gains(3.0, 0.0, 0.0, 10.0 * control_period / 1000.0);
//    left_pid.set_feedforward_flag(true);
//    Farmaid::TestMotorVelocitySine(left_motor, left_encoder, left_pid, 1); // Passed!
//    Farmaid::TestMotorVelocitySine(left_motor, left_encoder, left_pid, -1); // Passed!

//    right_pid.set_gains(3.0, 0.0, 0.0, 10.0 * control_period / 1000.0);
//    right_pid.set_feedforward_flag(true);
//    Farmaid::TestMotorVelocitySine(right_motor, right_encoder, right_pid, 1); // Passed!
//    Farmaid::TestMotorVelocitySine(right_motor, right_encoder, right_pid, -1); // Passed!

    // Test robot drive method (PID gains and feedforward_flag set in Robot constructor)
//    Farmaid::TestRobotForward(robot); // Passed!
//    Farmaid::TestRobotRotate(robot);
    Farmaid::TestRobotCircle(robot, 0.3, -1);

//    UpdateRos(); // publish and subscribe to ROS topics
//    robot.Drive(vel, ang_vel); // drive the robot

}

void UpdateRos()
{
    // Publish and subscribe to ROS topics
    if (curr_millis - prev_ros_millis >= ros_period)
    {
        // TODO
        // nh.spinOnce();
        prev_ros_millis = curr_millis;
    }
}
