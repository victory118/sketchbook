// Choose Arduino board and robot configuration (comment out the other one)
//#define UNO_2WD // initialize parameters for 2 wheel drive robot on Arduino Uno
#define MEGA_4WD // initialize parameters for 4 wheel drive robot on Arduino Mega

// Choose testing mode or ROS teleop mode (comment out the other one)
//#define TESTING // initialize motor, encoder, and pid controller objects individually for testing
#define ROS_TELEOP // initialize robot object only when connected to ROS for teleop

//#include <Arduino.h>

unsigned long prev_control_millis;
unsigned long prev_serial_millis;
unsigned long curr_millis;
const unsigned long control_period = 5; // control loop period [millis]
const unsigned long serial_period = 100; // period for printing to terminal for debugging [millis]

#ifdef ROS_TELEOP

#include "ros_setup.h"

#endif

#ifdef UNO_2WD

#include "uno_2wd_setup.h" // initialize motor, encoder, and PID parameters and encoder interrupt callback functions

#ifdef TESTING

#include "test.h"

// Initialize encoders, motors, and PID controllers for testing only
// Otherwise it occupies too much RAM to have these objects defined and the robot owning another copy
Farmaid::Encoder left_encoder = Farmaid::Encoder(left_encoder_p, &left_encoder_count);
Farmaid::Encoder right_encoder = Farmaid::Encoder(right_encoder_p, &right_encoder_count);

Farmaid::Motor left_motor = Farmaid::Motor(left_motor_p);
Farmaid::Motor right_motor = Farmaid::Motor(right_motor_p);

Farmaid::PidController left_pid = Farmaid::PidController(left_pid_p);
Farmaid::PidController right_pid = Farmaid::PidController(right_pid_p);

// Initialize robot
// Use this robot initialization for testing 
Farmaid::TwoWheelDrive robot = Farmaid::TwoWheelDrive(left_encoder, right_encoder,
                                      left_motor, right_motor,
                                      left_pid, right_pid,
                                      wheelbase, wheel_radius);

#else

// Use this robot initialization when running teleop mode with ROS
Farmaid::TwoWheelDrive robot = Farmaid::TwoWheelDrive(Farmaid::Encoder(left_encoder_p, &left_encoder_count), Farmaid::Encoder(right_encoder_p, &right_encoder_count),
                                      Farmaid::Motor(left_motor_p), Farmaid::Motor(right_motor_p),
                                      Farmaid::PidController(left_pid_p), Farmaid::PidController(right_pid_p),
                                      wheelbase, wheel_radius);

#endif
#endif

#ifdef MEGA_4WD

#include "mega_4wd_setup.h" // initialize motor, encoder, and PID parameters and encoder interrupt callback functions

#ifdef TESTING

#include "test.h"

// Initialize encoders, motors, and PID controllers for testing only
// Otherwise it occupies too much RAM to have these objects defined and the robot owning another copy
Farmaid::Encoder fleft_encoder = Farmaid::Encoder(fleft_encoder_p, &fleft_encoder_count);
Farmaid::Encoder fright_encoder = Farmaid::Encoder(fright_encoder_p, &fright_encoder_count);
Farmaid::Encoder rleft_encoder = Farmaid::Encoder(rleft_encoder_p, &rleft_encoder_count);
Farmaid::Encoder rright_encoder = Farmaid::Encoder(rright_encoder_p, &rright_encoder_count);

Farmaid::Motor fleft_motor = Farmaid::Motor(fleft_motor_p);
Farmaid::Motor fright_motor = Farmaid::Motor(fright_motor_p);
Farmaid::Motor rleft_motor = Farmaid::Motor(rleft_motor_p);
Farmaid::Motor rright_motor = Farmaid::Motor(rright_motor_p);

Farmaid::PidController fleft_pid = Farmaid::PidController(fleft_pid_p);
Farmaid::PidController fright_pid = Farmaid::PidController(fright_pid_p);
Farmaid::PidController rleft_pid = Farmaid::PidController(rleft_pid_p);
Farmaid::PidController rright_pid = Farmaid::PidController(rright_pid_p);

// Initialize robot
// Use this robot initialization for testing 
Farmaid::FourWheelDrive robot = Farmaid::FourWheelDrive(fleft_encoder, fright_encoder,
                                                        rleft_encoder, rright_encoder,
                                                        fleft_motor, fright_motor,
                                                        rleft_motor, rright_motor,
                                                        fleft_pid, fright_pid,
                                                        rleft_pid, rright_pid,
                                                        wheelbase, wheel_radius);

#else

// Use this robot initialization when running teleop mode with ROS
Farmaid::FourWheelDrive robot = Farmaid::FourWheelDrive(Farmaid::Encoder(fleft_encoder_p, &fleft_encoder_count), Farmaid::Encoder(fright_encoder_p, &fright_encoder_count),
                                                        Farmaid::Encoder(rleft_encoder_p, &rleft_encoder_count), Farmaid::Encoder(rright_encoder_p, &rright_encoder_count),
                                                        Farmaid::Motor(fleft_motor_p), Farmaid::Motor(fright_motor_p),
                                                        Farmaid::Motor(rleft_motor_p), Farmaid::Motor(rright_motor_p),
                                                        Farmaid::PidController(fleft_pid_p), Farmaid::PidController(fright_pid_p),
                                                        Farmaid::PidController(rleft_pid_p), Farmaid::PidController(rright_pid_p),
                                                        wheelbase, wheel_radius);

#endif
#endif

void setup() {

    curr_millis = millis();

#ifdef ROS_TELEOP
    // Initialize ROS node
    nh.initNode();
    nh.subscribe(sub);
    prev_ros_millis = curr_millis;
#endif

#ifdef TESTING
    prev_serial_millis = curr_millis;
    Serial.begin(9600);       // initialize Serial Communication
#endif

    prev_control_millis = curr_millis;
    
    // Attach encoder interrupts
#ifdef UNO_2WD
    attachInterrupt(0, LeftEncoderInterrupt, RISING);
    attachInterrupt(1, RightEncoderInterrupt, RISING);
#endif

#ifdef MEGA_4WD
    attachInterrupt(4, FLeftEncoderInterrupt, RISING);
    attachInterrupt(5, FRightEncoderInterrupt, RISING);
    attachInterrupt(0, RRightEncoderInterrupt, RISING);
    attachInterrupt(1, RLeftEncoderInterrupt, RISING);
#endif

    // Necessary for encoder interrupts to initialize
    delay(200);
}

void loop() {

    curr_millis = millis();
    
#ifdef ROS_TELEOP
    UpdateRos(); // publish and subscribe to ROS topics
    robot.Drive(cmd_vel.linear.x, cmd_vel.angular.z); // drive the robot
#endif

    // Test individual components
//    ReadEncoders(); // Passed!
//    Farmaid::TestEncoderClass(left_encoder, right_encoder); // Passed!
//    Farmaid::TestEncoderClass(fleft_encoder, fright_encoder); // Passed!
//    Farmaid::TestEncoderClass(rleft_encoder, rright_encoder); // Passed!

    // Test motor open loop
//    Farmaid::TestMotorOpenLoop(left_motor, 1); // Passed!
//    Farmaid::TestMotorOpenLoop(left_motor, -1); // Passed!
//    Farmaid::TestMotorOpenLoop(right_motor, 1); // Passed!
//    Farmaid::TestMotorOpenLoop(right_motor, -1); // Passed!

//    Farmaid::TestMotorOpenLoop(fleft_motor, 1); // Passed with Mega 2wd robot
//    Farmaid::TestMotorOpenLoop(fleft_motor, -1); // Passed with Mega 2wd robot
//    Farmaid::TestMotorOpenLoop(fright_motor, 1); // Passed with Mega 2wd robot
//    Farmaid::TestMotorOpenLoop(fright_motor, -1); // Passed with Mega 2wd robot
//    Farmaid::TestMotorOpenLoop(rleft_motor, 1); // Passed with Mega 2wd robot
//    Farmaid::TestMotorOpenLoop(rleft_motor, -1); // Passed with Mega 2wd robot
//    Farmaid::TestMotorOpenLoop(rright_motor, 1); // Passed with Mega 2wd robot
//    Farmaid::TestMotorOpenLoop(rright_motor, -1); // Passed with Mega 2wd robot

    // Find deadband
//    left_motor.set_command(0.28);
//    left_motor.set_command(-0.11);
//    right_motor.set_command(0.35);
//    right_motor.set_command(-0.09);

//      fleft_motor.set_command(0.04);
//      fleft_motor.set_command(-0.04);
//      fright_motor.set_command(0.04);
//      fright_motor.set_command(-0.04);
//      rright_motor.set_command(0.04);
//      rright_motor.set_command(-0.04);
//      rleft_motor.set_command(0.04);
//      rleft_motor.set_command(-0.04);

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

//    fleft_pid.set_gains(7.5, 0.0, 0.25, 10.0 * control_period / 1000.0);
//    fleft_pid.set_max_deadband(0.04);
//    fleft_pid.set_min_deadband(-0.04);
//    Farmaid::TestMotorPositionControl(fleft_motor, fleft_encoder, fleft_pid, 1); // Passed with Mega 2wd robot
//    Farmaid::TestMotorPositionControl(fleft_motor, fleft_encoder, fleft_pid, -1); // Passed with Mega 2wd robot

//    fright_pid.set_gains(7.0, 0.0, 0.25, 10.0 * control_period / 1000.0);
//    fright_pid.set_max_deadband(0.04);
//    fright_pid.set_min_deadband(-0.04);
//    Farmaid::TestMotorPositionControl(fright_motor, fright_encoder, fright_pid, 1); // Passed with Mega 2wd robot
//    Farmaid::TestMotorPositionControl(fright_motor, fright_encoder, fright_pid, -1); // Passed with Mega 2wd robot

//    rleft_pid.set_gains(7.5, 0.0, 0.25, 10.0 * control_period / 1000.0);
//    rleft_pid.set_max_deadband(0.15);
//    rleft_pid.set_min_deadband(-0.1);
//    Farmaid::TestMotorPositionControl(rleft_motor, rleft_encoder, rleft_pid, 1); // Passed with Mega 2wd robot
//    Farmaid::TestMotorPositionControl(rleft_motor, rleft_encoder, rleft_pid, -1); // Passed with Mega 2wd robot

//    rright_pid.set_gains(7.0, 0.0, 0.25, 10.0 * control_period / 1000.0);
//    rright_pid.set_max_deadband(0.18);
//    rright_pid.set_min_deadband(-0.1);
//    Farmaid::TestMotorPositionControl(rright_motor, rright_encoder, rright_pid, 1); // Passed with Mega 2wd robot
//    Farmaid::TestMotorPositionControl(rright_motor, rright_encoder, rright_pid, -1); // Passed with Mega 2wd robot

    // Find maximum speed of each motor to add feedforward to PID controller
    
//    Farmaid::TestMaxSpeed(left_motor, left_encoder, 1); // 13.09 rad/s * 0.065/2 m = 0.425 m/s
//    Farmaid::TestMaxSpeed(left_motor, left_encoder, -1); // 12.44 rad/s * 0.065/2 m = 0.404 m/s
//    Farmaid::TestMaxSpeed(right_motor, right_encoder, 1); // 13.09 rad/s * 0.065/2 m = 0.425 m/s
//    Farmaid::TestMaxSpeed(right_motor, right_encoder, -1); // 13.09 rad/s * 0.065/2 m = 0.425 m/s

//    Farmaid::TestMaxSpeed(fleft_motor, fleft_encoder, 1); // Passed with Mega 2wd robot // 9.23 rad/s
//    Farmaid::TestMaxSpeed(fleft_motor, fleft_encoder, -1); // Passed with Mega 2wd robot // 9.23 rad/s
//    Farmaid::TestMaxSpeed(fright_motor, fright_encoder, 1); // Passed with Mega 2wd robot // 9.23 rad/s
//    Farmaid::TestMaxSpeed(fright_motor, fright_encoder, -1); // Passed with Mega 2wd robot // 9.23 rad/s

//    Farmaid::TestMaxSpeed(rleft_motor, rleft_encoder, 1); // Passed with Mega 2wd robot
//    Farmaid::TestMaxSpeed(rleft_motor, rleft_encoder, -1); // Passed with Mega 2wd robot
//    Farmaid::TestMaxSpeed(rright_motor, rright_encoder, 1); // Passed with Mega 2wd robot
//    Farmaid::TestMaxSpeed(rright_motor, rright_encoder, -1); // Passed with Mega 2wd robot

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

//    fleft_pid.set_gains(12.0, 3.0, 0.1, 10.0 * control_period / 1000.0);
//    fleft_pid.set_antiwindup(1.2); // Antiwindup makes it worse. Need to debug this.
//    Farmaid::TestMotorVelocityStep(fleft_motor, fleft_encoder, fleft_pid, 1); // Passed with Mega 2wd robot
//    fleft_pid.set_gains(11.0, 2.5, 0.0, 10.0 * control_period / 1000.0);
//    Farmaid::TestMotorVelocityStep(fleft_motor, fleft_encoder, fleft_pid, -1); // Passed with Mega 2wd robot

//    fright_pid.set_gains(12.0, 3.0, 0.1, 10.0 * control_period / 1000.0);
//    Farmaid::TestMotorVelocityStep(fright_motor, fright_encoder, fright_pid, 1); // Passed with Mega 2wd robot
//    fright_pid.set_gains(11.0, 2.0, 0.0, 10.0 * control_period / 1000.0);
//    Farmaid::TestMotorVelocityStep(fright_motor, fright_encoder, fright_pid, -1); // Passed with Mega 2wd robot

//    rleft_pid.set_gains(12.0, 3.0, 0.1, 10.0 * control_period / 1000.0);
//    rleft_pid.set_antiwindup(1.2); // Antiwindup makes it worse. Need to debug this.
//    Farmaid::TestMotorVelocityStep(rleft_motor, rleft_encoder, rleft_pid, 1); // Passed with Mega 2wd robot
//    rleft_pid.set_gains(11.0, 2.5, 0.0, 10.0 * control_period / 1000.0);
//    Farmaid::TestMotorVelocityStep(rleft_motor, rleft_encoder, rleft_pid, -1); // Passed with Mega 2wd robot

//    rright_pid.set_gains(12.0, 3.0, 0.1, 10.0 * control_period / 1000.0);
//    Farmaid::TestMotorVelocityStep(rright_motor, rright_encoder, rright_pid, 1); // Passed with Mega 2wd robot
//    rright_pid.set_gains(11.0, 2.0, 0.0, 10.0 * control_period / 1000.0);
//    Farmaid::TestMotorVelocityStep(rright_motor, rright_encoder, rright_pid, -1); // Passed with Mega 2wd robot

    // Step velocity commands with PID and feedforward
//    left_pid.set_gains(2.0, 0.0, 0.0, 10.0 * control_period / 1000.0);
//    left_pid.set_feedforward_flag(true);
//    Farmaid::TestMotorVelocityStep(left_motor, left_encoder, left_pid, 1); // Passed!
//    Farmaid::TestMotorVelocityStep(left_motor, left_encoder, left_pid, -1); // Passed!

//    right_pid.set_gains(2.0, 0.0, 0.0, 10.0 * control_period / 1000.0);
//    right_pid.set_feedforward_flag(true);
//    Farmaid::TestMotorVelocityStep(right_motor, right_encoder, right_pid, 1); // Passed!
//    Farmaid::TestMotorVelocityStep(right_motor, right_encoder, right_pid, -1); // Passed!

//    fleft_pid.set_gains(2.0, 0.0, 0.0, 10.0 * control_period / 1000.0);
//    fleft_pid.set_feedforward_flag(true);
//    Farmaid::TestMotorVelocityStep(fleft_motor, fleft_encoder, fleft_pid, 1); // Passed with Mega 2wd robot
//    Farmaid::TestMotorVelocityStep(fleft_motor, fleft_encoder, fleft_pid, -1); // Passed with Mega 2wd robot

//    fright_pid.set_gains(2.0, 0.0, 0.0, 10.0 * control_period / 1000.0);
//    fright_pid.set_feedforward_flag(true);
//    Farmaid::TestMotorVelocityStep(fright_motor, fright_encoder, fright_pid, 1); // Passed with Mega 2wd robot
//    Farmaid::TestMotorVelocityStep(fright_motor, fright_encoder, fright_pid, -1); // Passed with Mega 2wd robot

//    rleft_pid.set_gains(2.0, 0.0, 0.0, 10.0 * control_period / 1000.0);
//    rleft_pid.set_feedforward_flag(true);
//    Farmaid::TestMotorVelocityStep(rleft_motor, rleft_encoder, rleft_pid, 1); // Passed with Mega 2wd robot
//    Farmaid::TestMotorVelocityStep(rleft_motor, rleft_encoder, rleft_pid, -1); // Passed with Mega 2wd robot

//    rright_pid.set_gains(2.0, 0.0, 0.0, 10.0 * control_period / 1000.0);
//    rright_pid.set_feedforward_flag(true);
//    Farmaid::TestMotorVelocityStep(rright_motor, rright_encoder, rright_pid, 1); // Passed with Mega 2wd robot
//    Farmaid::TestMotorVelocityStep(rright_motor, rright_encoder, rright_pid, -1); // Passed with Mega 2wd robot

    // Sinusoidal velocity command with PID and feedforward
    
//    left_pid.set_gains(3.0, 0.0, 0.0, 10.0 * control_period / 1000.0);
//    left_pid.set_feedforward_flag(true);
//    Farmaid::TestMotorVelocitySine(left_motor, left_encoder, left_pid, 1); // Passed!
//    Farmaid::TestMotorVelocitySine(left_motor, left_encoder, left_pid, -1); // Passed!

//    right_pid.set_gains(3.0, 0.0, 0.0, 10.0 * control_period / 1000.0);
//    right_pid.set_feedforward_flag(true);
//    Farmaid::TestMotorVelocitySine(right_motor, right_encoder, right_pid, 1); // Passed!
//    Farmaid::TestMotorVelocitySine(right_motor, right_encoder, right_pid, -1); // Passed!

//    fleft_pid.set_gains(3.0, 0.0, 0.0, 10.0 * control_period / 1000.0);
//    fleft_pid.set_feedforward_flag(true);
//    Farmaid::TestMotorVelocitySine(fleft_motor, fleft_encoder, fleft_pid, 1); // Passed with Mega 2wd robot
//    Farmaid::TestMotorVelocitySine(fleft_motor, fleft_encoder, fleft_pid, -1); // Passed with Mega 2wd robot

//    fright_pid.set_gains(3.0, 0.0, 0.0, 10.0 * control_period / 1000.0);
//    fright_pid.set_feedforward_flag(true);
//    Farmaid::TestMotorVelocitySine(fright_motor, fright_encoder, fright_pid, 1); // Passed with Mega 2wd robot
//    Farmaid::TestMotorVelocitySine(fright_motor, fright_encoder, fright_pid, -1); // Passed with Mega 2wd robot

//    rleft_pid.set_gains(2.0, 0.0, 0.0, 10.0 * control_period / 1000.0);
//    rleft_pid.set_feedforward_flag(true);
//    Farmaid::TestMotorVelocitySine(rleft_motor, rleft_encoder, rleft_pid, 1); //
//    Farmaid::TestMotorVelocitySine(rleft_motor, rleft_encoder, rleft_pid, -1); //

//    rright_pid.set_gains(3.0, 0.0, 0.0, 10.0 * control_period / 1000.0);
//    rright_pid.set_feedforward_flag(true);
//    Farmaid::TestMotorVelocitySine(rright_motor, rright_encoder, rright_pid, 1); //
//    Farmaid::TestMotorVelocitySine(rright_motor, rright_encoder, rright_pid, -1); //

    // Test robot drive method (PID gains and feedforward_flag set in Robot constructor)
//    Farmaid::TestDiffSteerForward(robot); // Passed!
//    Farmaid::TestDiffSteerRotate(robot);
//    Farmaid::TestDiffSteerCircle(robot, 0.3, -1);

}
