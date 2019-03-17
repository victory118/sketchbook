#include <Arduino.h>
#include "differential_drive.h"
#include "drv8833_motor.h"
#include "encoder.h"
#include "pid_controller.h"

// Control loop sample rate
const float kControlFreq = 100; // Hz
const float kControlSampleTime = 1.0 / kControlFreq; // sec
const float kControlSampleTimeMicros = kControlSampleTime * 1e6; // microsec
unsigned long prev_control_time_micros = 0;

// ROS communication sample rate
const float kRosFreq = 50; // Hz
const float kRosSampleTime = 1.0 / kRosFreq; // sec
const float kRosSampleTimeMicros = kRosSampleTime * 1e6; // microsec
unsigned long prev_ros_time_micros = 0;

unsigned long curr_time_micros;

// Robot parameters
const float kWheelbase = 0.14; // meters
const float kWheelRadius = 0.065 / 2; // meters

// Motor hardware interface parameters
const int kLeftMotorPwmPin = 6;
const int kLeftMotorDirPin = 7;
const int kRightMotorPwmPin = 5;
const int kRightMotorDirPin = 4;
const float kMaxMotorSpeed = 0.4 / kWheelRadius; // use the minimum of the two max motor speeds [rad/sec]
// empirically, max_wheel_vel = 0.4 m/s; in rad/s = wheel_vel [m/s] / wheel_radius [m]  

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
        : left_motor_(Motor(kLeftMotorPwmPin, kLeftMotorDirPin)),
          right_motor_(Motor(kRightMotorPwmPin, kRightMotorDirPin)),
          left_motor_pid_(PidController(p_gain, i_gain, d_gain, filt_const, kControlSampleTime)),
          right_motor_pid_(PidController(p_gain, i_gain, d_gain, filt_const, kControlSampleTime)),
          left_encoder_(Encoder(kLeftEncClkPin, kLeftEncDirPin, kControlSampleTime)),
          right_encoder_(Encoder(kRightEncClkPin, kRightEncDirPin, kControlSampleTime)),
          diff_drive_(DifferentialDrive(kWheelbase, kWheelRadius, kMaxMotorSpeed))
        {
        }

    void Execute(float vel, float ang_vel)
    {

        // Process encoder measurements
        left_encoder_.ProcessMeasurement(left_encoder_count);
        right_encoder_.ProcessMeasurement(right_encoder_count);

        // Calculate the current wheel velocities
        float curr_left_wheel_vel = diff_drive_.AngVelToWheelVel(left_encoder_.get_ang_vel_rps());
        float curr_right_wheel_vel = diff_drive_.AngVelToWheelVel(right_encoder_.get_ang_vel_rps());

        // Calculate the desired wheel velocities
        diff_drive_.MapUniToDiff(vel, ang_vel);
        float des_left_wheel_vel = diff_drive_.get_left_wheel_vel();
        float des_right_wheel_vel = diff_drive_.get_right_wheel_vel();

        // Compute controller commands based on desired and current wheel velocities
        float left_motor_command = left_motor_pid_.ComputeCommand(des_left_wheel_vel, curr_left_wheel_vel);
        float right_motor_command = right_motor_pid_.ComputeCommand(des_right_wheel_vel, curr_right_wheel_vel);

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

//Farmaid::Robot robot; // instantiate robot
bool run_test;

void setup() {
    // put your setup code here, to run once:

    // Attach encoder interrupts
    attachInterrupt(0, LeftEncoderInterrupt, RISING);
    attachInterrupt(1, RightEncoderInterrupt, RISING);

    Serial.begin(9600);       // initialize Serial Communication

    run_test = true;

    // Necessary for encoder interrupts to initialize
    delay(200);
}

void loop() {
    // put your main code here, to run repeatedly:

    curr_time_micros = micros();

    // Publish and subscribe to/from ROS master at this rate
    if ((curr_time_micros - prev_ros_time_micros) >= kRosSampleTimeMicros)
    {
//      nh.spinOnce();
        prev_ros_time_micros = curr_time_micros;
    }

    // Execute control loop at this rate
    if ((curr_time_micros - prev_control_time_micros) >= kControlSampleTimeMicros)
    {
        // Get the desired velocity and angular velocity from the subscribed topic
        float des_vel = 0; // TODO
        float des_ang_vel = 0; // TODO        right_motor.set_command(-0.0);
//      robot.Execute(des_vel, des_ang_vel);
        prev_control_time_micros = curr_time_micros;
    }

    if (run_test)
    {
//        TestMotorOpenLoop();
        TestEncodersManual();
        run_test = false;
    }

}

void TestMotorOpenLoop()
{

//    Serial.println("Testing right motor...");
//    Farmaid::Motor motor = Farmaid::Motor(kRightMotorPwmPin, kRightMotorDirPin);
         
    Serial.println("Testing left motor...");
    Farmaid::Motor motor = Farmaid::Motor(kLeftMotorPwmPin, kLeftMotorDirPin);

    delay(200);

    int run_test = true;
    float factor = 0.5;
    int test_array[7] = {0, 1, 2, 3, 2, 1, 0};
    int array_size = sizeof(test_array) / sizeof(test_array[0]);
    
    Serial.println("Moving forward...");
    for (int i = 0; i < array_size; i++) {
        motor.set_command(test_array[i] * factor);
        Serial.println(motor.get_command());
        delay(2000);
    }

    Serial.println("Moving backward...");
    for (int i = 0; i < array_size; i++)
    {
        motor.set_command(-test_array[i] * factor);
        Serial.println(motor.get_command());
        delay(2000);
    }
}

void TestEncodersManual()
{
    Farmaid::Encoder left_encoder = Farmaid::Encoder(kLeftEncClkPin, kLeftEncDirPin, kControlSampleTime);
    Farmaid::Encoder right_encoder = Farmaid::Encoder(kRightEncClkPin, kRightEncDirPin, kControlSampleTime);

//    unsigned long now = millis();
//    unsigned long prev = now;
//    int serial_period = 1000;

    while (true)
    {
        
//        if (left_encoder_change_flag) {
//            left_encoder_change_flag = false;
//            Serial.print("Left encoder count = ");
//            Serial.println(left_encoder_count);
//        }
//
//  
//        if (right_encoder_change_flag) {
//            right_encoder_change_flag = false;
//            Serial.print("Right encoder count = ");
//            Serial.println(right_encoder_count);
//        }
//
//        delay(50);
            
            left_encoder.ProcessMeasurement(left_encoder_count);
            right_encoder.ProcessMeasurement(right_encoder_count);

            if (left_encoder.get_curr_count() != left_encoder.get_prev_count())
            {
                Serial.print("Left encoder count (prev) = ");
                Serial.println(left_encoder.get_prev_count());
                Serial.print("Left encoder count = ");
                Serial.println(left_encoder.get_curr_count());
            }

            if (right_encoder.get_curr_count() != right_encoder.get_prev_count())
            {
                Serial.print("Right encoder count (prev) = ");
                Serial.println(right_encoder.get_prev_count());
                Serial.print("Right encoder count = ");
                Serial.println(right_encoder.get_curr_count());
            }

            delay(1000);

        }        
}
