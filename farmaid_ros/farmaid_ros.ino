#include <Arduino.h>
#include "differential_drive.h"
#include "drv8833_motor.h"
#include "encoder.h"
#include "pid_controller.h"
#include "params.h"

unsigned long ros_prev_millis;
unsigned long control_prev_millis;
unsigned long curr_millis;
const unsigned long ros_period = 40; // ROS communication period [millis]
const unsigned long control_period = 5; // control loop period [millis]

const float wheelbase = 0.14; // [m]
const float wheel_radius = 0.065 / 2; [m]
const float no_load_rps = 0.4 / wheel_radius; // determined experimentally in [m/s] --> [rad/s]

// Encoder counter variables must be global because they use hardware interrupts
volatile int left_encoder_count = 0;
volatile boolean left_encoder_change_flag = false;
volatile int right_encoder_count = 0;
volatile boolean right_encoder_change_flag = false;

// Initialize encoder parameter struct
// {byte clk_pin (byte), byte dir_pin, float counts_per_rev, unsigned int sample_period, int *count_ptr}
Farmaid::EncoderParams left_encoder_p = {2, 8, 1920.0, control_period, &left_encoder_count};
Farmaid::EncoderParams right_encoder_p = {3, 9, 1920.0, control_period, &right_encoder_count};

// Initialize motor parameter struct
// {byte pwm_pin, byte dir_pin, unsigned int max_command, float no_load_rps}
Farmaid::MotorParams left_motor_p = {6, 7, 255, no_load_rps};
Farmaid::MotorParams right_motor_p = {5, 4, 255, no_load_rps};

// Initialize PID controller parameter struct
// {float p_gain, float i_gain, float d_gain, float filt_const, unsigned int sample_period}
Farmaid::PidParams left_pid_p = {1.0, 0.0, 0.0, 0.0, control_period};
Farmaid::PidParams right_pid_p = {1.0, 0.0, 0.0, 0.0, control_period};

// Initialize motor controller parameter struct
// {EncoderParams encoder_p, MotorParams motor_p, PidParams pid_p}
Farmaid::MotorControllerParams left_motor_controller_p = {left_encoder_p, left_motor_p, left_pid_p};
Farmaid::MotorControllerParams right_motor_controller_p = {right_encoder_p, right_motor_p, right_pid_p};

// Initialize robot parameter struct
// {MotorControllerParams left_p, MotorControllerParams right_p, float wheelbase, float wheel_radius}
Farmaid::RobotParams robot_p = {left_motor_controller_p, right_motor_controller_p, wheelbase, wheel_radius}

void LeftEncoderInterrupt() {
  if (digitalRead(left_encoder_count) == HIGH) {
    left_encoder_count++;
  }
  else {
    left_encoder_count--;
  }
  left_encoder_change_flag = true;
}

void RightEncoderInterrupt() {
  if (digitalRead(right_encoder_count) == LOW) {
    right_encoder_count++;
  }
  else {
    right_encoder_count--;
  }
  right_encoder_change_flag = true;
}

namespace Farmaid
{
    class Robot
    {
    public:
        Robot(MotorControllerParams left_motor_controller_p, MotorControllerParams right_motor_controller_p,
              float wheelbase, float wheel_radius)
        : left_motor_controller_(MotorController(left_motor_controller_p)),
          right_motor_controller_(MotorController(right_motor_controller_p)),
          wheelbase_(wheelbase), wheel_radius_(wheel_radius)
        {
            // Calculate max_vel

            // Calculate max_ang_vel

            
        }

    void Drive(float vel, float ang_vel)
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

        UniToDiff(vel,Ang
    }

    void UniToDiff(float vel, float ang_vel)
        {
            // This function ensures that ang_vel is respected as best as possible
            // by scaling vel.
            // vel - desired robot linear velocity [m/s]
            // ang_vel - desired robot angular velocity [rad/s]

            if (!ensure_ang_vel_)
            {
                right_wheel_vel_ = (vel + ang_vel * wheelbase_ / 2.0);
                left_wheel_vel_ = (vel - ang_vel* wheelbase_ / 2.0);
                return;
            }
            
            // 1. Limit vel and ang_vel to be within the possible range
            float lim_ang_vel = max(min(ang_vel, max_ang_vel_), -max_ang_vel_);
            float lim_vel = max(min(vel, max_vel_), -max_vel_);

            // 2. Compute left and right wheel velocities required to achieve limited vel and ang_vel
            float lim_right_wheel_vel = (lim_vel + lim_ang_vel * wheelbase_ / 2.0);
            float lim_left_wheel_vel = (lim_vel - lim_ang_vel* wheelbase_ / 2.0);

            // 3. Find max and min of the limited wheel velocities
            float max_lim_wheel_vel = max(lim_right_wheel_vel, lim_left_wheel_vel);
            float min_lim_wheel_vel = min(lim_right_wheel_vel, lim_left_wheel_vel);

            // 4. Shift limited wheel velocities if they exceed the maximum wheel velocity
            if (max_lim_wheel_vel > max_vel_)
            {
                right_wheel_vel_ = lim_right_wheel_vel - (max_lim_wheel_vel - max_vel_);
                left_wheel_vel_ = lim_left_wheel_vel - (max_lim_wheel_vel - max_vel_);
            }
            else if (min_lim_wheel_vel < -max_vel_) 
            {
                right_wheel_vel_ = lim_right_wheel_vel - (min_lim_wheel_vel + max_vel_);
                left_wheel_vel_ = lim_left_wheel_vel - (min_lim_wheel_vel + max_vel_);
            }
            else
            {
                right_wheel_vel_ = lim_right_wheel_vel;
                left_wheel_vel_ = lim_left_wheel_vel;
            }      
        }
        
    private:
        MotorController left_motor_controller_;
        MotorController right_motor_controller_;

        float wheelbase_;
        float wheel_radius_;

        float max_vel_; // maximum forward velocity of robot with no rotation [m/s]
        float max_ang_vel_; // maximum angular velocity of robot with pure rotation [rad/s]
    };
};

// Initialize objects
Farmaid::Encoder left_encoder = Farmaid::Encoder(left_encoder_p);
Farmaid::Encoder right_encoder = Farmaid::Encoder(right_encoder_p);
Farmaid::Motor left_motor = Farmaid::Motor(left_motor_p);
Farmaid::Motor right_motor = Farmaid::Motor(right_motor_p);
Farmaid::PidController left_pid = Farmaid::PidController(left_pid_p);
Farmaid::PidController right_pid = Farmaid::PidController(right_pid_p);
Farmaid::MotorController left_motor_controller = Farmaid::MotorController(left_motor_controller_p);
Farmaid::MotorController right_motor_controller = Farmaid::MotorController(right_motor_controller_p);
Farmaid::Robot robot = Farmaid::Robot(robot_p); // instantiate robot

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
//        TestEncoderManual();
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

void TestEncoderManual()
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

void TestPositionControl()
{
    Farmaid::Encoder left_encoder = Farmaid::Encoder(kLeftEncClkPin, kLeftEncDirPin, kControlSampleTime);
    Farmaid::Motor motor = Farmaid::Motor(kLeftMotorPwmPin, kLeftMotorDirPin);
    Farmaid::PidController motor_pid = Farmaid::PidController(1.0, 0.1, 0.0, 0.0, kControlSampleTime)

    unsigned long current_time = micros();

    

    
    
}
