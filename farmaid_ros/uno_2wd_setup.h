#ifndef UNO_2WD_SETUP_H
#define UNO_2WD_SETUP_H

#include "drv8833_motor.h"
#include "ls7184_encoder.h"
#include "pid_controller.h"
#include "robot.h"

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

// Encoder counter variables are global because they use hardware interrupts
volatile int left_encoder_count = 0;
volatile bool left_encoder_change_flag = false;
volatile int right_encoder_count = 0;
volatile bool right_encoder_change_flag = false;

void ReadEncoders()
{

    if (curr_millis - prev_serial_millis >= serial_period)
    {
        // This reads directly from the global variables
        if (left_encoder_change_flag) {
            left_encoder_change_flag = false;
            Serial.print("Left encoder count = ");
            Serial.println(left_encoder_count);
        }

        if (right_encoder_change_flag) {
            right_encoder_change_flag = false;
            Serial.print("Right encoder count = ");
            Serial.println(right_encoder_count);
        }

        prev_serial_millis = curr_millis;
    }        
}

void LeftEncoderInterrupt() {
  if (digitalRead(left_encoder_p.dir_pin) == HIGH) {
    left_encoder_count++;
  }
  else {
    left_encoder_count--;
  }
  left_encoder_change_flag = true;
}

void RightEncoderInterrupt() {
  if (digitalRead(right_encoder_p.dir_pin) == LOW) {
    right_encoder_count++;
  }
  else {
    right_encoder_count--;
  }
  right_encoder_change_flag = true;
}

#endif
