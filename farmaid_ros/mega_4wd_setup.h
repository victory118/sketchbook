#ifndef MEGA_4WD_HELPER_H
#define MEGA_4WD_HELPER_H

#include "drv8833_motor.h"
#include "ls7184_encoder.h"
#include "pid_controller.h"
#include "robot.h"

const float wheelbase = 0.14; // [m]
const float wheel_radius = 0.065 / 2; // [m]
const float no_load_rps = 0.4 / wheel_radius; // determined experimentally in [m/s] --> [rad/s]

// Initialize encoder parameter struct
// {byte clk_pin (byte), byte dir_pin, float counts_per_rev, float sample_period, float filt_tc}
Farmaid::EncoderParams fleft_encoder_p = {19, 23, 1920.0, control_period / 1000.0, 4.0 * control_period / 1000.0};
Farmaid::EncoderParams fright_encoder_p = {18, 22, 1920.0, control_period / 1000.0, 4.0 * control_period / 1000.0};
Farmaid::EncoderParams rright_encoder_p = {2, 25, 1920.0, control_period / 1000.0, 4.0 * control_period / 1000.0};
Farmaid::EncoderParams rleft_encoder_p = {3, 24, 1920.0, control_period / 1000.0, 4.0 * control_period / 1000.0};

// Initialize motor parameter struct
// {byte pwm_pin, byte dir_pin, unsigned int max_command, float no_load_rps}
Farmaid::MotorParams fleft_motor_p = {5, 4, 255, no_load_rps};
Farmaid::MotorParams fright_motor_p = {7, 6, 255, no_load_rps};
Farmaid::MotorParams rright_motor_p = {9, 8, 255, no_load_rps};
Farmaid::MotorParams rleft_motor_p = {11, 10, 255, no_load_rps};

// Initialize PID controller parameter struct
// {float p_gain, float i_gain, float d_gain, float filt_const, float sample_period}
Farmaid::PidParams fleft_pid_p = {1.0, 0.0, 0.0, 0.0, control_period / 1000.0};
Farmaid::PidParams fright_pid_p = {1.0, 0.0, 0.0, 0.0, control_period / 1000.0};
Farmaid::PidParams rright_pid_p = {1.0, 0.0, 0.0, 0.0, control_period / 1000.0};
Farmaid::PidParams rleft_pid_p = {1.0, 0.0, 0.0, 0.0, control_period / 1000.0};

// Encoder counter variables are global because they use hardware interrupts
volatile int fleft_encoder_count = 0;
volatile bool fleft_encoder_change_flag = false;
volatile int fright_encoder_count = 0;
volatile bool fright_encoder_change_flag = false;
volatile int rleft_encoder_count = 0;
volatile bool rleft_encoder_change_flag = false;
volatile int rright_encoder_count = 0;
volatile bool rright_encoder_change_flag = false;

void ReadEncoders()
{

    if (curr_millis - prev_serial_millis >= serial_period)
    {
        // This reads directly from the global variables
        if (fleft_encoder_change_flag) {
            fleft_encoder_change_flag = false;
            Serial.print("Front Left encoder count = ");
            Serial.println(fleft_encoder_count);
        }

        if (fright_encoder_change_flag) {
            fright_encoder_change_flag = false;
            Serial.print("Front Right encoder count = ");
            Serial.println(fright_encoder_count);
        }

        if (rleft_encoder_change_flag) {
            rleft_encoder_change_flag = false;
            Serial.print("Rear Left encoder count = ");
            Serial.println(rleft_encoder_count);
        }

        if (rright_encoder_change_flag) {
            rright_encoder_change_flag = false;
            Serial.print("Rear Right encoder count = ");
            Serial.println(rright_encoder_count);
        }

        prev_serial_millis = curr_millis;
    }        
}

void FLeftEncoderInterrupt() {
  if (digitalRead(fleft_encoder_p.dir_pin) == HIGH) {
    fleft_encoder_count++;
  }
  else {
    fleft_encoder_count--;
  }
  fleft_encoder_change_flag = true;
}

void FRightEncoderInterrupt() {
  if (digitalRead(fright_encoder_p.dir_pin) == LOW) {
    fright_encoder_count++;
  }
  else {
    fright_encoder_count--;
  }
  fright_encoder_change_flag = true;
}

void RLeftEncoderInterrupt() {
  if (digitalRead(rleft_encoder_p.dir_pin) == HIGH) {
    rleft_encoder_count++;
  }
  else {
    rleft_encoder_count--;
  }
  rleft_encoder_change_flag = true;
}

void RRightEncoderInterrupt() {
  if (digitalRead(rright_encoder_p.dir_pin) == LOW) {
    rright_encoder_count++;
  }
  else {
    rright_encoder_count--;
  }
  rright_encoder_change_flag = true;
}

#endif
