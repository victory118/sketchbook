#ifndef TEST_H
#define TEST_H

#include <Arduino.h>
#include "encoder.h"

extern unsigned long curr_millis;
extern unsigned long print_prev_millis;
extern unsigned long control_prev_millis;
extern const unsigned long print_period;
extern const unsigned long control_period;

extern volatile int left_encoder_count;
extern volatile bool left_encoder_change_flag;
extern volatile int right_encoder_count;
extern volatile bool right_encoder_change_flag;

extern Farmaid::Encoder left_encoder;
extern Farmaid::Encoder right_encoder;

void ReadEncoders()
{

    if (curr_millis - print_prev_millis >= print_period)
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

        print_prev_millis = curr_millis;
    }        
}

namespace Farmaid
{
void TestEncoderClass()
{

    if (curr_millis - print_prev_millis >= print_period)
    {   
            
        left_encoder.ProcessMeasurement();
        right_encoder.ProcessMeasurement();

        if (left_encoder.get_count() != left_encoder.get_prev_count())
        {
            Serial.print("Left encoder count (prev) = ");
            Serial.println(left_encoder.get_prev_count());
            Serial.print("Left encoder count = ");
            Serial.println(left_encoder.get_count());
        }

        if (right_encoder.get_count() != right_encoder.get_prev_count())
        {
            Serial.print("Right encoder count (prev) = ");
            Serial.println(right_encoder.get_prev_count());
            Serial.print("Right encoder count = ");
            Serial.println(right_encoder.get_count());
        }

        print_prev_millis = curr_millis;
    }        
}

void TestMotorOpenLoop(Motor &motor, float factor)
{

    float command = 0;

    if (curr_millis - control_prev_millis >= control_period)
    {
        if (curr_millis < 2000)
        {
            command = 0 * factor;   
        }
        else if (curr_millis < 4000)
        {
            command = 0.5 * factor;
        }
        else if (curr_millis < 6000)
        {
            command = 1.0 * factor;
        }
        else if (curr_millis < 8000)
        {
            command = 0.5 * factor;
        }
        else
        {
            command = 0.0 * factor;
        }

        motor.set_command(command);
        control_prev_millis = curr_millis;
        Serial.println(motor.get_command() / 255.0);
    }
}

void TestMotorPositionControl(Motor &motor, Encoder &encoder, PidController &pid, float factor)
{

    float setpoint = 0;
    float units = encoder.get_counts_per_rev();
    float command = 0;
    float curr_pos = 0;

    if (curr_millis - control_prev_millis >= control_period)
    {
        if (curr_millis < 2000)
        {
            setpoint = 0 * factor;   
        }
        else if (curr_millis < 4000)
        {
            setpoint = 0.5 * factor;
        }
        else if (curr_millis < 6000)
        {
            setpoint = 1.0 * factor;
        }
        else if (curr_millis < 8000)
        {
            setpoint = 0.5 * factor;
        }
        else
        {
            setpoint = 0.0 * factor;
        }

        // Process encoder measurement
        encoder.ProcessMeasurement();

        // Get the current motor position
        curr_pos = encoder.get_count() / units;

        // Compute controller command based on desired and current position
        command = pid.ComputeCommand(setpoint, curr_pos);

        // Send controller command to motor
        motor.set_command(command);

        control_prev_millis = curr_millis;

//        Serial.print(pid.get_p_gain());
//        Serial.print(" ");
//        Serial.print(pid.get_p_control());
//        Serial.print(" ");
//        Serial.print(pid.get_error());
//        Serial.print(" ");
//        Serial.print(pid.get_command());
//        Serial.print(" ");
//        Serial.print(command);
//        Serial.print(" ");
        Serial.print(2.0);
        Serial.print(" ");
        Serial.print(-2.0);
        Serial.print(" ");
        Serial.print(setpoint);
        Serial.print(" ");
        Serial.println(curr_pos);
    }
}

void TestMotorVelocityControl(Motor &motor, Encoder &encoder, PidController &pid, float factor)
{

    float setpoint = 0;
    float units = encoder.get_counts_per_rev();
    float command = 0;
    float curr_pos = 0;

    if (curr_millis - control_prev_millis >= control_period)
    {
        if (curr_millis < 2000)
        {
            setpoint = 0 * factor;   
        }
        else if (curr_millis < 4000)
        {
            setpoint = 0.5 * factor;
        }
        else if (curr_millis < 6000)
        {
            setpoint = 1.0 * factor;
        }
        else if (curr_millis < 8000)
        {
            setpoint = 0.5 * factor;
        }
        else
        {
            setpoint = 0.0 * factor;
        }

        // Process encoder measurement
        encoder.ProcessMeasurement();

        // Get the current motor position
        curr_pos = encoder.get_count() / units;

        // Compute controller command based on desired and current position
        command = pid.ComputeCommand(setpoint, curr_pos);

        // Send controller command to motor
        motor.set_command(command);

        control_prev_millis = curr_millis;

//        Serial.print(pid.get_p_gain());
//        Serial.print(" ");
//        Serial.print(pid.get_p_control());
//        Serial.print(" ");
//        Serial.print(pid.get_error());
//        Serial.print(" ");
//        Serial.print(pid.get_command());
//        Serial.print(" ");
//        Serial.print(command);
//        Serial.print(" ");
        Serial.print(2.0);
        Serial.print(" ");
        Serial.print(-2.0);
        Serial.print(" ");
        Serial.print(setpoint);
        Serial.print(" ");
        Serial.println(curr_pos);
    }
}

//void TestMaxVelocity(Motor &motor, Encoder &encoder)
//{
//
//    float command = 0;
//
//    if curr_millis - control_prev_millis) >= control_period
//        if (curr_millis < 2000)
//        {
//            command = 0;
//        }
//        else
//        {
//            command = 1.0;
//        }
//
//    Serial.print(command);
//    Serial.print(" ");
//    Serial.println(encoder.get_vel_rps());
//}

}


#endif
