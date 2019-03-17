/**
 * @file drv8833_motor.h
 * @brief Motor device driver for the Adafruit DRV8833 dual motor driver
 * @author Victor Yu
 */

#include <Arduino.h>

namespace Farmaid
{
    class Motor
    {
    public:
        /*
         * @brief Class constructor
         * @param dir_pin_ the direction pin 
         * @param pwm_pin the PWM pin
         */
         Motor(int dir_pin, int pwm_pin)
            : dir_pin_(dir_pin), pwm_pin_(pwm_pin)
         {
             pinMode(pwm_pin_, OUTPUT);
             pinMode(dir_pin_, OUTPUT);
             motor_command_ = 0;
         }

        void SetCommand(float value)
        {
            // input value is limited to range [-1.0, 1.0] and then mapped to range [-255, 255]
            float lim_value = max(min(value, 1.0f), -1.0f);
 
            if (lim_value >= 0)
            {
                digitalWrite(dir_pin_, LOW);
//                analogWrite(motorPwmPin, (unsigned int)(maxOutput * clippedSpeed)); 
            }
            else
            {
                digitalWrite(dir_pin_, HIGH);
//                analogWrite(motorPwmPin, (unsigned int)(maxOutput * (1.0f + clippedSpeed))); // is this one correct?
//                analogWrite(motorPwmPin, (unsigned int)(maxOutput * (-clippedSpeed)));  
            }
            motor_command_ = (unsigned int)(abs(lim_value) * max_command_);
            analogWrite(pwm_pin_, motor_command_);
        }
         
    private:
        int dir_pin_;
        int pwm_pin_;
        int motor_command_;
        const int max_command_ = 255;
    };
};
