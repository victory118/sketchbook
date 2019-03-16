/**
 * @file drv8833_motor.h
 * @brief Motor device driver for the Adafruit DRV8833 dual motor driver
 * @author Victor Yu
 */

#include <Arduino.h>
#include "motor_driver.h"

namespace Farmaid
{
    class Motor : public MotorDriver
    {
    public:
        /*
         * @brief Class constructor
         * @param dir_pin_ the direction pin 
         * @param pwm_pin the PWM pin
         */
         Motor(int dir_pin, int pwm_pin)
            : MotorDriver(), dir_pin_(dir_pin), pwm_pin_(pwm_pin)
         {
             pinMode(pwm_pin_, OUTPUT);
             pinMode(dir_pin_, OUTPUT);
         }

        void SetCommand(int command)
        {
            // command is saturated to range [-1.0, 1.0] and then mapped to range [-255, 255]
            clipped_command = max(min(command, 1.0f), -1.0f);
 
            if (clipped_command >= 0)
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
            motorCommand = (unsigned int)(abs(clipped_command) * max_command_);
            analogWrite(motorPwmPin, motorCommand);
        }
         
    private:
        int dir_pin_;
        int pwm_pin_;
        const int max_command_ = 255;
    }
}
