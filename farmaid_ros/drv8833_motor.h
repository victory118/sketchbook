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
         * @param dirPin the direction pin 
         * @param pwmpin the PWM pin
         */
         Motor(int dirPin, int pwmPin)
            : MotorDriver(), motorDirPin(dirPin), motorPwmPin(pwmPin)
         {
             pinMode(pwmPin, OUTPUT);
             pinMode(dirPin, OUTPUT);
         }

        void setSpeed(int desSpeed)
        {
            // desSpeed is clipped to be in range [-1.0, 1.0] and then mapped to maximum input range [-255, 255]
            clippedSpeed = max(min(desSpeed, 1.0f), -1.0f);
            currentSpeed = speed;
            if (clippedSpeed >= 0)
            {
                digitalWrite(motorDirPin, LOW);
                analogWrite(motorPwmPin, (unsigned int)(maxInput * clippedSpeed)); 
            }
            else
            {
                digitalWrite(motorDirPin, HIGH);
                analogWrite(motorPwmPin, (unsigned int)(maxInput * (1.0f + clippedSpeed))); 
            }
        }
         
    private:
        int motorDirPin;
        int motorPwmPin;
        const int maxInput = 255;
    }
}
