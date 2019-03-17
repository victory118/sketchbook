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
         Motor(int pwm_pin, int dir_pin)
            : pwm_pin_(pwm_pin), dir_pin_(dir_pin),
              max_command_(255), command_(0)
         {
             pinMode(pwm_pin, OUTPUT);
             pinMode(dir_pin, OUTPUT);
         }

        void set_command(float value)
        {
            // input value is limited to range [-1.0, 1.0] and then mapped to range [-255, 255]
            float factor = max(min(value, 1.0f), -1.0f);
            if (factor >= 0)
            {
                command_ = (unsigned int)(max_command_ * factor);
                digitalWrite(dir_pin_, LOW);
            } else {
                command_ = (unsigned int)(max_command_ * (1.0f + factor));
                digitalWrite(dir_pin_, HIGH); 
            }
            analogWrite(pwm_pin_, command_);
        }

        int get_command() { return command_; };
         
    private:
        const int dir_pin_;
        const int pwm_pin_;
        const int max_command_;

        unsigned int command_;
    };
};
