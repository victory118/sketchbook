/**
 * @file drv8833_motor.h
 * @brief Motor device driver for the Adafruit DRV8833 dual motor driver
 * @author Victor Yu
 */

#include <Arduino.h>

namespace Farmaid
{

    struct MotorParams
    {
        byte pwm_pin;
        byte dir_pin;
        unsigned int max_command; //  0-255 for 0-100% PWM duty cycle
        float no_load_rps; // max speed with no load [rad/s]
    }
    
    class Motor
    {
    public:
        /*
         * @brief Class constructor
         * @param dir_pin_ the direction pin 
         * @param pwm_pin the PWM pin
         */
         Motor(MotorParams p)
            : pwm_pin_(p.pwm_pin), dir_pin_(p.dir_pin),
              max_command_(p.max_command), command_(0)
         {
             pinMode(pwm_pin, OUTPUT);
             pinMode(dir_pin, OUTPUT);
         }

        void set_command(float value)
        {
            // input value is limited to range [-1.0, 1.0] and then mapped to range [-max_command_, +max_command_]
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

void DoMotorOpenLoopTest(Motor motor)
{

    Serial.println("Starting motor open loop test.");
    Serial.println("");
    delay(200);

    float factor = 0.5;
    int test_array[7] = {0, 1, 2, 3, 2, 1, 0};
    int array_size = sizeof(test_array) / sizeof(test_array[0]);
    
    Serial.println("Moving forward...");
    for (int i = 0; i < array_size; i++) {
        motor.set_command(test_array[i] * factor);
        Serial.println("Motor command = " + motor.get_command());
        delay(2000);
    }

    Serial.println("Moving backward...");
    for (int i = 0; i < array_size; i++)
    {
        motor.set_command(-test_array[i] * factor);
        Serial.println("Motor command = " + motor.get_command());
        delay(2000);
    }
}
