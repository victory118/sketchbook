#include <Arduino.h>
#include "differential_drive.h"
#include "drv8833_motor.h"
#include "encoder.h"
#include "pid_controller.h"

namespace Farmaid
{   
    class MotorController
    {
    public:
        MotorController(Encoder encoder, Motor motor, PidController pid)
        : encoder_(encoder), motor_(motor), pid_(pid)
        {
        }

        void DoVelocityControl(float des_vel)
        {
            // Process encoder measurement
            encoder_.ProcessMeasurement();

            // Get the current motor velocity
            vel_ = encoder_.get_vel_rps();

            // Compute controller command based on desired and current velocities
            command_ = pid_.ComputeCommand(des_vel, vel_);

            // Send controller command to motor
            motor_.set_command(command_);
        }

        void DoPositionControl(float des_pos)
        {
            // Process encoder measurement
            encoder_.ProcessMeasurement();

            // Get the current motor velocity
            pos_ = encoder_.get_vel_rps();

            // Compute controller command based on desired and current velocities
            command_ = pid_.ComputeCommand(des_vel, pos_);

            // Send controller command to motor
            motor_.set_command(command_); 
        }

        void Reset()
        {
            pos_ = 0;
            vel_ = 0;
        }

        float get_vel() { return vel_; }
        float get_pos() { return pos_; }

    private:
        float vel_; // [rad/s]
        float pos_; // [rad]
        float command_;
       
    }
}
