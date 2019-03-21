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
        MotorController(int motor_pwm_pin, int motor_dir_pin, int enc_clk_pin, int enc_dir_pin, float sample_time)
        : motor_(Motor(motor_pwm_pin, motor_dir_pin)),
          pid_(PidController(p_gain=0, i_gain=0, d_gain=0, filt_const=0, sample_time)),
          encoder_(Encoder(enc_clk_pin, enc_dir_pin, sample_time))
        {
        }

        void Initialize()
        {
            // Do something here
        }


        void DoVelocityControl(float des_vel_rps, float curr_vel_rps)
        {
            command = pid_.ComputeCommand(des_vel_rps, curr_vel_rps_);
            motor_.set_command(command);
        }

        void DoPositionControl(float des_pos_rad, float curr_pos_rad)
        {
            // 
        }

        void Reset()
        {
            curr_pos_rad_ = 0;
            curr_vel_rps_ = 0;
        }

        float get_vel_rps() { return curr_vel_rps_; }
        float get_pos_rad() { return curr_pos_rps_; }

    private:
        float curr_vel_rps_
        float curr_pos_rad_

        
    }
}
