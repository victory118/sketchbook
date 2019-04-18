#ifndef FOUR_WHEEL_DRIVE_H
#define FOUR_WHEEL_DRIVE_H

#include "diff_steer.h"

namespace Farmaid
{  
    class FourWheelDrive : public DiffSteer
    {
    public:
        FourWheelDrive(Encoder fleft_encoder, Encoder fright_encoder,
              Encoder rleft_encoder, Encoder rright_encoder,
              Motor fleft_motor, Motor fright_motor,
              Motor rleft_motor, Motor rright_motor,
              PidController fleft_pid, PidController fright_pid,
              PidController rleft_pid, PidController rright_pid,
              float wheelbase, float wheel_radius)
        : fleft_encoder_(fleft_encoder), fright_encoder_(fright_encoder),
          rleft_encoder_(rleft_encoder), rright_encoder_(rright_encoder),
          fleft_motor_(fleft_motor), fright_motor_(fright_motor),
          rleft_motor_(rleft_motor), rright_motor_(rright_motor),
          fleft_pid_(fleft_pid), fright_pid_(fright_pid),
          rleft_pid_(rleft_pid), rright_pid_(rright_pid),
          DiffSteer(wheelbase, wheel_radius)
        {
            max_vel_ = min(min(fleft_motor_.get_no_load_rps(), fright_motor_.get_no_load_rps()),
                           min(rleft_motor_.get_no_load_rps(), rright_motor_.get_no_load_rps())) * wheel_radius_;
            max_ang_vel_ = max_vel_ / (wheelbase_ / 2);

            fleft_pid_.set_gains(3.0, 0.0, 0.0, 10.0 * fleft_pid_.get_sample_period());
            fleft_pid_.set_feedforward_flag(true);
            fright_pid_.set_gains(3.0, 0.0, 0.0, 10.0 * fright_pid_.get_sample_period());
            fright_pid_.set_feedforward_flag(true);

            rleft_pid_.set_gains(3.0, 0.0, 0.0, 10.0 * rleft_pid_.get_sample_period());
            rleft_pid_.set_feedforward_flag(true);
            rright_pid_.set_gains(3.0, 0.0, 0.0, 10.0 * rright_pid_.get_sample_period());
            rright_pid_.set_feedforward_flag(true);
        }

        void Drive(float vel, float ang_vel)
        {
            // Process encoder measurements
            fleft_encoder_.ProcessMeasurement();
            fright_encoder_.ProcessMeasurement();
            rleft_encoder_.ProcessMeasurement();
            rright_encoder_.ProcessMeasurement();

            // Calculate the current wheel velocities
            float curr_fleft_wheel_vel = fleft_encoder_.get_vel_rps() * wheel_radius_;
            float curr_fright_wheel_vel = fright_encoder_.get_vel_rps() * wheel_radius_;
            float curr_rleft_wheel_vel = rleft_encoder_.get_vel_rps() * wheel_radius_;
            float curr_rright_wheel_vel = rright_encoder_.get_vel_rps() * wheel_radius_;

            // Compute the desired wheel velocities
            DiffDriveWheelVel des_wheel_vel = UniToDiff(vel, ang_vel);

            // Compute controller commands based on desired and current wheel velocities
            float fleft_motor_command = fleft_pid_.ComputeCommand(des_wheel_vel.left, curr_fleft_wheel_vel);
            float fright_motor_command = fright_pid_.ComputeCommand(des_wheel_vel.right, curr_fright_wheel_vel);
            float rleft_motor_command = rleft_pid_.ComputeCommand(des_wheel_vel.left, curr_rleft_wheel_vel);
            float rright_motor_command = rright_pid_.ComputeCommand(des_wheel_vel.right, curr_rright_wheel_vel);

            // Send controller commands to each motor
            fleft_motor_.set_command(fleft_motor_command);
            fright_motor_.set_command(fright_motor_command);
            rleft_motor_.set_command(rleft_motor_command);
            rright_motor_.set_command(rright_motor_command);
        }
        
    private:
        Encoder fleft_encoder_;
        Encoder fright_encoder_;
        Encoder rleft_encoder_;
        Encoder rright_encoder_;

        Motor fleft_motor_;
        Motor fright_motor_;
        Motor rleft_motor_;
        Motor rright_motor_;

        PidController fleft_pid_;
        PidController fright_pid_;
        PidController rleft_pid_;
        PidController rright_pid_;
    };
};

#endif
