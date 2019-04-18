#ifndef TWO_WHEEL_DRIVE_H
#define TWO_WHEEL_DRIVE_H

#include "diff_steer.h"

namespace Farmaid
{  
    class TwoWheelDrive : public DiffSteer
    {
    public:
        TwoWheelDrive(Encoder left_encoder, Encoder right_encoder,
              Motor left_motor, Motor right_motor,
              PidController left_pid, PidController right_pid,
              float wheelbase, float wheel_radius)
        : left_encoder_(left_encoder), right_encoder_(right_encoder),
          left_motor_(left_motor), right_motor_(right_motor),
          left_pid_(left_pid), right_pid_(right_pid),
          DiffSteer(wheelbase, wheel_radius)
        {
            max_vel_ = min(left_motor_.get_no_load_rps(), right_motor_.get_no_load_rps()) * wheel_radius_;
            max_ang_vel_ = max_vel_ / (wheelbase_ / 2);

            left_pid_.set_gains(3.0, 0.0, 0.0, 10.0 * left_pid_.get_sample_period());
            left_pid_.set_feedforward_flag(true);
            right_pid_.set_gains(3.0, 0.0, 0.0, 10.0 * right_pid_.get_sample_period());
            right_pid_.set_feedforward_flag(true);
        }

        void Drive(float vel, float ang_vel)
        {
            // Process encoder measurements
            left_encoder_.ProcessMeasurement();
            right_encoder_.ProcessMeasurement();

            // Calculate the current wheel velocities
            float curr_left_wheel_vel = left_encoder_.get_vel_rps() * wheel_radius_;
            float curr_right_wheel_vel = right_encoder_.get_vel_rps() * wheel_radius_;

            // Compute the desired wheel velocities
            DiffDriveWheelVel des_wheel_vel = UniToDiff(vel, ang_vel);

            // Compute controller commands based on desired and current wheel velocities
            float left_motor_command = left_pid_.ComputeCommand(des_wheel_vel.left, curr_left_wheel_vel);
            float right_motor_command = right_pid_.ComputeCommand(des_wheel_vel.right, curr_right_wheel_vel);

            // Send controller commands to each motor
            left_motor_.set_command(left_motor_command);
            right_motor_.set_command(right_motor_command);
        }
        
    private:
        Encoder left_encoder_;
        Encoder right_encoder_;

        Motor left_motor_;
        Motor right_motor_;

        PidController left_pid_;
        PidController right_pid_;
    };
};

#endif
