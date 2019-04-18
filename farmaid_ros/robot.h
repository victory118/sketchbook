#ifndef ROBOT_H
#define ROBOT_H

namespace Farmaid
{

    struct DiffDriveWheelVel
    {
        // struct to hold left and right wheel velocity variables
        float left; // left wheel velocity [m/s]
        float right;// right wheel velocity [m/s]
    };
  
    class Robot
    {
    public:
        Robot(Encoder left_encoder, Encoder right_encoder,
              Motor left_motor, Motor right_motor,
              PidController left_pid, PidController right_pid,
              float wheelbase, float wheel_radius)
        : left_encoder_(left_encoder), right_encoder_(right_encoder),
          left_motor_(left_motor), right_motor_(right_motor),
          left_pid_(left_pid), right_pid_(right_pid),
          wheelbase_(wheelbase), wheel_radius_(wheel_radius),
          ensure_ang_vel_(true)
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

        DiffDriveWheelVel UniToDiff(float vel, float ang_vel)
        {
            // This function ensures that ang_vel is respected as best as possible
            // by scaling vel.
            // vel - desired robot linear velocity [m/s]
            // ang_vel - desired robot angular velocity [rad/s]

            DiffDriveWheelVel diff_drive_wheel_vel = {0.0, 0.0};

            if (!ensure_ang_vel_)
            {
                diff_drive_wheel_vel.right = (vel + ang_vel * wheelbase_ / 2.0);
                diff_drive_wheel_vel.left = (vel - ang_vel* wheelbase_ / 2.0);
                return diff_drive_wheel_vel;
            }
            
            // 1. Limit vel and ang_vel to be within the possible range
            float lim_ang_vel = max(min(ang_vel, max_ang_vel_), -max_ang_vel_);
            float lim_vel = max(min(vel, max_vel_), -max_vel_);

            // 2. Compute left and right wheel velocities required to achieve limited vel and ang_vel
            float lim_right_wheel_vel = (lim_vel + lim_ang_vel * wheelbase_ / 2.0);
            float lim_left_wheel_vel = (lim_vel - lim_ang_vel* wheelbase_ / 2.0);

            // 3. Find max and min of the limited wheel velocities
            float max_lim_wheel_vel = max(lim_right_wheel_vel, lim_left_wheel_vel);
            float min_lim_wheel_vel = min(lim_right_wheel_vel, lim_left_wheel_vel);

            // 4. Shift limited wheel velocities if they exceed the maximum wheel velocity
            if (max_lim_wheel_vel > max_vel_)
            {
                diff_drive_wheel_vel.right = lim_right_wheel_vel - (max_lim_wheel_vel - max_vel_);
                diff_drive_wheel_vel.left = lim_left_wheel_vel - (max_lim_wheel_vel - max_vel_);
            }
            else if (min_lim_wheel_vel < -max_vel_) 
            {
                diff_drive_wheel_vel.right = lim_right_wheel_vel - (min_lim_wheel_vel + max_vel_);
                diff_drive_wheel_vel.left = lim_left_wheel_vel - (min_lim_wheel_vel + max_vel_);
            }
            else
            {
                diff_drive_wheel_vel.right = lim_right_wheel_vel;
                diff_drive_wheel_vel.left = lim_left_wheel_vel;
            }

            return diff_drive_wheel_vel;
        }

        float get_max_vel() { return max_vel_; }
        float get_max_ang_vel() { return max_ang_vel_; }
        bool get_ensure_ang_vel() { return ensure_ang_vel_; }
        void set_max_vel(float max_vel) { max_vel_ = max_vel; }
        void set_max_ang_vel(float max_ang_vel) { max_ang_vel_ = max_ang_vel; }
        void set_ensure_ang_vel(bool ensure_ang_vel) { ensure_ang_vel_ = ensure_ang_vel; }
        
    private:
        Encoder left_encoder_;
        Encoder right_encoder_;

        Motor left_motor_;
        Motor right_motor_;

        PidController left_pid_;
        PidController right_pid_;

        float wheelbase_;
        float wheel_radius_;

        float max_vel_; // maximum forward velocity of robot with no rotation [m/s]
        float max_ang_vel_; // maximum angular velocity of robot with pure rotation [rad/s]

        bool ensure_ang_vel_;
    };
};

#endif
