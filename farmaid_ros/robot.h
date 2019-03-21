# include <Arduino.h>

namespace Farmaid
{

    struct DiffDriveWheelVel
    {
        // struct to hold left and right wheel velocity variables
        float left; // left wheel velocity [m/s]
        float right;// right wheel velocity [m/s]
    }
  
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
          wheelbase_(wheelbase), wheel_radius_(wheel_radius)
        {
            // TODO: Calculate max_vel
            max_vel = 0.0;
            // TODO: Calculate max_ang_vel
            max_ang_vel = 0.0;
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
        des_wheel_vel = UniToDiff(vel, ang_vel);

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
                right_wheel_vel_ = (vel + ang_vel * wheelbase_ / 2.0);
                left_wheel_vel_ = (vel - ang_vel* wheelbase_ / 2.0);
                return;
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
                right_wheel_vel_ = lim_right_wheel_vel - (max_lim_wheel_vel - max_vel_);
                left_wheel_vel_ = lim_left_wheel_vel - (max_lim_wheel_vel - max_vel_);
            }
            else if (min_lim_wheel_vel < -max_vel_) 
            {
                right_wheel_vel_ = lim_right_wheel_vel - (min_lim_wheel_vel + max_vel_);
                left_wheel_vel_ = lim_left_wheel_vel - (min_lim_wheel_vel + max_vel_);
            }
            else
            {
                right_wheel_vel_ = lim_right_wheel_vel;
                left_wheel_vel_ = lim_left_wheel_vel;
            }      
        }
        
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
    };
};