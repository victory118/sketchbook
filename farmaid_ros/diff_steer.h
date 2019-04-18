#ifndef DIFF_STEER_H
#define DIFF_STEER_H

namespace Farmaid
{  

    struct DiffDriveWheelVel
    {
        // struct to hold left and right wheel velocity variables
        float left; // left wheel velocity [m/s]
        float right;// right wheel velocity [m/s]
    };
    
    class DiffSteer
    {
    public:
        DiffSteer(float wheelbase, float wheel_radius) : 
            wheelbase_(wheelbase), wheel_radius_(wheel_radius),
            max_vel_(0), max_ang_vel_(0), ensure_ang_vel_(true)
        {
        }
        
        /**
         * @brief Converts unicycle velocity commands to motor velocity commands
         * @param vel: commanded forward velocity in m/s
         * ang_vel: commanded angular velocity in m/s
         */
        virtual void Drive(float vel, float ang_vel) = 0;

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
        
    protected:
        float wheelbase_;
        float wheel_radius_;

        float max_vel_; // maximum forward velocity of robot with no rotation [m/s]
        float max_ang_vel_; // maximum angular velocity of robot with pure rotation [rad/s]

        bool ensure_ang_vel_;
    };
};

#endif
