/**
 *  @file differential_drive.h
 *  @brief Converts linear and angular velocity commands to left and right motor commands
 *  @author Victor Yu (based on ROSBots by Jack Pien)
 */

namespace Farmaid
{
    class DifferentialDrive
    {
    public:
        /*
         * @brief Class constructor
         */
        DifferentialDrive(float wheelbase, float wheel_radius, float max_motor_speed_rps)
            : wheelbase_(wheelbase),
              wheel_radius_(wheel_radius),
              max_motor_speed_rps_(max_motor_speed_rps),
              max_vel_(max_motor_speed_rps * wheel_radius),
              max_ang_vel_(max_vel_ / (wheelbase / 2.0)),
              ensure_ang_vel_(true),
              left_wheel_vel_(0), right_wheel_vel_(0)
        {
        }
      
        void MapUniToDiff(float vel, float ang_vel)
        {
            // This function ensures that ang_vel is respected as best as possible
            // by scaling vel.
            // vel - desired robot linear velocity [m/s]
            // ang_vel - desired robot angular velocity [rad/s]

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

        float AngVelToWheelVel(float ang_vel) { return ang_vel * wheel_radius_; }
        float get_left_wheel_vel() { return left_wheel_vel_; }
        float get_right_wheel_vel() { return right_wheel_vel_; }
        bool get_ensure_ang_vel() { return ensure_ang_vel_; }
        void set_ensure_ang_vel(bool ensure_ang_vel) { ensure_ang_vel_ = ensure_ang_vel; }
        
    private:
        const float wheelbase_; // [m]
        const float wheel_radius_; // [m]
        const float max_motor_speed_rps_; // maximum motor speed [rad/s]
        const float max_vel_; // maximum forward velocity in robot frame is when it has zero angular velocity (same as max wheel speed) [m/s]
        const float max_ang_vel_; // maximum angular velocity in robot frame is when it has zero linear velocity (pure rotation) [rad/s]
        bool ensure_ang_vel_; // flag to ensure that ang_vel is respected as best as possible by scaling vel

        float left_wheel_vel_; // left wheel velocity calculated from desired forward velocity [m/s]
        float right_wheel_vel_; // right wheel velocity calculated from desired angular velocity [m/s]
    };
};
