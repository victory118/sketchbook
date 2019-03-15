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
        DifferentialDrive(float wheelbase, float wheel_radius, float maxMotorSpeed_rps)
            : wheelbase(wheelbase_meters), wheelRadius(wheelRadius_meters), maxMotorSpeed(maxMotorSpeed_rps)
        {
            // maxVel - max linear velocity of robot when it has zero angular velocity (same as max wheel speed)
            // maxAngVel - max angular velocity of robot when it has zero linear velocity
            maxVel = maxMotorSpeed_rps * wheelRadius_meters; // [m/s] = omega [rad/s] * (2*pi*r)/(2*pi) [m/rad]
            maxAngVel = 2.0 * maxWheelSpeed / wheelbase_meters
        }
      
        float UpdateDesWheelVel(float des_vel, float des_ang_vel)
        {
            // v - desired robot linear velocity [m/s]
            // w - desired robot angular velocity [rad/s]

            // Limit v, w from controller to +/- of their max
            desAngVelLim = max(min(desAngVel, maxAngVel), -maxAngVel);
            desVelLim = max(min(desVel, maxVel), -maxVel);

            // 2. Compute desired vel_r, vel_l needed to ensure new desired angular velocity
            rightDesWheelVel = (desiredVel + desiredAngVel*b);
            leftDesWheelVel = (desiredVel - desiredAngVel*b);
            
            rightWheelVel = ((2.0 * v) + (w * wheelbase)) / (2.0 * wheelRadius);

            // TODO: Finish implementation here
            
            return rightWheelVel;
        }

        float uni_to_diff_left(float v, float w)
        {
            // v - desired robot forward velocity [m/s]
            // w - desired robot angular velocity [rad/s]
            // leftWheelVel - desired left wheel velocity [rad/s]
            
            leftWheelVel = ((2.0 * v) - (w * wheelbase)) / (2.0 * wheelRadius);
            
            return leftWheelVel;
        }

        float get_left_des_wheel_vel_( return leftDesWheelVel );
        float get_right_des_wheel_vell( return rightDesWheelVel );
        
    private:
        const float kWheelbase_;
        const float kWheelRadius_;
        const float kMaxMotorSpeed_;
        const float kMaxVel_; // can this be set in the constructor?
        const float kMaxAngVel_; // can this be set in the constructor?

        float left_des_wheel_vel_; // desired left wheel linear velocity [m/s]
        float right_des_wheel_vel_; // desired right wheel linear velocity [m/s]
    };
};
