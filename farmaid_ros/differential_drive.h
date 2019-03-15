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
        DifferentialDrive(float wheelbase_meters, float wheelRadius_meters, float maxMotorSpeed_rps)
            : wheelbase(wheelbase_meters), wheelRadius(wheelRadius_meters), maxMotorSpeed(maxMotorSpeed_rps)
        {
            // maxVel - max linear velocity of robot when it has zero angular velocity (same as max wheel speed)
            // maxAngVel - max angular velocity of robot when it has zero linear velocity
            maxVel = maxMotorSpeed_rps * wheelRadius_meters; // [m/s] = omega [rad/s] * (2*pi*r)/(2*pi) [m/rad]
            maxAngVel = 2.0 * maxWheelSpeed / wheelbase_meters
        }
      
        float computeDesWheelVel(float desVel, float desAngVel)
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

        float getLeftDesWheelVel( return leftDesWheelVel );
        float getRightDesWheelVel( return rightDesWheelVel );
        
    private:
        float wheelbase
        float wheelRadius
        float maxMotorSpeed
        float maxVel
        float maxAngVel

        float leftDesWheelVel // desired left wheel linear velocity [m/s]
        float rightDesWheelVel // desired right wheel linear velocity [m/s]
    };
};
