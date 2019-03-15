

// Sampling rates
const float CONTROL_FREQ = 100; // Hz
const float CONTROL_PERIOD = 1.0 / CONTROL_FREQ; // sec
const float CONTROL_PERIOD_MICROS = CONTROL_PERIOD * 1e6; // microsec
unsigned long prevControlTime = 0;

const float ROS_FREQ = 50; // Hz
const float ROS_PERIOD = 1.0 / ROS_FREQ; // sec
const float ROS_PERIOD_MICROS = ROS_PERIOD * 1e6; // microsec
unsigned long prevRosTime = 0;

// Robot parameters
const float WHEELBASE = 0.5; // meters
const float WHEEL_RADIUS = 0.1; // meters

// Motor parameters
const int MOT_PWM_PIN_L = 6;
const int MOT_DIR_PIN_L = 7;
const int MOT_PWM_PIN_R = 5;
const int MOT_DIR_PIN_R = 4;
const float MAX_MOTOR_SPEED = 20; // use the minimum of the two max motor speeds [rad/sec]

namespace Farmaid
{
    class Robot
    {
    public:
        Robot()
        : leftMotor(MOT_PWM_PIN_L, MOT_DIR_PIN_L), rightMotor(MOT_PWM_PIN_R, MOT_DIR_PIN_R),
          leftEncoder(ENC_CLK_PIN_L, DIR_PIN_L), rightEncoder(ENC_CLK_PIN_R, ENC_DIR_PIN_R),
          diffDrive(WHEELBASE, WHEEL_RADIUS, MAX_MOTOR_SPEED)
        {
            leftMotorPid = PidController(Kp, Ki, Kd, Kf);
            rightMotorPid = PidController(Kp, Ki, Kd, Kf);
        }

    void execute(float desVel, float desAngVel)
    {

        // Calculates and updates the current speed and odometry based on encoder measurements
        leftEncoder.updateMeas();
        rightEncoder.updateMeas();

        // Calculate current linear wheel velocities
        currLeftWheelVel = leftEncoder.getWheelVel(WHEEL_RADIUS);
        currRightWheelVel = rightEncoder.getWheelVel(WHEEL_RADIUS);

        // Calculate the desired left and right wheel velocities
        diffDrive.computeDesWheelVel(desVel, desAngVel);
        desLeftWheelVel = diffDrive.getDesLeftWheelVel();
        desRightWheelVel = diffDrive.getDesRightWheelVel();

        // Compute the controller commands based on desired and current wheel velocities
        leftMotorCommand = leftMotorPid.compute(desLeftWheelVel, currLeftWheelVel);
        rightMotorCommand = rightMotorPid.compute(desLeftWheelVel, currRightWheelVel);

        leftMotor.setSpeed(leftMotorCommand);
        rightMotor.setSpeed(rightMotorCommand);
    }
        
    private:
        Motor leftMotor;
        Motor rightMotor;
        
        PidController leftMotorPid;
        PidController rightMotorPid;
        
        Encoder leftEncoder;
        Encoder rightEncoder;

        DifferentialDrive diffDrive;
    };
};

void setup() {
  // put your setup code here, to run once:

  // Attach encoder interrupts
  attachInterrupt(0, encoderIntL, RISING);
  attachInterrupt(1, encoderIntR, RISING);
  
}

void loop() {
  // put your main code here, to run repeatedly:

  currentTime = millis();

  if (currentTime - prevRosTime) >= ROS_PERIOD_MICROS:
  {
      nh.spinOnce();
      prevRosTime = currentTime;
  }
  
  if (currentTime - prevControlTime) >= CONTROL_PERIOD_MICROS:
  {
      // Get the desired velocity and angular velocity from the subscribed topic
      desVel = getDesVel();
      desAngVel = getDesAngVel();
      
      robot.execute(desVel, desAngVel));
      prevControlTime = currentTime;
  }

}
