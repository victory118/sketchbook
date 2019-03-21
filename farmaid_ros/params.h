namespace Farmaid
{
    struct EncoderParams
    {
        byte clk_pin;
        byte dir_pin;
        float counts_per_rev;
        unsigned int sample_period;
        int *count_ptr
    }

    struct MotorParams
    {
        byte pwm_pin;
        byte dir_pin;
        unsigned int max_command; //  0-255 for 0-100% PWM duty cycle
        float no_load_rps; // max speed with no load [rad/s]
    }

    struct PidParams
    {
        float p_gain;
        float i_gain;
        float d_gain;
        float filt_const;
        unsigned int sample_period;
    }

    struct MotorControllerParams
    {
        EncoderParams encoder_p;
        MotorParams motor_p;
        PidParams pid_p;
    }
    
    struct RobotParams
    {
        MotorControllerParams left_p;
        MotorControllerParams right_p;
        float wheelbase;
        float wheel_radius;
    }
}
