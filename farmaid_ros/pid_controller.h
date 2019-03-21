#include <Arduino.h>

namespace Farmaid
{
    class PidController
    {
    public:
        PidController(float p_gain, float i_gain, float d_gain, float filt_const, float sample_time)
            : p_gain_(p_gain), i_gain_(i_gain), d_gain_(d_gain), filt_const_(filt_const), sample_time_(sample_time),
              prev_filt_error_(0),
              int_error_(0),
              p_control_(0), i_control_(0), d_control_(0), control_command_(0)
        {
        }

        float ComputeCommand(float des, float curr)
        {
            float error = des - curr;
            float filt_error = (1 - filt_const_) * prev_filt_error_ + filt_const_ * error;
            int_error_ = int_error_ + error * sample_time_;
            
            p_control_ = p_gain_ * error;
            i_control_ = i_gain_ * int_error_;
            d_control_ = d_gain_ * (filt_error - prev_filt_error_) / sample_time_;

            control_command_ = p_control_ + i_control_ + d_control_;

            prev_filt_error_ = filt_error;

            return control_command_;
        }

        // TODO: Anti-integrator windup
        // TODO: Feedforward

        void set_int_error(float int_error) { int_error_ = int_error; }

        void set_prev_filt_error(float prev_filt_error) { prev_filt_error_ = prev_filt_error; }

        void Reset()
        {
            set_int_error(0);
            set_prev_filt_error(0);
        }

        void set_p_gain(float p_gain) { p_gain_ = p_gain; }
        void set_i_gain(float i_gain) { i_gain_ = i_gain; }
        void set_d_gain(float d_gain) { d_gain_ = d_gain; }
        void set_filt_const(float filt_const) { filt_const_ = filt_const; }
        float get_p_gain() { return p_gain_; }
        float get_i_gain() { return i_gain_; }
        float get_d_gain() { return d_gain_; }
        float get_filt_const() { return filt_const_; }
        float get_p_control() { return p_control_; }
        float get_i_control() { return i_control_; }
        float get_d_control() { return d_control_; }
        float get_control_command() { return control_command_; }

    private:
        float p_gain_; // proportional gain
        float i_gain_; // integral gain
        float d_gain_; // derivative gain
        float filt_const_; // first order low-pass filter constant for derivative error 
        // 0: pass-through, 0-1: low-pass, 1: one time-step delay
        // filt_const = Ts / (Ts + Tf) - typically around 0.1
        // higher filt_const -> lower Tf -> higher cut-off freq. -> more filtering
        
        float sample_time_; // [sec]
        
        float prev_filt_error_; // filtered error in previous time step
        float int_error_; // integral error

        // Debug
        float p_control_; // proportional component of control command
        float i_control_; // integral component of control command
        float d_control_; // derivative component of control command
        float control_command_; // total control command
    };
};
