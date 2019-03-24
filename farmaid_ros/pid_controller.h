#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

namespace Farmaid
{
    struct PidParams
    {
        float p_gain;
        float i_gain;
        float d_gain;
        float filt_tc;
        float sample_period;
    };
    
    class PidController
    {
    public:
        PidController(PidParams p)
            : p_gain_(p.p_gain), i_gain_(p.i_gain), d_gain_(p.d_gain), filt_tc_(p.filt_tc), sample_period_(p.sample_period),
              prev_filt_error_(0), filt_error_(0),
              int_error_(0),
              p_control_(0), i_control_(0), d_control_(0), command_(0), error_(0),
              max_deadband_(0), min_deadband_(),
              max_command_(1), antiwindup_(100)
        {
        }

        float ComputeCommand(float des, float curr)
        {
            
            // Calculate error
            error_ = des - curr;

            // Calculate current filtered error based on current measurement
            float alpha = sample_period_ / (sample_period_ + filt_tc_);
            filt_error_ = (1 - alpha) * prev_filt_error_ + alpha * error_;

            // Update integral error
            int_error_ = int_error_ + error_ * sample_period_;

            // Calculate control output for proportional, integral, and derivative components
            p_control_ = p_gain_ * error_;
            i_control_ = i_gain_ * int_error_;
            d_control_ = d_gain_ * (filt_error_ - prev_filt_error_) / sample_period_;

            // Sum up components
            command_ = p_control_ + i_control_ + d_control_;

            // Do integral reset/antiwindup
            IntegralErrorReset();

            // Set previous equal to current for next time step
            prev_filt_error_ = filt_error_;

            return command_;
        }

        void IntegralErrorReset()
        {
            if (command_ > antiwindup_ * max_command_)
            {
                command_ = max_command_;
                int_error_ = (antiwindup_ * max_command_ - p_control_ - d_control_) / i_gain_;
            }
            else if (command_ < -antiwindup_ * max_command_)
            {
                command_ = -max_command_;
                int_error_ = (-antiwindup_ * max_command_ - p_control_ - d_control_) / i_gain_;
            }   
        }

        // TODO: Feedforward

        void set_error(float error) { error_ = error; }
        void set_int_error(float int_error) { int_error_ = int_error; }
        void set_filt_error(float filt_error) { filt_error_ = filt_error; }
        void set_prev_filt_error(float prev_filt_error) { prev_filt_error_ = prev_filt_error; }

        void Reset()
        {
            set_int_error(0);
            set_prev_filt_error(0);
            set_filt_error(0);
        }

        void set_gains(float p_gain, float i_gain, float d_gain, float filt_tc)
        {
            p_gain_ = p_gain;
            i_gain_ = i_gain;
            d_gain_ = d_gain;
            filt_tc_ = filt_tc;
        }

        void set_p_gain(float p_gain) { p_gain_ = p_gain; }
        void set_i_gain(float i_gain) { i_gain_ = i_gain; }
        void set_d_gain(float d_gain) { d_gain_ = d_gain; }
        void set_filt_tc(float filt_tc) { filt_tc_ = filt_tc; }
        void set_max_deadband(float max_deadband) { max_deadband_ = max_deadband; }
        void set_min_deadband(float min_deadband) { min_deadband_ = min_deadband; }
        float get_p_gain() { return p_gain_; }
        float get_i_gain() { return i_gain_; }
        float get_d_gain() { return d_gain_; }
        float get_filt_tc() { return filt_tc_; }
        float get_p_control() { return p_control_; }
        float get_i_control() { return i_control_; }
        float get_d_control() { return d_control_; }
        float get_command() { return command_; }
        float get_error() { return error_; }

    private:
        float p_gain_; // proportional gain
        float i_gain_; // integral gain
        float d_gain_; // derivative gain
        float filt_tc_; // first order low-pass filter time constant for derivative error 
        // alpha = 0: one time step delay, 0-1: low-pass, 1: no filtering
        // alpha = Ts / (Ts + Tf) - typically around 0.1
        // Tf should be about 10*Ts
        // higher alpha -> lower Tf -> higher cut-off freq. -> less filtering
        float max_deadband_;
        float min_deadband_;
        
        float sample_period_; // [sec]

        float filt_error_; // filtered error
        float prev_filt_error_; // filtered error in previous time step
        float int_error_; // integral of the error multiplied by sample period

        // Debug
        float error_;
        float p_control_; // proportional component of control command
        float i_control_; // integral component of control command
        float d_control_; // derivative component of control command
        float command_; // total control command
        float max_command_; // absolute value of maximum command [0, 1]
        float antiwindup_; // usually set between 1.2 and 2; default = 100 will basically have no antiwindup effect
    };
};

#endif
