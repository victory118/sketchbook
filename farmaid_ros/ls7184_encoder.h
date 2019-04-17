/**
 * @file ls7184_encoder.h
 * @brief Quadrature encoder counter driver for the LSI-LS7184
 * @author Victor Yu
 */

#ifndef LS7184_ENCODER_H
#define LS7184_ENCODER_H

#include <Arduino.h>

namespace Farmaid
{

    struct EncoderParams
    {
        byte clk_pin;
        byte dir_pin;
        float counts_per_rev; // encoder counts per revolution
        float sample_period; // time period between sampling encoder measurements [sec]
        float filt_tc_;
    };
    
    class Encoder
    {
    public:
        Encoder(EncoderParams p, volatile int *count_ptr)
            : clk_pin_(p.clk_pin), dir_pin_(p.dir_pin),
              counts_per_rev_(p.counts_per_rev),
              sample_period_(p.sample_period),
              filt_tc_(p.filt_tc_),
              count_ptr_(count_ptr),
              count_(0), prev_count_(0),
              vel_cps_(0), filt_vel_cps_(0), prev_filt_vel_cps_(0)                        
        {
            pinMode(clk_pin_, INPUT);
            pinMode(dir_pin_, INPUT);
        }

        void ProcessMeasurement()
        {
            prev_count_ = count_;
            count_ = *count_ptr_;
            vel_cps_ = (count_ - prev_count_) / sample_period_;

            float alpha = sample_period_ / (sample_period_ + filt_tc_);
            filt_vel_cps_ = (1 - alpha) * prev_filt_vel_cps_ + alpha * vel_cps_; 
        }

        void Reset()
        {
            prev_count_ = 0;
            count_ = 0;
            vel_cps_ = 0;
        }

        float get_vel_cps() { return vel_cps_; }
        float get_filt_vel_cps() { return filt_vel_cps_; }
        float get_vel_rps() { return vel_cps_ / counts_per_rev_ * 2.0 * PI; }
        float get_filt_vel_rps() { return filt_vel_cps_ / counts_per_rev_ * 2.0 * PI; }
        signed long get_count() { return count_; }
        float get_pos_rad() { return count_ / counts_per_rev_ * 2.0 * PI; }
        signed long get_prev_count() { return prev_count_; }
        float get_counts_per_rev() { return counts_per_rev_; }

    private:
        const byte clk_pin_;
        const byte dir_pin_;
        const float sample_period_; // time period between sampling encoder measurements [sec]
        
        const float counts_per_rev_; // encoder counts per revolution

        const float filt_tc_; // first order low-pass filter time constant

        signed long count_;
        signed long prev_count_;
        float vel_cps_; // velocity [encoder counts/s]
        float filt_vel_cps_; // filtered velocity [encoder counts/s]
        float prev_filt_vel_cps_;

        volatile int *count_ptr_; // pointer to the global variable that holds the encoder counts
    };
};

#endif
