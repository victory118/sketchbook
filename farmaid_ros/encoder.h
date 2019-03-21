#include <Arduino.h>

namespace Farmaid
{

    struct EncoderParams
    {
        byte clk_pin;
        byte dir_pin;
        float counts_per_rev; // encoder counts per revolution
        float sample_period; // time period between sampling encoder measurements [sec]
    }
    
    class Encoder
    {
    public:
        Encoder(EncoderParams p, int *count_ptr)
            : clk_pin_(p.clk_pin), dir_pin_(p.dir_pin),
              counts_per_rev_(p.counts_per_rev),
              sample_period_(p.sample_period),
              count_ptr_(count_ptr)
              count_(0),
              prev_count_(0),
              vel_cps_(0)                        
        {
            pinMode(clk_pin_, INPUT);
            pinMode(dir_pin_, INPUT);
        }

        void ProcessMeasurement()
        {
            prev_count_ = count_;
            count_ = *count_ptr;
            vel_cps_ = (count_ - prev_count_) / sample_period_;
        }

        void Reset()
        {
            prev_count_ = 0;
            count_ = 0;
            vel_cps_ = 0;
        }

        float get_vel_cps() { return vel_cps_; }
        float get_vel_rps() { return vel_cps_ / counts_per_rev_ * 2.0 * PI; }
        signed long get_count() { return count_; }
        signed long get_prev_count() { return prev_count_; }

    private:
        const byte clk_pin_;
        const byte dir_pin_;
        const float sample_period_; // time period between sampling encoder measurements [sec]
        
        const float counts_per_rev_; // encoder counts per revolution

        signed long count_;
        signed long prev_count_;
        float vel_cps_; // velocity [encoder counts/s]

        int *count_ptr; // pointer to the global variable that holds the encoder counts
    };
};

void ReadEncoders()
{

    if (curr_millis - prev_print_millis >= print_period)
    {
        // This reads directly from the global variables
//        if (left_encoder_change_flag) {
//            left_encoder_change_flag = false;
//            Serial.print("Left encoder count = ");
//            Serial.println(left_encoder_count);
//        }
//
//        if (right_encoder_change_flag) {
//            right_encoder_change_flag = false;
//            Serial.print("Right encoder count = ");
//            Serial.println(right_encoder_count);
//        }

        // This reads from the encoder objects
        
        left_encoder.ProcessMeasurement();
        right_encoder.ProcessMeasurement();

        if (left_encoder.get_count() != left_encoder.get_prev_count())
        {
            Serial.print("Left encoder count (prev) = ");
            Serial.println(left_encoder.get_prev_count());
            Serial.print("Left encoder count = ");
            Serial.println(left_encoder.get_count());
        }

        if (right_encoder.get_count() != right_encoder.get_prev_count())
        {
            Serial.print("Right encoder count (prev) = ");
            Serial.println(right_encoder.get_prev_count());
            Serial.print("Right encoder count = ");
            Serial.println(right_encoder.get_count());
        }

        prev_print_millis = curr_millis;
    }        
}
