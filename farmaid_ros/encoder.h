#include <Arduino.h>

namespace Farmaid
{
    class Encoder
    {
    public:
        Encoder(int clk_pin, int dir_pin, float sample_time)
            : clk_pin_(clk_pin), dir_pin_(dir_pin),
              sample_time_(sample_time),
              counts_per_rev_(1920),
              curr_count_(0),
              prev_count_(0),
              ang_vel_cps_(0)                        
        {
            pinMode(clk_pin_, INPUT);
            pinMode(dir_pin_, INPUT);
        }

        void ProcessMeasurement(int encoder_count)
        {
            prev_count_ = curr_count_;
            curr_count_ = encoder_count;
            vel_cps_ = (curr_count_ - prev_count_) / sample_time_;
        }

        void Reset()
        {
            prev_count_ = 0;
            curr_count_ = 0;
            ang_vel_cps_ = 0;
        }

        float get_vel_cps() { return vel_cps_; }
        float get_vel_rps() { return vel_cps_ / counts_per_rev_ * 2.0 * PI; }
        signed long get_curr_count() { return curr_count_; }
        signed long get_prev_count() { return prev_count_; }

    private:
        const int clk_pin_;
        const int dir_pin_;
        const float sample_time_; // time period between sampling encoder measurements
        
        const int counts_per_rev_; // encoder counts per revolution

        signed long curr_count_;
        signed long prev_count_;
        float vel_cps_; // angular velocity [encoder counts/s]
    };
};
