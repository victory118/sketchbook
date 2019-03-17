#include "Arduino.h"

// Encoder parameters
const int kLeftEncClkPin = 2;
const int kLeftEncDirPin = 8;
const int kRightEncClkPin = 3;
const int kRightEncDirPin= 9;

// const float ENC_FACTOR = 4
const float kEncoderCpr = 1920.0; // encoder counts per revolution

// Left encoder variables
volatile int left_encoder_count = 0;
volatile boolean left_encoder_change_flag = false;

// Right encoder variables
volatile int right_encoder_count = 0;
volatile boolean right_encoder_change_flag = false;

void LeftEncoderInterrupt() {
  if (digitalRead(kLeftEncDirPin) == HIGH) {
    left_encoder_count++;
  }
  else {
    left_encoder_count--;
  }
  left_encoder_change_flag = true;
}

void RightEncoderInterrupt() {
  if (digitalRead(kRightEncDirPin) == LOW) {
    right_encoder_count++;
  }
  else {
    right_encoder_count--;
  }
  right_encoder_change_flag = true;
}

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
            ang_vel_cps_ = (curr_count_ - prev_count_) / sample_time_;
        }

        float get_ang_vel_cps() { return ang_vel_cps_; }
        float get_ang_vel_rps() { return ang_vel_cps_ / counts_per_rev_ * 2.0 * PI; }

    private:
        const int clk_pin_;
        const int dir_pin_;
        const float sample_time_; // time period between sampling encoder measurements
        
        const int counts_per_rev_; // encoder counts per revolution

        signed long curr_count_;
        signed long prev_count_;
        float ang_vel_cps_; // angular velocity [encoder counts/s]
    };
};
