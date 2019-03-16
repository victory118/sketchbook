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
              sample_time_(sample_time)
        {
            pinMode(clk_pin_, INPUT);
            pinMode(dir_pin_, INPUT);

            prev_count_ = 0;
            delta_count_ = 0
        }

        void ProcessMeasurement()
        {
            
            
        }

    private:
        const int clk_pin_;
        const int dir_pin_;
        const float sample_time_; // time period between sampling encoder measurements
        
        const int rev_to_enc = 1920; // encoder counts per revolution
        const float enc_to_rev = 1.0 / rev_to_enc; // revolutions per encoder count
        const float enc_to_rad = enc_to_rev * 2 * PI; // radians per encoder count
        
        signed long prev_count_;
        int delta_count_;
    };
};
