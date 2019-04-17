#ifndef UNO_2WD_HELPER_H
#define UNO_2WD_HELPER_H

extern unsigned long curr_millis;
extern unsigned long prev_serial_millis;
extern const unsigned long serial_period;

extern Farmaid::EncoderParams left_encoder_p;
extern Farmaid::EncoderParams right_encoder_p;

// Encoder counter variables are global because they use hardware interrupts
volatile int left_encoder_count = 0;
volatile bool left_encoder_change_flag = false;
volatile int right_encoder_count = 0;
volatile bool right_encoder_change_flag = false;

void ReadEncoders()
{

    if (curr_millis - prev_serial_millis >= serial_period)
    {
        // This reads directly from the global variables
        if (left_encoder_change_flag) {
            left_encoder_change_flag = false;
            Serial.print("Left encoder count = ");
            Serial.println(left_encoder_count);
        }

        if (right_encoder_change_flag) {
            right_encoder_change_flag = false;
            Serial.print("Right encoder count = ");
            Serial.println(right_encoder_count);
        }

        prev_serial_millis = curr_millis;
    }        
}

void LeftEncoderInterrupt() {
  if (digitalRead(left_encoder_p.dir_pin) == HIGH) {
    left_encoder_count++;
  }
  else {
    left_encoder_count--;
  }
  left_encoder_change_flag = true;
}

void RightEncoderInterrupt() {
  if (digitalRead(right_encoder_p.dir_pin) == LOW) {
    right_encoder_count++;
  }
  else {
    right_encoder_count--;
  }
  right_encoder_change_flag = true;
}

#endif
