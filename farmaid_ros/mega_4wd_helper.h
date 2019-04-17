#ifndef MEGA_4WD_HELPER_H
#define MEGA_4WD_HELPER_H

extern Farmaid::EncoderParams fleft_encoder_p;
extern Farmaid::EncoderParams fright_encoder_p;
extern Farmaid::EncoderParams rleft_encoder_p;
extern Farmaid::EncoderParams rright_encoder_p;

// Encoder counter variables are global because they use hardware interrupts
volatile int fleft_encoder_count = 0;
volatile bool fleft_encoder_change_flag = false;
volatile int fright_encoder_count = 0;
volatile bool fright_encoder_change_flag = false;
volatile int rleft_encoder_count = 0;
volatile bool rleft_encoder_change_flag = false;
volatile int rright_encoder_count = 0;
volatile bool rright_encoder_change_flag = false;

void ReadEncoders()
{

    if (curr_millis - prev_serial_millis >= serial_period)
    {
        // This reads directly from the global variables
        if (fleft_encoder_change_flag) {
            fleft_encoder_change_flag = false;
            Serial.print("Front Left encoder count = ");
            Serial.println(fleft_encoder_count);
        }

        if (fright_encoder_change_flag) {
            fright_encoder_change_flag = false;
            Serial.print("Front Right encoder count = ");
            Serial.println(fright_encoder_count);
        }

        if (rleft_encoder_change_flag) {
            rleft_encoder_change_flag = false;
            Serial.print("Rear Left encoder count = ");
            Serial.println(rleft_encoder_count);
        }

        if (rright_encoder_change_flag) {
            rright_encoder_change_flag = false;
            Serial.print("Rear Right encoder count = ");
            Serial.println(rright_encoder_count);
        }

        prev_serial_millis = curr_millis;
    }        
}

void FLeftEncoderInterrupt() {
  if (digitalRead(fleft_encoder_p.dir_pin) == HIGH) {
    fleft_encoder_count++;
  }
  else {
    fleft_encoder_count--;
  }
  fleft_encoder_change_flag = true;
}

void FRightEncoderInterrupt() {
  if (digitalRead(fright_encoder_p.dir_pin) == LOW) {
    fright_encoder_count++;
  }
  else {
    fright_encoder_count--;
  }
  fright_encoder_change_flag = true;
}

void RLeftEncoderInterrupt() {
  if (digitalRead(rleft_encoder_p.dir_pin) == HIGH) {
    rleft_encoder_count++;
  }
  else {
    rleft_encoder_count--;
  }
  rleft_encoder_change_flag = true;
}

void RRightEncoderInterrupt() {
  if (digitalRead(rright_encoder_p.dir_pin) == LOW) {
    rright_encoder_count++;
  }
  else {
    rright_encoder_count--;
  }
  rright_encoder_change_flag = true;
}

#endif
