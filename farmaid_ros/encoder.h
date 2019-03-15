#include "Arduino.h"

// Encoder parameters
const int kLeftEncClkPin = 2;
const int kLeftEncDirPin = 8;
const int kRightEncClkPin = 3;
const int kRightEncDirPin= 9;

// const float ENC_FACTOR = 4
const float COUNTS_PER_REV = 1920.0;

// Left encoder variables
volatile int encoderCountL = 0;
volatile boolean changeFlagL = false;
volatile int countResetL = 0;

// Right encoder variables
volatile int encoderCountR = 0;
volatile boolean changeFlagR = false;
volatile int countResetR = 0;

void encoderIntL() {
  if (digitalRead(ENC_DIR_PIN_L) == HIGH) {
    encoderCountL++;
    countResetL++;
  }
  else {
    encoderCountL--;
    countResetL--;
  }
  changeFlagL = true;
}

void encoderIntR() {
  if (digitalRead(ENC_DIR_PIN_R) == LOW) {
    encoderCountR++;
    countResetR++;
  }
  else {
    encoderCountR--;
    countResetR--;
  }
  changeFlagR = true;
}

namespace Farmaid
{
    class Encoder
    {
    public:
        Encoder(int clkPin, int dirPin)
            : clkPin(clkPin), dirPin(dirPin),
            prevEncCount(0), dTheta(0)
        {
            pinMode(clkPin, INPUT);
            pinMode(dirPin, INPUT);
        }

        void updateCount()
        {
            
        }

    private:
        const float revToEnc = 1920.0; // encoder counts per revolution
        const float encToRev = 1.0 / revToEnc; // revolutions per encoder count
        const float encToRad = encToRev * 2 * PI; // encoder counts to radians

        signed long prevEncCount;
        float dTheta;
    };
};
