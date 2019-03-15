#include "Arduino.h"

namespace Farmaid
{
    class PidController
    {
    public:
        PidController(float pGain, float iGain, float dGain, float filtConst, float sampleTime)
            : Kp(pGain), Ki(iGain), Kd(dGain), Kf(filtConst), Ts(sampleTime)
        {
            prevFiltError = 0; // filtered error in previous time step
            intError = 0; // integral error
        }

        float compute(des, curr)
        {
            error = des - curr;
            filtError = (1 - Kf) * prevFiltError + Kf * error;
            intError = intError + error * Ts;
            
            pControl = Kp * error;
            iControl = Ki * intError;
            dControl = Kd * (filtError - prevFiltError) / Ts;

            controlCommand = pControl + iControl + dControl;

            prevFiltError = filtError;

            return controlCommand;
        }

        void resetIntError()
        {
            intError = 0;
        }

        void resetFiltError()
        {
            prevFiltError = 0;
        }

        void reset()
        {
            resetIntError();
            resetFiltError();
        }

        float getPGain() { return Kp; };
        float getIGain() { return Ki; };
        float getDGain() { return Kd; };
        float getFiltConst() { return Kf; };
        float getPControl() { return pControl; };
        float getIControl() { return iControl; };
        float getDControl() { return dControl; };

    private:
        float Kp; // proportional gain
        float Ki; // integral gain
        float Kd; // derivative gain
        float Kf; // first order low-pass filter constant
        // Kf = Ts / (Ts + Tf) - typically around 0.1
        // higher Kf -> lower Tf -> higher cut-off freq. -> more filtering
        
        float Ts; // sample time [sec]
        
        float prevFiltError;
        float intError;

        // Debug
        float pControl; // proportional component of control input
        float iControl; // integral component of control input
        float dControl; // derivative component of control input
    };
};
