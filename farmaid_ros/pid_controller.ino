#include "Arduino.h"

namespace Farmaid
{
    class PidController
    {
    public:
        PidController(float pGain, float iGain, float dGain, float sampleTime)
            : Kp(pGain), Ki(iGain), Kd(dGain), Ts(sampleTime)
            prevEncCount(0), dTheta(0.0)
        {
            prevFiltError = 0; // filtered error in previous time step
            intError = 0; // integral error
        }

        float compute(des, curr)
        {
            error = des - curr;
            filtError = (1 - Kf) * prevFiltError + Kf * error;
            
            pControl = Kp * error;
            iControl = Ki * intError + error * Ts;
            dControl = Kd * (filtError - prevFiltError) / Ts;
        }

    private:
        float Kp; // proportional gain
        float Ki; // integral gain
        float Kd; // derivative gain
        float Kf; // first order low-pass filter constant
        // Kf = Ts / (Ts + Tf) - typically around 0.1
        
        float Ts; // sample time
        
        float prevFiltError;

        // Debug
        float pControl; // proportional component of control input
    };
};
