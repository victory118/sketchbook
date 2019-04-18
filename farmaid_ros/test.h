#ifndef TEST_H
#define TEST_H

extern unsigned long curr_millis;
extern unsigned long prev_serial_millis;
extern unsigned long prev_control_millis;
extern const unsigned long serial_period;
extern const unsigned long control_period;

namespace Farmaid
{
void TestEncoderClass(Encoder &left_encoder, Encoder &right_encoder)
{

    if (curr_millis - prev_serial_millis >= serial_period)
    {   
            
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

        prev_serial_millis = curr_millis;
    }        
}

void TestMotorOpenLoop(Motor &motor, float factor)
{

    float command = 0;

    if (curr_millis - prev_control_millis >= control_period)
    {
        if (curr_millis < 2000)
        {
            command = 0 * factor;   
        }
        else if (curr_millis < 4000)
        {
            command = 0.5 * factor;
        }
        else if (curr_millis < 6000)
        {
            command = 1.0 * factor;
        }
        else if (curr_millis < 8000)
        {
            command = 0.5 * factor;
        }
        else
        {
            command = 0.0 * factor;
        }

        motor.set_command(command);
        prev_control_millis = curr_millis;
        Serial.println(motor.get_command() / 255.0);
    }
}

void TestMotorPositionControl(Motor &motor, Encoder &encoder, PidController &pid, float factor)
{

    float setpoint = 0;
    float max_range = encoder.get_counts_per_rev();
    float command = 0;
    float curr_pos = 0;

    if (curr_millis - prev_control_millis >= control_period)
    {
        if (curr_millis < 4000)
        {
            setpoint = 0 * factor;   
        }
        else if (curr_millis < 8000)
        {
            setpoint = 0.5 * factor;
        }
        else if (curr_millis < 12000)
        {
            setpoint = 1.0 * factor;
        }
        else if (curr_millis < 16000)
        {
            setpoint = 0.5 * factor;
        }
        else
        {
            setpoint = 0.0 * factor;
        }

        // Process encoder measurement
        encoder.ProcessMeasurement();

        // Get the current motor position
        curr_pos = encoder.get_count() / max_range;

        // Compute controller command based on desired and current position
        command = pid.ComputeCommand(setpoint, curr_pos);

        // Send controller command to motor
        motor.set_command(command);

        prev_control_millis = curr_millis;

//        Serial.print(pid.get_p_gain());
//        Serial.print(" ");
//        Serial.print(pid.get_p_control());
//        Serial.print(" ");
//        Serial.print(pid.get_error());
//        Serial.print(" ");
//        Serial.print(pid.get_command());
//        Serial.print(" ");
//        Serial.print(command);
//        Serial.print(" ");
        Serial.print(2.0);
        Serial.print(" ");
        Serial.print(-2.0);
        Serial.print(" ");
        Serial.print(setpoint);
        Serial.print(" ");
        Serial.println(curr_pos);
    }
}

void TestMotorVelocityStep(Motor &motor, Encoder &encoder, PidController &pid, float dir)
{

    float setpoint = 0;
    float max_range = motor.get_no_load_rps();
    float command = 0;
    float curr_vel = 0;

    if (curr_millis - prev_control_millis >= control_period)
    {
        if (curr_millis < 5000)
        {
            setpoint = 0;   
        }
        else if (curr_millis < 10000)

        {
            setpoint = 0.4 * dir;
        }
        else if (curr_millis < 15000)
        {
            setpoint = 0.8 * dir;
        }
        else if (curr_millis < 20000)
        {
            setpoint = 0.4 * dir;
        }
        else
        {
            setpoint = 0;
        }

        // Process encoder measurement
        encoder.ProcessMeasurement();

        // Get the current motor vel
        curr_vel = encoder.get_filt_vel_rps() / max_range;

        // Compute controller command based on desired and current position
        command = pid.ComputeCommand(setpoint, curr_vel);

        // Send controller command to motor
        motor.set_command(command);

        prev_control_millis = curr_millis;

        Serial.print(2.0);
        Serial.print(" ");
        Serial.print(-2.0);
        Serial.print(" ");
        Serial.print(setpoint);
        Serial.print(" ");
        Serial.println(curr_vel);
    }
}

void TestMotorVelocitySine(Motor &motor, Encoder &encoder, PidController &pid, float dir)
{

    float sin_freq = 0.5; // (Hz)
    float max_range = motor.get_no_load_rps();
    float setpoint = dir * 0.5 * sin(2.0 * PI * sin_freq * curr_millis / 1000.0);
    float command = 0;
    float curr_vel = 0;

    if (curr_millis - prev_control_millis >= control_period)
    {
        if (curr_millis < 4000)
        {
            setpoint = 0;   
        }
        else if (curr_millis > 12000)
        {
            setpoint = 0;
        }

        // Process encoder measurement
        encoder.ProcessMeasurement();

        // Get the current motor vel
        curr_vel = encoder.get_filt_vel_rps() / max_range;

        // Compute controller command based on desired and current position
        command = pid.ComputeCommand(setpoint, curr_vel);

        // Send controller command to motor
        motor.set_command(command);

        prev_control_millis = curr_millis;

        Serial.print(2.0);
        Serial.print(" ");
        Serial.print(-2.0);
        Serial.print(" ");
        Serial.print(setpoint);
        Serial.print(" ");
        Serial.println(curr_vel);
    }
}

void TestMaxSpeed(Motor &motor, Encoder &encoder, float dir)
{

    if (curr_millis - prev_control_millis >= control_period)
    {
        encoder.ProcessMeasurement();
        prev_control_millis = curr_millis;
    }
    
    if (curr_millis - prev_serial_millis >= serial_period)
    {
        if (curr_millis < 5000)
        {
            motor.set_command(dir);
            Serial.print("Velocity (cps) = ");
            Serial.println(encoder.get_vel_cps());
            Serial.print("Velocity (rps) = ");
            Serial.println(encoder.get_vel_rps());
        }
        else
        {
            motor.set_command(0.0);
        }

        prev_serial_millis = curr_millis;
    }
}

void TestDiffSteerForward(DiffSteer &robot)
{
    if (curr_millis - prev_control_millis >= control_period)
    {
        if (curr_millis < 4000) { robot.Drive(0, 0); }
        else if (curr_millis < 6000) { robot.Drive(robot.get_max_vel()/4, 0); } // Forward
        else if (curr_millis < 8000) { robot.Drive(robot.get_max_vel()/2, 0); } // Forward
        else if (curr_millis < 10000) { robot.Drive(robot.get_max_vel()/4, 0); } // Forward
        else if (curr_millis < 11000) { robot.Drive(0, 0); } // Stop
        else if (curr_millis < 13000) { robot.Drive(-robot.get_max_vel()/4, 0); } // Backward
        else if (curr_millis < 15000) { robot.Drive(-robot.get_max_vel()/2, 0); } // Backward
        else if (curr_millis < 17000) { robot.Drive(-robot.get_max_vel()/4, 0); } // Backward
        else { robot.Drive(0, 0); }
    }
}

void TestDiffSteerRotate(DiffSteer &robot)
{
    if (curr_millis - prev_control_millis >= control_period)
    {
        if (curr_millis < 4000) { robot.Drive(0, 0); }
        else if (curr_millis < 6000) { robot.Drive(0, robot.get_max_ang_vel()/4); } // Forward
        else if (curr_millis < 8000) { robot.Drive(0, robot.get_max_ang_vel()/2); } // Forward
        else if (curr_millis < 10000) { robot.Drive(0, robot.get_max_ang_vel()/4); } // Forward
        else if (curr_millis < 11000) { robot.Drive(0, 0); } // Stop
        else if (curr_millis < 13000) { robot.Drive(0, -robot.get_max_ang_vel()/4); } // Backward
        else if (curr_millis < 15000) { robot.Drive(0, -robot.get_max_ang_vel()/2); } // Backward
        else if (curr_millis < 17000) { robot.Drive(0, -robot.get_max_ang_vel()/4); } // Backward
        else { robot.Drive(0, 0); }
    }
}

void TestDiffSteerCircle(DiffSteer &robot, float radius, float dir)
{
    float vel = robot.get_max_vel()/3;
    float ang_vel = vel / radius * dir;
    if (curr_millis - prev_control_millis >= control_period)
    {
        if (curr_millis < 4000) { robot.Drive(0, 0); }
        else if (curr_millis < 16000) { robot.Drive(vel, ang_vel); }
        else { robot.Drive(0, 0); }
    }
}

}


#endif
