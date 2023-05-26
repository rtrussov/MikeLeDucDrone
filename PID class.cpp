//////////////////////////////////
// PID Classes
//////////////////////////////////

// c 2015 IowaKAPer / Mike LeDuc

PID::PID()   // class constructor
{
   fdbk_in_last = 0.0;
   error = 0.0;
   P = 0.0;
   I = 0.0;
   D = 0.0;
   out = 0.0;
}

void PID::reset(void)
{
    fdbk_in_last = 0.0;
    error = 0.0;
    P = 0.0;
    I = 0.0;
    D = 0.0;
    out = 0.0;
}

void PID::set_parameters(float kp, float ki, float kd, float i_lim)
{
    Kp=kp;
    Ki=ki * ITER_PERIOD;
    Kd=kd / ITER_PERIOD;
    Ilim = i_lim;
}

void PID::compute(float cmd_in, float fdbk_in) // PID controller
{
    error = cmd_in - fdbk_in;
    P = Kp * error;                      // proportional term
    I = I + (Ki * error);                // integral term
    D = Kd * (fdbk_in - fdbk_in_last);   // differntial term

    // limit integrator output
    if (I > Ilim) I = Ilim;
    else if (I < -Ilim) I = -Ilim;

    out = P + I - D;

    fdbk_in_last = fdbk_in;
}