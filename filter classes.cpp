//////////////////////////////////
// filter Classes
//////////////////////////////////

// c 2015 IowaKAPer / Mike LeDuc

low_pass_filter::low_pass_filter()    // class constructor
{
    out = 0.0;
    out_last = 0.0;
    in_last = 0.0;
}

void low_pass_filter::initialize(float initial_value)    // set initial value of filter
                                                                              // time constant in seconds
{
    out = initial_value;
    out_last = initial_value;
    in_last = initial_value;
}

void low_pass_filter::compute(float in, float time_constant)     // compute low pass filter
{
    if(time_constant < ITER_PERIOD) out = in; // setting time_constant to a low value (i.e. 0) will bypass filter function
    else
    {
       //aircon low pass filter
       //hpf_out = alpha*(hpf_out.s+(in-in.s))
       //alpha = 1-(time_constant/(time_constant + ITER_PERIOD));        // filter constant
       //out_last = out;  // save previous output before computing new output
       //out = out_last + alpha * ((in + in_last)/2 - out_last);  //lpf_out = lpf_out.s + tau *(in+in.s)/2 - out.s)
       //in_last = in;

       // wiki low pass filter
       // hpf_out = alpha * x[i] + (1-alpha) * y[i-1]
       // alpha = dt / (RC + dt)
       alpha = ITER_PERIOD / (time_constant + ITER_PERIOD);
       out_last = out;
       out = (alpha * in) + ((1-alpha) * out_last);
    }
}

/*
void low_pass_filter::compute(float in, float time_constant)     // compute low pass filter
{
    //hpf_out = alpha*(hpf_out.s+(in-in.s))
    alpha = 1-(time_constant/(time_constant + ITER_PERIOD));        // filter constant
    out_last = out;  // save previous output before computing new output
    out = out_last + alpha * ((in + in_last)/2 - out_last);  //lpf_out = lpf_out.s + tau *(in+in.s)/2 - out.s)
    in_last = in;
}
*/
hi_pass_filter::hi_pass_filter()    // class constructor
{
    out = 0.0;
    out_last = 0.0;
    in_last = 0.0;
}

void hi_pass_filter::initialize(float initial_value)   // set initial value of filter
                                                                            // time constant in seconds
{
    out = initial_value;
    out_last = initial_value;
    in_last = initial_value;
}

void hi_pass_filter::compute(float in, float time_constant)     // compute low pass filter
{ 
    alpha = (time_constant/(time_constant + ITER_PERIOD));          // filter constant
    out_last = out;  // save previous output before computing new output
    out = alpha * (out_last + (in - in_last));  // hpf_out = alpha*(hpf_out.s+(in-in.s))
    in_last = in;
}