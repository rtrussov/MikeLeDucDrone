
//////////////////////////////////
// Rate Limit Class
//////////////////////////////////

// c 2015 IowaKAPer / Mike LeDuc

rate_limit::rate_limit()  // class constructor
{
    out = 0.0;
}

void rate_limit::compute(float in, float limit)     // compute rate limit of input.  output is limited each frame to delta limit value
{
    // AIRCON rate limiter (rlm)
    if((in - out) > limit) out = out + limit;
    else if((out - in) > limit) out = out - limit;
    else out = in;
}
