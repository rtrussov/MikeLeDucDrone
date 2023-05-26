//////////////////////////////////
// Gyro Class
//////////////////////////////////

// c 2015 IowaKAPer / Mike LeDuc

Gyro::Gyro()    // class constructor
{
    rate = 0.0;      // deg/sed, current rate
    max = 0.0;       // deg/sec maximum recorded gyro rate since last power up
    max_chg = 0.0;   // deg/sec maximum difference between samples (used to determine noise in system)
    num_samples = 0;  // number of samples used in running average calculation
    accum = 0.0;        // accumulated sum of all rate values (used to calculate average rate)
}

void Gyro::update(float raw_rate, float offset, float gain_adj)     // process MPU gyro data
{
    rate_last = rate;                               // save rate from last iteration
    rate = (((raw_rate  / 32768.0) * GYRO_FS) - offset) * gain_adj;    // compute new rate

    if (num_samples == 0) rate_last = rate;         // initialize rate last for first iteration frame

    delta = (rate + rate_last) / 2.0 * ITER_PERIOD;   // determine gyro angle delta since last iteration

    if ((fabs)(rate) > max) max = rate;             // keep track of maximum gyro rate

    if ((fabs)(rate - rate_last) > max_chg) max_chg = rate - rate_last; // record max change between samples
                                                                        // to determine if system is noisy)
     
    // calculate running average of rate with and without offset compensation (scaled to deg/sec).  Used to calculate offsets
    if(num_samples == 0)  
    {
        raw_accum = 0.0;
        accum = 0.0;
    }
    num_samples++;
    raw_accum += raw_rate / 32768.0 * GYRO_FS;
    accum += rate;
    if(num_samples == 500)
    {
        raw_average = raw_accum / 500;
        average = accum / 500;
        num_samples = 0;
    }
}
