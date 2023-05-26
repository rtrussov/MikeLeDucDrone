//////////////////////////////////
// Accel Class
//////////////////////////////////

// c 2015 IowaKAPer / Mike LeDuc

Accel::Accel()  // class constructor
{
    accel = 0.0;      // g, current accelerometer scaled in g's
    max = 0.0;        // g, maximum recorded accel g's since last power up
    max_chg = 0.0;    // g, maximum difference between samples (used to determine noise in system)
    num_samples = 0;  // number of samples used in running average calculation
    accum = 0.0;      // accumulated sum of all g values (used to calculate average g force)
}

void Accel::update(float raw_accel, float offset, float gain_adj)     // process MPU accelerometer data
{
    accel_last = accel; // save accelerometer value from last frame

    accel = (((raw_accel / 32768.0) * ACCEL_FS) - offset) * gain_adj;  // scale accelerometer output to g's  (1g = 32ft/sec/sec)

            // the inclinometer angle is calculated by arcsin(g) where "g" is accelerometer output scaled in g's.
            // when an angle (theta) is expressed in radians, the linear approximation
            // of arcsin(theta) is approx equal to theta for angles < 0.5 radians (~30 deg) (accurate within approx 5%)
            // therefore when the accelerometer is scaled in g's, the inclinometer angle
            // is approx equal to the accelerometer output for angles < 30 deg
            // for this application the accelerometer is used to compensate for long term gyro drift and wind angle bias
            // because both of these factors are relatively small in degrees, the linear approximation of arcsin provides good results
            // and the memory and thruput intensive "asin" math routine is not required.

    if (num_samples == 0) accel_last = accel;  // initialize accel_last for first iteration frame
    if ((fabs)(accel) > max) max = accel;  // keep track of maximum angle
    if ((fabs)(accel - accel_last) > max_chg) max_chg = accel - accel_last; // record max change between samples
                                                                            // to determine if system is noisy)

    // calculate running average of accel with and without offset compensation (scaled to g).  Used to calculate offsets
    if(num_samples == 0)
    {
        raw_accum = 0.0;
        accum = 0.0;
    }
    num_samples++;
    raw_accum += raw_accel / 32768.0 * ACCEL_FS;
    accum += accel;
    if(num_samples == 500)
    {
        raw_average = raw_accum / 500.0;
        average = accum / 500.0;
        num_samples = 0;
    }
    
    angle = accel * RAD_TO_DEG; // convert accel_output to angle in degrees,  angle approx. equals arcsin(g) for angles < 30 degrees
    //angle = asin(accel) * RAD_TO_DEG; // use this calculation for higher accuracy (thruput intensive)
}

