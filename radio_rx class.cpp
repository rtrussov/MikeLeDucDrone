//////////////////////////////////
// radio_rx Classes
//////////////////////////////////

// c 2015 IowaKAPer / Mike LeDuc

radio_rx::radio_rx()   // class constructor
{
    pulse_width = 0.0015;  // seconds, initialize to mid stick
    channel_active = false;
    pulse_start_time = 0;
    pulse_end_time = 0;
    timer_interval = 0.00;
    no_rx_data_count = 0;
}

void radio_rx::get_pw(unsigned char data_ready, int rx_FIFO)  // get pulse width from radio rx channel
{
    if(data_ready)
    {
        channel_active = true;
        no_rx_data_count = 0;  // reset counter since data has been recieved

        pulse_end_time = rx_FIFO;        // read start time from input capture module
        pulse_start_time_last = pulse_start_time;

        if((pulse_end_time - pulse_start_time) > 0) timer_interval = (pulse_end_time - pulse_start_time);
                                                        // pulse width = end time - start time if timer3 rollover has not occured
        else timer_interval = 0xFFFF + (pulse_end_time - pulse_start_time);  // if timer3 rollover occured during capture, then
                                                    // rising edge occured before roll over occured, calculate time between events
        timer_interval = timer_interval * T3_SEC_PER_BIT;

        if((timer_interval > RX_MIN_VALID_PULSE) & (timer_interval < RX_MAX_VALID_PULSE)) pulse_width = timer_interval; // a valid radio pulse has been received

        if(pulse_width < RX_MIN_PULSE_LIMIT) pulse_width = RX_MIN_PULSE_LIMIT;  // limit pulse width to min and max values
        if(pulse_width > RX_MAX_PULSE_LIMIT) pulse_width = RX_MAX_PULSE_LIMIT;

        pulse_start_time = pulse_end_time;   //  save start time for next interval calculation
    }
    else
    {
        no_rx_data_count++;    // no data received so increment counter
        if(no_rx_data_count > RX_ACTIVE_TIMEOUT)
        {
            channel_active = false;
        }
    }
}
