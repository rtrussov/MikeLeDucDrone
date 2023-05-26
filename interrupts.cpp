//////////////////////////////////////////
// Interrupt Handlers
//////////////////////////////////////////

// c 2015 IowaKAPer / Mike LeDuc

#ifdef __cplusplus  // interrupt handler must be in c language (not c++)
extern "C" {
#endif

void __ISR(_TIMER_1_VECTOR, ipl2auto) _Timer1Handler(void)  // interrupt routine for timer T1 (frame rate timer)
{
    ClearWDT();     //clear watchdog timer at iteration rate (WDT timeout set for 8 msec)

    if(platform_initialized)
    {
        if(absolute_time > 2.0)
        {
            update_MPU_sensors();   // get MPU i2c sensor data and process it (500 Hz update rate)
            //get_tether_tension();
            update_baro_data();       // get temperature compensated baro data (50 Hz update rate)
            if(!rx_override)
            {
                get_rx_cmds();
                set_modes();
            }
        }
        if(absolute_time > 3.0)   // delay to let sensors and rx data settle before using data
        {
            compute_altitude();
            compute_platform_attitude();    // perform sensor fusion to compute platform attitude
            compute_tether_attitude();      // perform sensor fusion to compute tether attitude
            compute_platform_acceleration();
        }
        if(absolute_time > 4.0)
        {
            tether_hold_control_laws();
            xyz_damper_control_laws();
            primary_control_laws();
        }
        if(absolute_time > 6.0)
        {
            drive_motors();  // delay motor drive until control law data stabilizes and MPU sensors are averaged
            drive_servos();
        }
        if(datalogging_enabled) 
        {
            datalog_flash();
            //datalog_uart();
        }
        if(get_sensor_offsets_enable) get_sensor_offsets();
    }

    unit_status();  // blink unit status LED

    absolute_time = absolute_time + ITER_PERIOD;  // keep track of elapsed time in seconds

    frame_count++;  // increment frame counter
    if(frame_count == 11) frame_count = 1;   // set up multi-rate structure, cycles every 10 frames to create 50Hz sub frame

    if(absolute_time > 4.0) get_high_water();	// determine max elapsed time of interrupt routine to check for frame overrun
    mT1ClearIntFlag();  // clear timer interrupt flag
 }

#ifdef __cplusplus  // interrupt handler must be in c language (not c++)
}
#endif
