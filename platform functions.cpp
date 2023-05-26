//////////////////////////////////
// Platform Functions
//////////////////////////////////

void configure_io(void)     // configure all io ports and peripherals
{
    // SYSTEMConfigPerformance(SYS_FREQ);     // use this when changing processors for optimum performance settings

    // configure timers
    OpenTimer1(T1_ON | T1_SOURCE_INT | T1_PS_1_1, T1_PRELOAD);  // t1 set for 20 Mhz clk for primary frame interrupt, rollover and interrupt at system iteration rate
    OpenTimer2(T2_ON | T2_PS_1_4, T2_PRELOAD);                  // t2 set for 5 Mhz clk, used for pwm outputs, rollover set to servo PWM period (used for pwm outputs)
    OpenTimer3(T3_ON | T3_PS_1_8, 0xFFFF);                      // t3 set for 2.5 Mhz clk, used for radio rx input capture and pwm servo control, rollover set to max period (26.2ms, 38Hz)
  
    // configure io ports
   
    mPORTASetPinsDigitalOut(BIT_0);   // Shutter Disc Output
    mPORTASetPinsDigitalIn(BIT_1);   // confibure port a input capture pin as input
    
    mPORTBSetPinsDigitalIn(BIT_2 | BIT_7 | BIT_10 | BIT_13);   // configure port b input capture pins as inputs
    mPORTBSetPinsDigitalOut(BIT_3);   // DP32 board LED 1
    mPORTBSetPinsDigitalIn(BIT_8);   // set I2C SCL to input
    mPORTBSetPinsDigitalIn(BIT_9);   // set I2C SDA to input
    mPORTBSetPinsDigitalOut(BIT_11); // DP32 board USB D- (eliminate floating spare input)

    mPORTAClearBits(BIT_0); //set all discrete outputs to zero
    mPORTAClearBits(BIT_3);
    mPORTBClearBits(BIT_3);
    mPORTBClearBits(BIT_11);

    // configure I2C
    OpenI2C1(I2C_ON, PRIMARY_I2C_CLK_SPEED); // 0x16 set PIC32MX i2c for approx 400khz i2c clock

    // configure pwm modules
    SetDCOC1PWM(0);             // set Output Compare 1 output to 0 to initialize
    PPSOutput(1,RPB4,OC1);      // configure Port B4 to Output Compare 1 (pwm output)
    OpenOC1( OC_ON | OC_TIMER_MODE16 | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE ,0,0); //continuous pwm mode with 0% duty cycle

    SetDCOC2PWM(0);             // set Output Compare 2 output to 0 to initialize
    PPSOutput(2,RPB5,OC2);      // configure Port B5 to Output Compare 2 (pwm output)
    OpenOC2( OC_ON | OC_TIMER_MODE16 | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE ,0,0); //continuous pwm mode with 0% duty cycle

    SetDCOC3PWM(0);             // set Output Compare 3 output to 0 to initialize
    PPSOutput(4,RPB14,OC3);      // configure Port B14 to Output Compare 4 (pwm output)
    OpenOC3( OC_ON | OC_TIMER_MODE16 | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE ,0,0); //continuous pwm mode with 0% duty cycle

    SetDCOC4PWM(0);             // set Output Compare 4 output to 0 to initialize
    PPSOutput(3,RPA4,OC4);      // configure Port A4 to Output Compare 4 (pwm output)
    OpenOC4( OC_ON | OC_TIMER_MODE16 | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE ,0,0); //continuous pwm mode with 0% duty cycle

    //  ChipKIT DP32 board requires external crystal to be removed before OC5 can be used
    SetDCOC5PWM(0);             // set Output Compare 5 output to 0 to initialize
    PPSOutput(3,RPA2,OC5);      // configure Port B13 to Output Compare 5 (pwm output)
    OpenOC5( OC_ON | OC_TIMER_MODE16 | OC_TIMER3_SRC | OC_PWM_FAULT_PIN_DISABLE ,0,0); //continuous pwm mode with 0% duty cycle

    // configure input capture modules (for radio rx)
    PPSInput(3,IC1,RPB2);       // configure input capture 1 module to Port B2 (radio rx input)
    OpenCapture1(IC_ON | IC_FEDGE_RISE | IC_CAP_16BIT | IC_TIMER3_SRC | IC_INT_1CAPTURE | IC_SP_EVERY_EDGE);  //input capture time since last rx edge change, start with rising edge

    PPSInput(4,IC2,RPB10);       // configure input capture 2 module to Port B10 (radio rx input)
    OpenCapture2(IC_ON | IC_FEDGE_RISE | IC_CAP_16BIT | IC_TIMER3_SRC | IC_INT_1CAPTURE | IC_SP_EVERY_EDGE);  //input capture time since last rx edge change, start with rising edge

    PPSInput(2,IC3,RPA1);       // configure input capture 4 module to Port B7 (radio rx input)
    OpenCapture3(IC_ON | IC_FEDGE_RISE | IC_CAP_16BIT | IC_TIMER3_SRC | IC_INT_1CAPTURE | IC_SP_EVERY_EDGE);  //input capture time since last rx edge change, start with rising edge

    PPSInput(1,IC4,RPB7);       // configure input capture 4 module to Port B7 (radio rx input)
    OpenCapture4(IC_ON | IC_FEDGE_RISE | IC_CAP_16BIT | IC_TIMER3_SRC | IC_INT_1CAPTURE | IC_SP_EVERY_EDGE);  //input capture time since last rx edge change, start with rising edge

    PPSInput(3,IC5,RPB13);       // configure input capture 5 module to Port B13 (radio rx input)
    OpenCapture5(IC_ON | IC_FEDGE_RISE | IC_CAP_16BIT | IC_TIMER3_SRC | IC_INT_1CAPTURE | IC_SP_EVERY_EDGE);  //input capture time since last rx edge change, start with rising edge

    for(int i = 1; i<4; i++) //clear Input Capture FIFOs after power up
    {
        int temp = mIC1ReadCapture(); 
        temp = mIC2ReadCapture();
        temp = mIC3ReadCapture();
        temp = mIC4ReadCapture();
        temp = mIC5ReadCapture();
     }

    // configure UART2
    //mPORTASetPinsDigitalOut(BIT_3);  // requires xtal DP32 board mod, used for UART output
    PPSOutput(4,RPA3,U2TX);      // configure Port A3 for UART2 Tx output
    OpenUART2(UART_EN | UART_NO_PAR_8BIT | UART_1STOPBIT, UART_TX_ENABLE | UART_RX_DISABLE, 21);
                                    //open uart in default mode (8 bit, no parity, 1 stop bit, rx disabled, baud = 57.6 Kbps)

    // configure A/D converter
    OpenADC10(  // config1
                ADC_MODULE_OFF |
                ADC_IDLE_CONTINUE |
                ADC_FORMAT_INTG16 |
                ADC_CLK_AUTO |
                ADC_AUTO_SAMPLING_ON |
                ADC_SAMP_ON,

                // config2
                ADC_VREF_AVDD_AVSS |  
                ADC_OFFSET_CAL_DISABLE |
                ADC_SCAN_ON |
                ADC_SAMPLES_PER_INT_2 | // set this to the desired adc inputs to sample per frame
                ADC_BUF_16 |
                ADC_ALT_INPUT_OFF,

                // config3
                ADC_SAMPLE_TIME_10 |    // sets SAMC which specifies number of Tad between successive conversions (allows sampling time of input)
                ADC_CONV_CLK_PB |       // use PB clock source
                ADC_CONV_CLK_3Tcy2,     // sets ADCS.   this configures ADC clock period (Tad), Tad =  2*(Tpb *(ADCS+1)  (note Tad must be >83.33ns)
                                        // ADC_CONV_CLK_3Tcy2 sets ADCS to 3, results in Tad = 300ns (ADC clk period) for 20MHz PB clk
                                        // ADC conversion time takes 12*Tad
            
                // configport    // configure io port
                ENABLE_AN9_ANA ,  // sets appropriate PIC pins to analog input

                // configscan
                SKIP_SCAN_AN0  |        // sets AD1CSSL(** bits are inverted and written to AD1CSSL), each bit corresponds to a mux channel (bit 0 = ch0),
                SKIP_SCAN_AN1  |        // set corresponding bit to 1 to include in scan sequence (0x0200 includes mux AN9 in scan)
                SKIP_SCAN_AN2  |
                SKIP_SCAN_AN3  |
                SKIP_SCAN_AN4  |
                SKIP_SCAN_AN5  |
                SKIP_SCAN_AN6  |
                SKIP_SCAN_AN7  |
                SKIP_SCAN_AN8  |
                // SKIP_SCAN_AN9  |     // auto scan AN9
                SKIP_SCAN_AN10 |
                // SKIP_SCAN_AN11 |     // auto scan AN11
                SKIP_SCAN_AN12 |
                SKIP_SCAN_AN13 |
                SKIP_SCAN_AN14 |
                SKIP_SCAN_AN15);

    ConfigIntADC10(ADC_INT_OFF);    // disable A/D interrupts
    EnableADC10();                  // enable A/D conversions
}

void configure_interrupts(void) // configure all interrupts
{
    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);
    INTSetVectorPriority(INT_TIMER_1_VECTOR, INT_PRIORITY_LEVEL_2);

    ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_2);    //timer1 is primary frame rate interrupt

    INTEnableSystemMultiVectoredInt();
    INTEnableInterrupts();

    EnableWDT();  // enable watchdog timer after interrupts are enabled
}

void delay(float delay_duration)  // delay routine (delays in seconds)
{
	volatile float start_time = 0; // seconds, start time for delay function

	start_time = absolute_time;
	while (absolute_time - start_time < delay_duration);    // wait for delay time
}

void get_high_water(void)   // record maximum interrupt routine execution time
{
        int_time = (ReadTimer1() / T1_CLK_FREQ);     //compute execution time of current interrupt routine
        if(int_time < 0.001)                        // if int_time < 1ms, then the interrupt routine exceeded ITER_PERIOD and Timer1 overflow occured
        {
            frame_overflow = true;
            high_water = high_water + ReadTimer1();  //add overflow time to current high water mark
            max_int_time = ITER_PERIOD + int_time;
            ++overflow_count;
        }

	if (ReadTimer1() > high_water)  // track high water mark if overflow has not occured
	{
            high_water = ReadTimer1();
            max_int_time = high_water / T1_CLK_FREQ;
	}
}

void unit_status(void)  // blink unit status LED
{
    unit_status_freq = DEFAULT_US_FREQ;
    if(!throttle_rx.channel_active) unit_status_freq = NO_RADIO_FREQ;
    if(trigger) unit_status_freq = LOGGING_US_FREQ;
    if(flash_full) unit_status_freq = FLASH_FULL_US_FREQ;
    if(annunciate_datalog_disable & absolute_time < 8) unit_status_freq = DATALOG_DISABLE_FREQ;
    if(camera_type == CANON_S95 & absolute_time < 8) unit_status_freq = S95_FREQ;
    if(erasing_flash) unit_status_freq = ERASING_FLASH_FREQ;
    if(!sensors_valid) unit_status_freq = SENSOR_FAILURE_FREQ;
    if(gyro_pu_offset_error) unit_status_freq = GYRO_OFFSET_ERROR_FREQ;

    if ((absolute_time - us_toggle_time > ((1/unit_status_freq)/2)) & !sensor_test_in_progress)
    {
        mPORTBToggleBits(BIT_3);
        us_toggle_time = absolute_time;
    }
}

float scale(float in_value, float in_min, float in_max, float out_min, float out_max) // scale input value to different set of coordinates using straight line equation
{                                   
    float mapped_value;
    float slope;
    float y_intercept;

    slope = (out_max - out_min)/(in_max - in_min);      // slope of scaling line
    y_intercept = out_max - (in_max * slope);           // y intercept of scaling line
    mapped_value = (slope * in_value) + y_intercept;    // line equation (y = mx+b)

    return mapped_value;
}

float limit(float in_value, float lower_limit, float upper_limit)
{
    float out_value = in_value;

    if(in_value < lower_limit) out_value = lower_limit;
    else if(in_value > upper_limit) out_value = upper_limit;
    return out_value;
}

float washout(float in_value, float washout_const)
{
    float out_value;
    if(washout_const == 0.0) out_value = in_value; // washout function is byapssed if constant set small
    else
    {
        if(in_value > 0.0) out_value = in_value - washout_const;
        else if(in_value < 0.0) out_value = in_value + washout_const;
    }
    return out_value;
}

void update_MPU_sensors(void)     // read latest MPU data and process sensor data
{
    // get latest MPU samples
    if((frame_count == 1) || (frame_count == 6)) platform_MPU9250.num_slave_bytes = 0;  // interleve reading tether sensors and baro data
    else platform_MPU9250.num_slave_bytes = 14;                                         // tether sensors are read at 500hz but skipped on 1st and 5th frame when baro is read


        platform_MPU9250.get_MPU_data();   // read MPU data if it has been initialized

//get_data_count++;
        calculate_platform_MPU_offsets();  // calculate temperature compensated sensor offsets

        // update gyro data with current samples
        platform_x_gyro.update(platform_MPU9250.local_x_rate, platform_x_gyro_offset, PLATFORM_X_GYRO_GAIN);
        platform_y_gyro.update(platform_MPU9250.local_y_rate, platform_y_gyro_offset, PLATFORM_Y_GYRO_GAIN);
        platform_z_gyro.update(platform_MPU9250.local_z_rate, platform_z_gyro_offset, PLATFORM_Z_GYRO_GAIN);

        // update accelerometer data with current samples
        platform_x_accel.update(platform_MPU9250.local_x_accel, platform_x_accel_offset, PLATFORM_X_ACCEL_GAIN);
        platform_y_accel.update(platform_MPU9250.local_y_accel, platform_y_accel_offset, PLATFORM_Y_ACCEL_GAIN);
        //platform_z_accel.update(platform_MPU9250.local_z_accel, platform_z_accel_offset, PLATFORM_Z_ACCEL_GAIN);

        if(tether_MPU_installed & platform_MPU9250.num_slave_bytes == 14)    //update tether sensors when sensor data is available
        {
            calculate_tether_MPU_offsets(); // calculate temperature compensated sensor offsets

            // update gyro data with current samples
            tether_x_gyro.update(platform_MPU9250.slave_x_rate, tether_x_gyro_offset, TETHER_X_GYRO_GAIN);
            tether_z_gyro.update(platform_MPU9250.slave_z_rate, tether_z_gyro_offset, TETHER_Z_GYRO_GAIN);

            // update accelerometer data with current samples
            tether_x_accel.update(platform_MPU9250.slave_x_accel, tether_x_accel_offset, TETHER_X_ACCEL_GAIN);
            tether_z_accel.update(platform_MPU9250.slave_z_accel, tether_z_accel_offset, TETHER_Z_ACCEL_GAIN);
        }
}


void initialize_PIDs(void) // set PID controller constants and reset states
{
    pitch_rate_PID.set_parameters(PITCH_RATE_KP, PITCH_RATE_KI, PITCH_RATE_KD, PITCH_RATE_I_LIM);  // initialize inner loop
    pitch_attitude_PID.set_parameters(PITCH_ATTITUDE_KP, PITCH_ATTITUDE_KI, PITCH_ATTITUDE_KD, PITCH_ATTITUDE_I_LIM);                      // initialize outer loop

    roll_rate_PID.set_parameters(ROLL_RATE_KP, ROLL_RATE_KI, ROLL_RATE_KD, ROLL_RATE_I_LIM);  // initialize inner loop
    roll_attitude_PID.set_parameters(ROLL_ATTITUDE_KP, ROLL_ATTITUDE_KI, ROLL_ATTITUDE_KD, ROLL_ATTITUDE_I_LIM);                      // initialize outer loop

    yaw_rate_PID.set_parameters(YAW_RATE_KP, YAW_RATE_KI, YAW_RATE_KD, YAW_RATE_I_LIM);  // initialize inner loop
    yaw_attitude_PID.set_parameters(YAW_ATTITUDE_KP, YAW_ATTITUDE_KI, YAW_ATTITUDE_KD, YAW_ATTITUDE_I_LIM);                      // initialize outer loop

    tether_pitch_rate_PID.set_parameters(T_RATE_KP, T_RATE_KI, T_RATE_KD, T_RATE_I_LIM);  // initialize inner loop
    tether_pitch_attitude_PID.set_parameters(T_ATTITUDE_KP, T_ATTITUDE_KI, T_ATTITUDE_KD, T_ATTITUDE_I_LIM);                      // initialize outer loop

    tether_roll_rate_PID.set_parameters(T_RATE_KP, T_RATE_KI, T_RATE_KD, T_RATE_I_LIM);  // initialize inner loop
    tether_roll_attitude_PID.set_parameters(T_ATTITUDE_KP, T_ATTITUDE_KI, T_ATTITUDE_KD, T_ATTITUDE_I_LIM);                      // initialize outer loop
  
    reset_PIDS();
}

void reset_PIDS(void)
{
    pitch_rate_PID.reset();
    roll_rate_PID.reset();
    yaw_rate_PID.reset();
    
    pitch_attitude_PID.reset();
    roll_attitude_PID.reset();
    yaw_attitude_PID.reset();
    
    tether_pitch_rate_PID.reset();
    tether_roll_rate_PID.reset();

    tether_pitch_attitude_PID.reset();
    tether_roll_attitude_PID.reset();
}

void drive_servos(void)
{
    tilt_cmd_rlm.compute((tilt_cmd + TILT_OFFSET), 90 * ITER_PERIOD);
    tilt_svo_pw = scale(tilt_cmd_rlm.out, -45, 45, 0.002032, 0.00113);
    SetDCOC5PWM(tilt_svo_pw/T3_SEC_PER_BIT); // pitch front pwm output
}

void measure_platform_vib(void)     // platform vibration measurement
{
    // **** CAUTION **** Note this test will run throttle automatically when executed

    // set tx mode ctrl stick (throttle stick) to >10% during this test to enable logging of proper variables.

    // set measure_platform_vib_enable = true; in main function to enable vib measurement mode

    // set datalog parameters to the following (in datalog() function):
        //log_sample_hz = 115;                   // log sample frequency
        //num_log_parameters = 4;                // number of parameters to log

        //log_data[1]  = absolute_time;
        //log_data[2]  = platform_x_accel.accel;
        //log_data[3]  = platform_y_accel.accel;
        //log_data[4]  = platform_z_accel.accel;

    trigger = true;  //start datalogging

    vib_throttle_pw = 0.001;                        // arm esc's
    SetDCOC1PWM(vib_throttle_pw/T2_SEC_PER_BIT);
    SetDCOC2PWM(vib_throttle_pw/T2_SEC_PER_BIT);
    SetDCOC3PWM(vib_throttle_pw/T2_SEC_PER_BIT);
    SetDCOC4PWM(vib_throttle_pw/T2_SEC_PER_BIT);
    delay(10);

    vib_throttle_pw = 0.00125;              //  log vib with 25% throttle
    SetDCOC1PWM(vib_throttle_pw/T2_SEC_PER_BIT);
    SetDCOC2PWM(vib_throttle_pw/T2_SEC_PER_BIT);
    SetDCOC3PWM(vib_throttle_pw/T2_SEC_PER_BIT);
    SetDCOC4PWM(vib_throttle_pw/T2_SEC_PER_BIT);
    delay(5);

   vib_throttle_pw = 0.00160;               // log vib with 60% throttle
    SetDCOC1PWM(vib_throttle_pw/T2_SEC_PER_BIT);
    SetDCOC2PWM(vib_throttle_pw/T2_SEC_PER_BIT);
    SetDCOC3PWM(vib_throttle_pw/T2_SEC_PER_BIT);
    SetDCOC4PWM(vib_throttle_pw/T2_SEC_PER_BIT);
    delay(5);

   vib_throttle_pw = 0.00175;               // log vib  with 75% throttle
   SetDCOC1PWM(vib_throttle_pw/T2_SEC_PER_BIT);
   SetDCOC2PWM(vib_throttle_pw/T2_SEC_PER_BIT);
   SetDCOC3PWM(vib_throttle_pw/T2_SEC_PER_BIT);
   SetDCOC4PWM(vib_throttle_pw/T2_SEC_PER_BIT);
   delay(5);

   vib_throttle_pw = 0.001;                        // stop motors
   SetDCOC1PWM(vib_throttle_pw/T2_SEC_PER_BIT);
   SetDCOC2PWM(vib_throttle_pw/T2_SEC_PER_BIT);
   SetDCOC3PWM(vib_throttle_pw/T2_SEC_PER_BIT);
   SetDCOC4PWM(vib_throttle_pw/T2_SEC_PER_BIT);
   while(true);                                    // infinite loop
}

void calibrate_ESCs(void)  // drive ESC's with minimum and maximum expected pulsewidths to calibrate their full scale range
{                          
                           // To calibrate T-Motor ESC's, follow these steps:

                           // ********** Perform ESC calibration with all propellers REMOVED from motors *****************
                           //      1.  **** Turn off ESC power *****  (IMPORTANT, DO NOT START THIS TEST EXECUTION WITH ESC POWER ON OR MOTORS WILL POWER UP TO FULL THROTTLE)
                           //      2.  Run this routine
                           //      3.  Place joystick in max position
                           //      4.  ***Turn on ESC power ***
                           //      5.  Wait for two short beeps from ESC
                           //      6.  Lower throttle joystick to minimum position
                           //      7.  Wait for single long beep from ESC
                           //      8.  Turn off ESC power - calibration complete
    delay(2);
    while(true)
    {
        while (throttle_rx_cmd < 85);  // wait for throttle to be raised to full stick position
        SetDCOC1PWM(0.00198/T2_SEC_PER_BIT);  // set maximum throttle to 0.00198 sec  (note that 0.002 = 100% pulsewidth at 500Hz iteration rate, so reduce full scale by 2%)
        SetDCOC2PWM(0.00198/T2_SEC_PER_BIT);
        SetDCOC3PWM(0.00198/T2_SEC_PER_BIT);
        SetDCOC4PWM(0.00198/T2_SEC_PER_BIT);

        while(throttle_rx_cmd > 20);  // wait for throttle to be set to minimum stick position
        SetDCOC1PWM(0.001/T2_SEC_PER_BIT);  // set minimum throttle to 0.001 sec
        SetDCOC2PWM(0.001/T2_SEC_PER_BIT);
        SetDCOC3PWM(0.001/T2_SEC_PER_BIT);
        SetDCOC4PWM(0.001/T2_SEC_PER_BIT);

        while(true);
    }
}

void measure_step_response(void)
{
    // set the following variable in main() function:
         // measure_step_response_enable = true;

    // set debug parameters to the following datalog(void) function:
         //log_sample_hz = 90;                    // log sample frequency
         //num_log_parameters = 5;                // number of parameters to log

         //log_data[2]  = pitch_rx_cmd;
         //log_data[3]  = -pitch_angle;
         //log_data[4]  = platform_y_accel.angle;
         //log_data[5]  = pitch_front_mtr_cmd * 1000.0;
  
    roll_ctrl_enable = false;  // enable_disable desired axis
    pitch_ctrl_enable = true;  // enable/disable desired axis
    yaw_ctrl_enable = false;   // disable yaw control for step response test

    rx_override = true;        // override rx commands to control axis and throttle
    delay(0.1);
    tether_mode = false;
    pan_mode = false;

    pitch_rx_cmd = 0;
    throttle_rx_cmd = 0;
    throttle_enable = true;
    delay(6);
    throttle_rx_cmd = 60;      // set throttle to desired %
    delay(5);
    trigger = true;            // start data logging  (check log
    delay(1);
    pitch_rx_cmd = 20;         // insert positive step
    delay(10);
    pitch_rx_cmd = 0;
    delay(10);
    throttle_rx_cmd = 0;
   
    int count;
    while(true)
    {
        count++;
    }

}

void initialize_baro(void)  // initialize barometer by resetting, reading constants and reading initial temperature
                            // this function must be executed prior to platform_initialized because it uses absolute_time;
{
    baro_alt.reset();                   // perform MS5611 baro reset (see datasheet)
    delay(0.1);
    baro_alt.get_cal_data();            // read MS5611 baro calibration constants from PROM
    baro_alt.start_temp_conversion();   // start adc conversion for baro temperature sensor
    delay(0.01);                        // wait 10 msec for temperature adc conversion to complete
    baro_alt.get_temp_data();           // read first temp data
    baro_alt.start_baro_conversion();   // initiate baro conversion after reading temperature
    delay(0.01);                        // wait 10 msec for conversion to complete
    baro_alt.get_baro_data();           // read first baro data
}

void update_baro_data(void) // get temperature compensated baro pressure
                            // this function utilizes multi-rate frame execution and performs interleaved baro and temperature conversions
                            // every 5th frame interval.  2msec frames results in an effective baro update rate of 50Hz
{
    if(frame_count == 1) // interleave baro and temperature conversions every 5th frame,  effective sample rate for each is 50Hz
        {
            baro_alt.get_temp_data();           // read temp data from previous conversion
            baro_alt.start_baro_conversion();   // initiate baro conversion (must allow ~10ms between start of conversion
        }                                       // and reading data (see MS5611 datasheet)
        if(frame_count == 6) // interleave baro and temperature conversions every 5th frame,  effective sample rate for each is 50Hz
        {
            baro_alt.get_baro_data();           // read baro data from previous conversion
            baro_alt.start_temp_conversion();   // initiate temperature conversion (read result in 5 frames (10msec))
        }
}

void initialize_MPUs(void)
{
    platform_MPU9250.initialize(PLATFORM_MPU_ADDRESS, PLATFORM_MAG_ADDRESS); // configure MPU1 via PIC i2c bus
    platform_MPU9250.get_MPU_WhoAmI();                         // read MPU WhoAMI register

    platform_MPU9250.bypass_mode_enable();                     // set bypass mode on MPU1 to allow PIC to directly configure MPU2
    tether_MPU9250.initialize(TETHER_MPU_ADDRESS, TETHER_MAG_ADDRESS); // configure MPU2 via PIC i2c bus
    
    tether_MPU9250.get_MPU_WhoAmI();                         // read MPU WhoAMI register
    if(tether_MPU9250.MPU_WhoAmI == 0x71) 
    {
        tether_MPU_installed = true;            // set flag true if tether MPU is installed
        platform_MPU9250.num_slave_bytes = 14;  // number of bytes to autonomously unload from tether MPU (accel, gyro and temperature)
    }
    else
    {
        tether_MPU_installed = false;          // set flag false if tether MPU is not installed
        platform_MPU9250.num_slave_bytes = 0;  // number of bytes to autonomously unload from tether MPU (accel, gyro and temperature)
    }
    platform_MPU9250.bypass_mode_disable();    // disable i2c bypass mode and set platform MPU to be the master of it's aux i2c bus and autonomously unload slave i2c device

    if(tether_MPU_installed) // configure platform MPU to autonomously read tether (slave) MPU data if it is installed
    {
        writeI2c(PLATFORM_MPU_ADDRESS, I2C_MST_DELAY_CTRL, 0x80);   // delay shadowing of external sensor data until all data is received
        writeI2c(PLATFORM_MPU_ADDRESS, I2C_SLAVE_ADDR, 0xE9);       // set slave i2c address (tether MPU i2c adrs is 0x69, bits 6:0) and set r/w bit 7 to 1 (read mode)
        writeI2c(PLATFORM_MPU_ADDRESS, I2C_SLV0_REG, 0x3B);         // starting address of tether MPU data registers (accel, temperature, gyro)
        writeI2c(PLATFORM_MPU_ADDRESS, I2C_SLV0_CTRL, 0x8E);        // enable autonomous data transfer of 14 bytes from tether MPU into platform MPU data memory                                                           // autonomous transfer rate is Fs (1000 Hz) for each parameter
    }                                                               // autonomuous transfer rate is Fs (1000Hz) for each parameter
}

void calculate_platform_MPU_offsets(void)    // temperature compensation using x = mx+b straight line scaling plus power up rate offset
{
    platform_x_gyro_offset = (PLATFORM_X_GYRO_TC_SLOPE * platform_MPU9250.local_temperature) + PLATFORM_X_GYRO_TC_Y_INT + plat_x_gyro_pu_offset;
    platform_y_gyro_offset = (PLATFORM_Y_GYRO_TC_SLOPE * platform_MPU9250.local_temperature) + PLATFORM_Y_GYRO_TC_Y_INT + plat_y_gyro_pu_offset;
    platform_z_gyro_offset = (PLATFORM_Z_GYRO_TC_SLOPE * platform_MPU9250.local_temperature) + PLATFORM_Z_GYRO_TC_Y_INT + plat_z_gyro_pu_offset;

    platform_x_accel_offset = (PLATFORM_X_ACCEL_TC_SLOPE * platform_MPU9250.local_temperature) + PLATFORM_X_ACCEL_TC_Y_INT;
    platform_y_accel_offset = (PLATFORM_Y_ACCEL_TC_SLOPE * platform_MPU9250.local_temperature) + PLATFORM_Y_ACCEL_TC_Y_INT;
    //platform_z_accel_offset = (PLATFORM_Z_ACCEL_TC_SLOPE * platform_MPU9250.local_temperature) + PLATFORM_Z_ACCEL_TC_Y_INT;
    
    // non temp compensated accel angle offset.  to set this value, place platform on level surface (level table) and run software
    // pause software after about 10 seconds to allow sensors to stabilize.  set platform_x/y_accel_offset values to the steady state values
    // of platform_x/y_accel.raw_average values.  accel variation over temperature is assumed negligable.  long term tether angle is assumed to be
    // very close to the steady state accel angle, however gyro rate offset will result in additional angle offset due to the complementary filter
    // used for sensor fusion.  it is assumed that the gyro rate offset is small, therefore, tehter angle offset from gyro is assumed small
    // consider adding temp compensation
}

void calculate_tether_MPU_offsets(void)    // temperature compensation using x = mx+b straight line scaling
{
    tether_x_gyro_offset = (TETHER_X_GYRO_TC_SLOPE * platform_MPU9250.slave_temperature) + TETHER_X_GYRO_TC_Y_INT + tether_x_gyro_pu_offset;
    tether_z_gyro_offset = (TETHER_Z_GYRO_TC_SLOPE * platform_MPU9250.slave_temperature) + TETHER_Z_GYRO_TC_Y_INT + tether_z_gyro_pu_offset;

    tether_x_accel_offset = (TETHER_X_ACCEL_TC_SLOPE * platform_MPU9250.slave_temperature) + TETHER_X_ACCEL_TC_Y_INT;
    tether_z_accel_offset = (TETHER_Z_ACCEL_TC_SLOPE * platform_MPU9250.slave_temperature) + TETHER_Z_ACCEL_TC_Y_INT;

    // non temp compensated tether accel angle offset.  to set this value, place tether sensor in vertical orientation and run software
    // pause software after about 10 seconds to allow sensors to stabilize.  set tether_x/z_accel_offset values to the steady state values
    // of tether_x/z_accel.raw_average values.  accel variation over temperature is assumed negligable.  long term tether angle is assumed to be
    // very close to the steady state accel angle, however gyro rate offset will result in additional angle offset due to the complementary filter
    // used for sensor fusion.  it is assumed that the gyro rate offset is small, therefore, tehter angle offset from gyro is assumed small
}

void get_sensor_offsets(void)
{
    // To measure sensor offsets see file "MPU Offset Calibration Instructions.doc"

    #define OFFSET_SAMPLE_HZ 1.0 // hz, debug sample frequency

    // trigger condition
    if (absolute_time > 4) offset_trigger = true;  //  start sampling offsets 4 seconds after power up

    if((absolute_time - last_offset_sample_time) > (1.0/OFFSET_SAMPLE_HZ - 0.001))   // sample parameters at specified rate (subtract 0.001 from desired sample interval due to rtc quantization)
    {
        if((offset_sample_count < MAX_OFFSET_SAMPLES) & offset_trigger)
        {
            // assign parameters to datalog
            offset_buf[0][offset_sample_count] = absolute_time;

            offset_buf[1][offset_sample_count] = platform_MPU9250.local_temperature;

            offset_buf[2][offset_sample_count] = platform_x_gyro.raw_average;
            offset_buf[3][offset_sample_count] = platform_y_gyro.raw_average;
            offset_buf[4][offset_sample_count] = platform_z_gyro.raw_average;

            offset_buf[5][offset_sample_count] = platform_x_accel.raw_average;
            offset_buf[6][offset_sample_count] = platform_y_accel.raw_average;
            //offset_buf[7][offset_sample_count] = platform_z_accel.raw_average;


        if(tether_MPU_installed)
        {
            offset_buf[8][offset_sample_count] = platform_MPU9250.slave_temperature;

            offset_buf[9][offset_sample_count] = tether_x_gyro.raw_average;
            offset_buf[10][offset_sample_count] = tether_z_gyro.raw_average;

            offset_buf[11][offset_sample_count] = tether_x_accel.raw_average;
            offset_buf[12][offset_sample_count] = tether_z_accel.raw_average;
        }

       offset_sample_count++;
       last_offset_sample_time = absolute_time;
        }
    }
}

void pan_platform(float angle)  // pan the platform specified degrees.  abort if not in pan mode
{
    if(pan_mode) pan_cmd += angle;
}

void wait_pan(float interval)  // delay interval seconds.  abort if pan mode is disabled
{
    volatile float elapsed_time = 0.0;
    volatile float start_time = absolute_time;
    volatile float end_time = start_time + interval;

    while((absolute_time < end_time) & pan_mode);
}

void __attribute__((optimize("O0"))) shutter_half_press(void) // Ricoh GR shutter half press (Ricoh CA-1 / CA-2 protocal) with single 30ms pulse
{
      mPORTASetBits(BIT_0);
      delay(0.03);
      mPORTAClearBits(BIT_0);
}

void __attribute__((optimize("O0"))) shutter_full_press(void) // Ricoh GR shutter full press, (Ricoh CA-1 / CA-2 protocal)(150 ms pulse, shutter releases on falling edge)
{
    mPORTASetBits(BIT_0);
    delay(0.15);
    mPORTAClearBits(BIT_0);
}

void __attribute__((optimize("O0"))) shutter_release(void)  // Ricoh GR release shutter (Ricoh CA-1 / CA-2 protocal) with two 30ms pulses separated by 30ms
{
    delay(0.2);  // wait for previous full press to complete
    mPORTASetBits(BIT_0);
    delay(0.03);
    mPORTAClearBits(BIT_0);
    delay(0.03);
    mPORTASetBits(BIT_0);
    delay(0.03);
    mPORTAClearBits(BIT_0);
}

void __attribute__((optimize("O0"))) take_picture_wait_pan_trigger(void)
{
    shutter_half_press();
    delay(0.2);
    while(pan_mode & abs(yaw_rx_rate_cmd) < 10);  //wait for yaw rx signal to take picture
    shutter_full_press();  // take picture
    shutter_release();
}

void __attribute__((optimize("O0"))) take_picture(void)
{
     shutter_half_press();
     delay(0.2);
     shutter_full_press();
     shutter_release();
}

void check_sensors(void)
{
     if(!(platform_MPU9250.MPU_WhoAmI == 0x71 & tether_MPU_installed & (baro_alt_lpf.out > 50 & baro_alt_lpf.out < 400)))
     {
         sensors_valid = false;
         while(true);   // endless loop if sensors are invalid
     }

     get_gyro_pu_offsets();

     manual_sensor_test();
}

void disable_datalogging(void)
{
    datalogging_enabled = false; // disable flash datalogging (function currently not used)
    annunciate_datalog_disable = true;
}

void get_gyro_pu_offsets(void)  // measure power up platform gyro offsets
                                // platform must be stable and not moving when this measurement is taken
{
    plat_x_gyro_pu_offset = limit(platform_x_gyro.average, -0.3, 0.3);  //x gyro offset limit larger than other axis, re-do sensor calibration later to reduce this
    plat_y_gyro_pu_offset = limit(platform_y_gyro.average, -0.3, 0.3);
    plat_z_gyro_pu_offset = limit(platform_z_gyro.average, -0.4, 0.4);

    tether_x_gyro_pu_offset = limit(tether_x_gyro.average, -0.3, 0.3);
    tether_z_gyro_pu_offset = limit(tether_z_gyro.average, -0.3, 0.3);

    pitch_angle = 0.0;  //remove any accumulated offsets and drifts
    roll_angle = 0.0;
    yaw_angle = 0.0;

    if(abs(plat_x_gyro_pu_offset) == 0.3 || abs(plat_y_gyro_pu_offset) == 0.3 || abs(plat_z_gyro_pu_offset) == 0.4 || abs(tether_x_gyro_pu_offset) == 0.3 || abs(tether_z_gyro_pu_offset) == 0.3 || abs(tether_pitch_angle) > 10 || abs(tether_roll_angle) > 10)
    {
        gyro_pu_offset_error = true;
        while(true);
    }
}

void manual_sensor_test(void)  // check platform and tether angle sensors prior to flight.
{                              // shortly after power up unit status LED will freeze on to indicate manual test in progress.
                               // To test platform sensors, raise pitch and roll axis >20 deg and observe LED turn off at threshold.
                               // To test tether sensors, move sensor > 45 deg pitch and roll and observe LED turn off at threshold.
                               // Move roll joystick > 10 deg to exit manual sensor test mode.

    if (camera_type == CANON_S95) delay(2.0);  // wait for unit status to blink a few times before performing sensor test
                                               // if S95 camera selected to indicate camera mode change
    sensor_test_in_progress = true; // set flag to suspend unit status routine

     while(abs(roll_rx_cmd_raw) < 10)  //exit manual sensor test if roll joystick moved to > 10 deg either direction
     {
         if((abs(pitch_angle) < 20) & (abs(roll_angle) < 20) & (abs(tether_pitch_angle) < 45) & (abs(tether_roll_angle) < 45)) mPORTBSetBits(BIT_3);
         else mPORTBClearBits(BIT_3);
     }
    sensor_test_in_progress = false; // clear flag to resume unit status routine
}

void delay_usec(float time_delay)   // delay specified usec based on timer1
                                    // routine should only be called within interrupt routine to ensure timer1 does not roll-over during delay period.
{
    int timeout = ReadTimer1() + int(float(time_delay * (T1_CLK_FREQ * 1E-6)));
    while(ReadTimer1() < timeout); //wait for timer1 to reach timeout count
}

void __attribute__((optimize("O0"))) thrust_test(void)
{
    float throttle_limit = 0;
    float throttle_thrust_cmd = 0;

    pitch_ctrl_enable = false;
    roll_ctrl_enable = false;
    yaw_ctrl_enable = false;
    delay(2);

    while(true)
    {
         if(throttle_enable & (throttle_rx_cmd > THROTTLE_START_VALUE))
         {
            if(mode_sw_posn == DOWN) throttle_limit = 50;
            if(mode_sw_posn == MIDDLE) throttle_limit = 65;
            if(mode_sw_posn == UP) throttle_limit = 75;

            throttle_thrust_cmd = limit(throttle_rx_cmd, 0, throttle_limit);
            pitch_rear_mtr_cmd = throttle_thrust_cmd;
            pitch_rear_mtr_cmd = scale(pitch_rear_mtr_cmd, 0, 100, 0.001, 0.00198);
            SetDCOC2PWM(pitch_rear_mtr_cmd/T2_SEC_PER_BIT); // pitch rear pwm output
         }
    }
}

void __attribute__((optimize("O0"))) take_s95_picture(void)
{
     mPORTASetBits(BIT_0);   // S95 shutter half press
     delay(2);
     mPORTAClearBits(BIT_0); // S95 full shutter press
}

void __attribute__((optimize("O0"))) take_s95_picture_wait_pan_trigger(void)
{
    mPORTASetBits(BIT_0);   // S95 shutter half press
    delay(2);
    while(pan_mode & abs(yaw_rx_rate_cmd) < 10);  //wait for yaw rx signal to take picture
    mPORTAClearBits(BIT_0); // S95 full shutter press
}

void __attribute__((optimize("O0"))) ricoh_gr_routine()
{
    if(!pan_mode)
    {
       tilt_cmd = 10;
       delay(1);
       take_picture();
       if(!pan_mode)delay(1);
       if(!pan_mode)delay(1);
       if(!pan_mode)delay(1);
       if(!pan_mode)delay(1);
    }

    if(pan_mode)
    {
        if(pan_mode)
        {
            tilt_cmd = -15;
            delay(1.0);
            take_picture_wait_pan_trigger();
        }

        if(pan_mode)
        {
            if(pan_mode) tilt_cmd = 20;
            if(pan_mode) delay(0.4);
            take_picture();
        }

//!!!
if(vert_pic_arm == true) vert_pic_arm = false;  //future code to take vertical picture during first pan sequence

        if(pan_mode)
        {
            if(pan_mode) tilt_cmd = 50;
            if(pan_mode) delay(0.4);
            shutter_half_press();
            delay(0.2);
            shutter_full_press();
            pan_platform(pan_increment);
            tilt_cmd = -15;
            shutter_release();
            delay(1.3);
        }
    }
}

void __attribute__((optimize("O0"))) canon_s95_routine()
{
    if(!pan_mode)
    {
       tilt_cmd = 10;
       delay(1);
       take_s95_picture();
       if(!pan_mode)delay(1);
       if(!pan_mode)delay(1);
       if(!pan_mode)delay(1);
       if(!pan_mode)delay(1);
    }

    if(pan_mode)
    {
        if(pan_mode)
        {
            tilt_cmd = -15;
            delay(1.0);
            take_s95_picture_wait_pan_trigger();
            delay(0.5);
        }

        if(pan_mode)
        {
            if(pan_mode) tilt_cmd = 20;
            if(pan_mode) delay(1);
            take_s95_picture();
            delay(0.5);
        }

//!!!
if(vert_pic_arm == true)  vert_pic_arm = false;  //future code to take vertical picture during first pan sequence

        if(pan_mode)
        {
            pan_platform(pan_increment);
            tilt_cmd = -15;
            delay(1);
        }
    }
}
