       // XC32 command summary
        //      port IO
        //          PPSOutput(3,RPA4,OC4);      // peripheral bus matrix select
        //          mPORTBSetPinsDigitalOut(BIT_2 | BIT_3);
        //          mPORTBSetBits(BIT_2);
        //          mPORTBClearBits(BIT_2);
        //          mPORTBToggleBits(BIT_2);
        //      timers
        //          OpenTimer1(T1_ON | T1_SOURCE_INT | T1_PS_1_1, T1_PRELOAD);
        //          ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_2);
        //          OpenTimer2(T2_ON | T2_PS_1_4, T2_PRELOAD);  //timer t2 set to free run without interrupt
        //      output compare (set for continuous pwm)
        //          OpenOC4( OC_ON | OC_TIMER_MODE16 | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE,0,0);  //pwm freqency = t2 rollover value
        //          SetDCOC4PWM(pulse_width1); //(pw in seconds = pulse_width*T2 clock period)
        //      I2C
        //          OpenI2C1(I2C_ON, 0X02F);
        //          StartI2C1();
        //          MasterWriteI2C1(0X00);
        //          IdleI2C1();
