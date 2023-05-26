//////////////////////////////////
// I2C Library
//////////////////////////////////

// c 2015 IowaKAPer / Mike LeDuc

int readI2c(unsigned char slaveAddress, unsigned char slaveRegister, unsigned int numBytes, unsigned char i2cData[])  // returns 0 if successful, returns -1 if I2C abort
{
    unsigned char data_buffer[numBytes];
    abort_I2C = false;

    IdleI2C1();                                 // wait for clear bus
    StartI2C1();                                // send the start sequence
    while (I2C1CONbits.SEN );                   // wait until start sequence is over

    if(MasterWriteI2C1((slaveAddress << 1) | 0) == -2)  abort_I2C = 1;  //MasterWriteI2C1 returns -2 if slave NACK, if so, then restart and write slave address again
    if(!abort_I2C)
    {
        if(MasterWriteI2C1(slaveRegister) == -2) abort_I2C = 2;  // send the register address to read
        if(!abort_I2C)
        {
            RestartI2C1();                              // send restart sequence
            while (I2C1CONbits.RSEN );                  // wait until restart sequence is over
            if(MasterWriteI2C1((slaveAddress << 1) | 1 ) == -2) abort_I2C = 3;  // transmit read command
            if(!abort_I2C)
            {
                for(int i=0; i<numBytes; i++)               // Receive the number of bytes specified by numBytes
                {                                       // code in this for loop is modeled from MastergetsI2C1 XC32 library routine
                    if(!abort_I2C)
                    {
                        I2C1CONbits.RCEN = 1;               // start receive sequence
                        int timeout = ReadTimer1() + 600;
                        while(I2C1CONbits.RCEN) if(ReadTimer1() > timeout)  resetI2c();            // wait for byte to be received
                        if(!abort_I2C)
                        {
                            data_buffer[i] = I2C1RCV;               // save byte received
                            if(i == numBytes-1)                 // If last byte, generate NACK sequence
                            {
                                I2C1CONbits.ACKDT = 1;
                                I2C1CONbits.ACKEN = 1;
                            }
                            else                                // if not last byte, generate ACK sequence
                            {
                                I2C1CONbits.ACKDT = 0;
                                I2C1CONbits.ACKEN = 1;
                            }
                            int timeout = ReadTimer1() + 600;
                            while(I2C1CONbits.ACKEN) if(ReadTimer1() > timeout)  resetI2c();   // wait for ACK/NACK sequence to complete
                        }
                    }
                }
            }
        }
    }
    StopI2C1();                                 //send the stop sequence
    while (I2C1CONbits.PEN );                   // wait until stop sequence is over

    if(abort_I2C)
    {
        abort_I2C_count++;
        last_abort_I2C_cause = abort_I2C;
        return -1;
    }

    if(!abort_I2C) 
    {
        for(int i=0; i<numBytes; i++) i2cData[i] = data_buffer[i];  //return sensor data if I2C cycle was not aborted
        return 0;
    }
}

void writeI2c(unsigned char slaveAddress, unsigned char slaveRegister, unsigned char writeData)
{
    IdleI2C1();     // wait for clear bus
    StartI2C1();    // send the start sequence
    while (I2C1CONbits.SEN );                   // wait until start condition is over

    MasterWriteI2C1((slaveAddress << 1) | 0);   // send the first byte (device adrs and R/W bit)
    IdleI2C1();                                 // wait to complete
    MasterWriteI2C1(slaveRegister);             // send the register address to write
    IdleI2C1();                                 // wait to complete
    MasterWriteI2C1(writeData);                 // send the data to write
    IdleI2C1();                                 // wait to complete

    StopI2C1();                                 //send the stop sequence
    while (I2C1CONbits.PEN );                   // wait until stop condition is over
}

void writeCommandI2c(unsigned char slaveAddress, unsigned char i2cCommand)  //this is a modified i2c write sequence that replaces the write address with a
                                                                            // command byte and sends no write data (see MS5611 barometer datasheet
{
    IdleI2C1();                                 // wait for clear bus
    StartI2C1();                                // send the start sequence
    while (I2C1CONbits.SEN );                   // wait until start condition is over

    MasterWriteI2C1((slaveAddress << 1) | 0);   // send the first byte (device adrs and R/W bit)
    IdleI2C1();                                 // wait to complete
    MasterWriteI2C1(i2cCommand);                // send the write command
    IdleI2C1();                                 // wait to complete
    StopI2C1();                                 //send the stop sequence
    while (I2C1CONbits.PEN );                   // wait until stop condition is over
}

void __attribute__((optimize("O0"))) resetI2c(void)  // take control of I2C SCL and drive 9 clock pulses to flush slave shift registers
{                                                    // turn off compiler optimization to ensure sequential bit control operations are not optimized out
     CloseI2C1(); // Close I2C to allow SCL pins to be driven manually
     delay_usec(2);
     mPORTBSetPinsDigitalOut(BIT_8);   // set I2C SCL to output
     delay_usec(2);
     mPORTBClearBits(BIT_8);  // drive SCL low
     delay_usec(1.25);        // wait for 1/2 period of 400Kbps clock
     for(int i=0; i<9; i++)    // clock scl 9 times to clear slave registers  at 400 Kbps rate (slave should drive SDA high during this time)
     {
           mPORTBSetBits(BIT_8);
           delay_usec(1.25);
           mPORTBClearBits(BIT_8);
           delay_usec(1.25);
     }
     mPORTBSetPinsDigitalIn(BIT_8);   // set I2C SCL to input
     delay_usec(1);
     OpenI2C1(I2C_ON, PRIMARY_I2C_CLK_SPEED); // re-open I2C device
     delay_usec(5);
     abort_I2C = 4;
}
