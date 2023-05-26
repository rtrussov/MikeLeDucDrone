//////////////////////////////////
// BaroLib Classes
//////////////////////////////////

// c 2015 IowaKAPer / Mike LeDuc

BaroAltLib::BaroAltLib()    // class constructor
{                           // only run baro i2c functions in non-interruptable routines to avoid i2c lock-up
    raw_baro = 0;           // 24bit value read from MS5611 barometer
    C1 = 0;
    C2 = 0;
    C3 = 0;
    C4 = 0;
    C5 = 0;
    C6 = 0;
    pressure = 0;
    temperature = 0;
    altitude = 0;
    baro_initialized = false;    // don't run baro in real time interrupt routines until it has been initialized
}

void BaroAltLib::reset()  // initialize baro device
{
    writeCommandI2c(BARO_ADRS, BARO_RESET_CMD);                                          
    delay(0.1);             // wait for baro to stabilize after initializing
    baro_initialized = true; // set flag to indicate baro is initialized and can be used for computations
}

void BaroAltLib::start_baro_conversion()  // initiate baro adc conversion
{
    writeCommandI2c(BARO_ADRS, START_BARO_ADC_CMD);
}

void BaroAltLib::start_temp_conversion()    // initiate temperature adc conversion
{
    writeCommandI2c(BARO_ADRS, START_TEMP_ADC_CMD);
}

void BaroAltLib::get_baro_data(void)  // read 24bit raw baro data from MS5611  (assumes 25C for all baro calculatcions)
                                      // must wait ~ 10ms between start of adc conversion and reading data (see MS5611 datasheet)
{
    unsigned char i2c_buffer[3];  // buffer for baro i2c data

    if(readI2c(BARO_ADRS, BARO_ADC_REG, 3, i2c_buffer) == 0)   //read 3 bytes of baro data into i2c read buffer
    {
        raw_baro = (unsigned int)(i2c_buffer[0]<<16 | i2c_buffer[1]<<8 | i2c_buffer[2]);  // transfer i2c buffer data into 24bit baro data

        // calculate temperature compensated pressure and altitude (see MS5611 datasheet)
        off = C2 * 65536 + (C4 * dT)/128;
        sens = C1 * 32768 + (C3 * dT)/256;
        p = ((raw_baro * sens)/2097152 - off)/32768;

        pressure = (float)(p)/100;  // pressure in mbar

        altitude = (1 - powf(pressure/1013.25, 0.190284)) * (145366.45 * 0.3048);  // altitude in meters
        // =((1-(pressure/1013.25)^0.190284))*145366.45
    }
}

void BaroAltLib::get_temp_data(void)  // read 24bit raw temperature data from MS5611
{                                     // ** this temperature only read once at power up and is performed outside of frame interrupt routine,
                                      // ** and uses Absolute_time to delay between baro adc conversion and reading of data
                                      // ** therefore only call this function prior to Platform_Initialized = true
                                      // ** must wait ~ 10ms between start of adc conversion and reading data (see MS5611 datasheet)
    unsigned char i2c_buffer[3];  // buffer for temperature i2c data

    if(readI2c(BARO_ADRS, BARO_ADC_REG, 3, i2c_buffer) == 0)   //read 3 bytes of temperature data into i2c read buffer
    {
        raw_temp = (unsigned int)(i2c_buffer[0]<<16 | i2c_buffer[1]<<8 | i2c_buffer[2]);  // transfer i2c buffer data into 24bit temperature data

        dT = raw_temp - (C5 * 256.0);
        temp = 2000 + (unsigned long)(dT * C6)/8388608;
        temperature = (float)(temp)/100;
    }
}
 
void BaroAltLib::get_cal_data(void)  // read 6 16 bit words of calibration from MS5611 PROM
{
    unsigned char i2c_buffer[2];  // buffer for baro i2c data

    if(readI2c(BARO_ADRS, 0xA2, 2, i2c_buffer) == 0)  C1 = (unsigned int)(i2c_buffer[0]<<8 | i2c_buffer[1]);  //read 2 bytes of baro data into i2c read buffer
    if(readI2c(BARO_ADRS, 0xA4, 2, i2c_buffer) == 0) C2 = (unsigned int)(i2c_buffer[0]<<8 | i2c_buffer[1]);   //transfer i2c buffer data into 24bit baro data
    if(readI2c(BARO_ADRS, 0xA6, 2, i2c_buffer) == 0) C3 = (unsigned int)(i2c_buffer[0]<<8 | i2c_buffer[1]);
    if(readI2c(BARO_ADRS, 0xA8, 2, i2c_buffer) == 0) C4 = (unsigned int)(i2c_buffer[0]<<8 | i2c_buffer[1]);
    if(readI2c(BARO_ADRS, 0xAA, 2, i2c_buffer) == 0) C5 = (unsigned int)(i2c_buffer[0]<<8 | i2c_buffer[1]);
    if(readI2c(BARO_ADRS, 0xAC, 2, i2c_buffer) == 0) C6 = (unsigned int)(i2c_buffer[0]<<8 | i2c_buffer[1]);
}

