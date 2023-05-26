//////////////////////////////////
// MPULib Classes
//////////////////////////////////

// c 2015 IowaKAPer / Mike LeDuc

MPULib::MPULib()   // class constructor
{                  // only run MPU i2c functions in non-interruptable routines to avoid i2c lock-up
    MPU_WhoAmI = 0;
    num_slave_bytes = 0;
    MPU_initialized = false;    // don't run MPU in real time interrupt routines until it has been initialized
}

void MPULib::initialize(unsigned char MPU_address, unsigned char mag_address)  // initialize MPU device
{
    delay(0.25);
    MPU_adrs = MPU_address;
    mag_adrs = mag_address;

    // configure MPU registers
    writeI2c(MPU_adrs , SMPLRT_DIV_REG, 0x00);    // 0x00: Set sample rate Fs divider to 1
    writeI2c(MPU_adrs , CONFIG_REG, 0x04);        // 0x04: Set gyro DLPF filter BW = 20hz, 9.9 msec delay, Fs = 1khz, temp BW = 20hz
    writeI2c(MPU_adrs , GYRO_CONFIG_REG, 0x10);   // 0x10: Gyro full scale to 1000dps, use DLPF
    writeI2c(MPU_adrs , ACCEL_CONFIG_REG, 0x08);  // 0x08: Set Accel FS to 4g.  (0x10 = 8g FS)
    writeI2c(MPU_adrs , PWR_MGMT1_REG, 0x01);     // 0x01: Select pll for clock when stable
    writeI2c(MPU_adrs , ACCEL_CONFIG2_REG, 0x04); // 0x04: Set accel DLPF filter to 20hz, Fs = 1khz, delay = 19.8 msec
    writeI2c(MPU_adrs, I2C_MST_CTRL, 0x0D);       // 0x0D: Set MPU aux i2c bus clock to 400 KHz
            
    delay(0.25);            // wait for MPU to stabilize after initializing
    MPU_initialized = true; // set flag to indicate MPU is initialized and can be used for computations
}

void MPULib::get_MPU_data(void)  // read raw MPU gyro, acceleration and temperature data and format into raw 16 bit format
{                                // also read autonomously unloaded i2c slave data if aux device is installed

    unsigned char i2c_buffer[14+num_slave_bytes];  // buffer for gyro, accel and temperature bytes from MPU
                                                   // local MPU has 14 bytes of data for accel, temperature and gyro, unload these plus any
                                                   // data autonomously unloaded from slave devices
    if(readI2c(MPU_adrs, MPU_DATA_REG, (14+num_slave_bytes), i2c_buffer) == 0)   //read bytes from MPU data buffer into i2c read buffer
    {
        // parse i2c read buffer to extract local MPU data
        local_x_accel = (float)((short)(i2c_buffer[0]<<8) | (short)i2c_buffer[1]);
        local_y_accel = (float)((short)(i2c_buffer[2]<<8) | (short)i2c_buffer[3]);
        local_z_accel = (float)((short)(i2c_buffer[4]<<8) | (short)i2c_buffer[5]);

        local_temperature = (float)((short)(i2c_buffer[6]<<8) | (short)i2c_buffer[7]);
        local_temperature = ((local_temperature - ROOM_TEMP_OFFSET) / TEMPERATURE_SCALE) + 21.0;  //rely on MPU filtering for "average" temperature
    
        local_x_rate = (float)((short)(i2c_buffer[8]<<8)  | (short)i2c_buffer[9]);
        local_y_rate = (float)((short)(i2c_buffer[10]<<8) | (short)i2c_buffer[11]);
        local_z_rate = (float)((short)(i2c_buffer[12]<<8) | (short)i2c_buffer[13]);

        if(tether_MPU_installed & num_slave_bytes == 14) // unload slave device data from local memory
        {
            slave_x_accel = (float)((short)(i2c_buffer[14]<<8) | (short)i2c_buffer[15]);
            slave_y_accel = (float)((short)(i2c_buffer[16]<<8) | (short)i2c_buffer[17]);
            slave_z_accel = (float)((short)(i2c_buffer[18]<<8) | (short)i2c_buffer[19]);

            slave_temperature = (float)((short)(i2c_buffer[20]<<8) | (short)i2c_buffer[21]);
            slave_temperature = ((slave_temperature - ROOM_TEMP_OFFSET) / TEMPERATURE_SCALE) + 21.0;  //rely on MPU filtering for "average" temperature

            slave_x_rate = (float)((short)(i2c_buffer[22]<<8)  | (short)i2c_buffer[23]);
            slave_y_rate = (float)((short)(i2c_buffer[24]<<8) | (short)i2c_buffer[25]);
            slave_z_rate = (float)((short)(i2c_buffer[26]<<8) | (short)i2c_buffer[27]);
        }
    }
       
}

void MPULib::get_MPU_WhoAmI(void)    // gets contents of WhoAmI register
{
    unsigned char i2c_buffer[1];
    if(readI2c(MPU_adrs, MPU_whoAmI_REG, 1, i2c_buffer) == 0) MPU_WhoAmI = i2c_buffer[0];
}

void MPULib::get_mag_WhoAmI(void)    // gets contents of WhoAmI register
{
    unsigned char i2c_buffer[1];
    if(readI2c(mag_adrs, MAG_ID_REG, 1, i2c_buffer) == 0) mag_WhoAmI = i2c_buffer[0];
}

void MPULib::bypass_mode_enable(void)   // enables i2c bypass mode and allows PIC to be master of the MPU aux i2c bus
{
    writeI2c(MPU_adrs, INT_PIN_CFG, 0x02);  // set i2c bypass enable bit
    writeI2c(MPU_adrs, USER_CTRL, 0x00);    // clear i2c I2C_MST_EN enable bit to allow PIC to be master of aux i2c bus
}

void MPULib:: bypass_mode_disable(void)  // disables i2c bypass mode and configures MPU to be master of it's aux i2c bus
{
   writeI2c(MPU_adrs, INT_PIN_CFG, 0x00);  // clear i2c bypass enable bit
   writeI2c(MPU_adrs, USER_CTRL, 0x20);    // set i2c I2C_MST_EN enable pin to allow the MPU to be master of the aux i2c bus
}
