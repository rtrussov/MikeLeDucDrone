//////////////////////////////////
// Tether Quad Dataloging
//////////////////////////////////

// c 2016, 2017 IowaKAPer / Mike LeDuc

void datalog_flash(void)      // log data to flash
{
    //////////////////////////////////////////////////////////////////
    // To create log file for Excel:
    //      From Debugger or production build:
    //          Pause button (if in debug)
    //          Hold in Reset button (if in production build)
    //          Read Device Memory Main Project (into Flash Memory window)
    //      From Flash Memory Window:
    //          GoTo Adrs 0x1D01_0000 (first logged Flash data location)
    //          *** assumes program data size <65.5KB ****
    //          Right mouse on data and "Export Table" to .mch file
    //          Select single column output option
    //      From Excel:
    //          Open .mch file
    //          Fixed Width, column width set to 50, General
    /////////////////////////////////////////////////////////////

    flash_log_sample_hz = 2;          // log sample frequency
    flash_num_log_parameters = 32;    // number of parameters to log

    if((ctrl_stick > 10) & (absolute_time > 8) & (throttle_enable)) trigger = true;   // trigger condition
    
    if(trigger & !flash_record_created) create_flash_record();  // create flash record header

    if(trigger & flash_record_created & ((absolute_time - last_flash_sample_time) > (1.0/flash_log_sample_hz - 0.001)))   // sample parameters at specified rate (subtract 0.001 from desired sample interval due to rtc quantization)
    {
        // store log parameters in array

        // flight data list
        flash_log_data[1]  = absolute_time;
        flash_log_data[2]  = throttle_cmd - pitch_rate_PID.out - yaw_rate_PID.out;
        flash_log_data[3]  = throttle_cmd + pitch_rate_PID.out - yaw_rate_PID.out;
        flash_log_data[4]  = throttle_cmd + roll_rate_PID.out + yaw_rate_PID.out;
        flash_log_data[5]  = throttle_cmd - roll_rate_PID.out + yaw_rate_PID.out;
        flash_log_data[6]  = pitch_attitude_PID.out;
        flash_log_data[7]  = pitch_rate_PID.out;
        flash_log_data[8]  = roll_attitude_PID.out;
        flash_log_data[9]  = roll_rate_PID.out;
        flash_log_data[10] = yaw_attitude_PID.out;
        flash_log_data[11] = yaw_rate_PID.out;
        flash_log_data[12] = pitch_cmd;
        flash_log_data[13] = roll_cmd;
        flash_log_data[14] = yaw_cmd;
        flash_log_data[15] = pitch_angle;
        flash_log_data[16] = roll_angle;
        flash_log_data[17] = yaw_angle;
        flash_log_data[18] = pitch_rx_cmd_raw;
        flash_log_data[19] = roll_rx_cmd_raw;
        flash_log_data[20] = yaw_rx_rate_cmd;
        flash_log_data[21] = tether_pitch_angle;
        flash_log_data[22] = tether_roll_angle;
        flash_log_data[23] = tether_pitch_PID_rlm.out;
        flash_log_data[24] = tether_roll_PID_rlm.out;
        flash_log_data[25] = y_damper_lpf.out;
        flash_log_data[26] = x_damper_lpf.out;
        flash_log_data[27] = throttle_rx_cmd;
        flash_log_data[28] = baro_alt_lpf.out;
        flash_log_data[29] = platform_altitude;
        flash_log_data[30] = ctrl_stick;
        flash_log_data[31] = mode_sw_posn + yaw_rate_lpf.out * 0.01;
        flash_log_data[32] = overflow_count + max_int_time * 100 + (abort_I2C_count*10);

        if(ctrl_stick < 50 & throttle_rx_cmd < 25)  //store temperatures and gyro offsets at low stick and low throttle
        {
            flash_log_data[2] = platform_MPU9250.local_temperature;
            flash_log_data[3] = platform_MPU9250.slave_temperature;
            flash_log_data[15] = plat_x_gyro_pu_offset;
            flash_log_data[16] = plat_y_gyro_pu_offset;
            flash_log_data[17] = plat_z_gyro_pu_offset;
            flash_log_data[18] = platform_x_gyro.average;
            flash_log_data[19] = platform_y_gyro.average;
            flash_log_data[20] = platform_z_gyro.average;
            flash_log_data[21] = tether_x_gyro_pu_offset;
            flash_log_data[22] = tether_z_gyro_pu_offset;
            flash_log_data[23] = tether_x_gyro.average;
            flash_log_data[24] = tether_z_gyro.average;
        }

        last_flash_sample_time = absolute_time;
        flash_log_index = 0;
     }
        // Platform Accelerometer log parameters (9 parameters, 40 Hz)
        //flash_log_data[1]  = absolute_time;
        //flash_log_data[2]  = platform_MPU9250.local_temperature;
        //flash_log_data[3]  = platform_x_accel.raw_average * 1000;
        //flash_log_data[4]  = platform_y_accel.raw_average * 1000;
        //flash_log_data[5]  = throttle_rx_cmd;
        //flash_log_data[6]  = pitch_angle;
        //flash_log_data[7]  = roll_angle;
        //flash_log_data[8]  = tether_pitch_angle;
        //flash_log_data[9]  = tether_roll_angle;

    // write log data to FLASH
    if(!flash_full & trigger & flash_record_created & (flash_log_index < flash_num_log_parameters))  
    {
        write_flash((flash_adrs+=4), flash_log_data[++flash_log_index]); // write log data and advance address 4 bytes (1 word)
    }
    //***************************************
        //**** code for erasing one page of flash memory (1024 bytes) ****
            //unsigned int erase_status;  // 0 = successful page erase
            //if(pitch_rx_cmd > 10)  erase_status = NVMErasePage((void*) 0x1D010000);  // erase 1 page (1024 bytes) each page erase takes ~2.5msec
                                                                                       // NVMErasePage returns 0 if completed sucessfully
            //if(pitch_rx_cmd > -10) NVMErasePage((void*) 0xBD00F400);

        //**** code for reading flash data ****
            //unsigned int * flash_ptr = (unsigned int*)0x9D00F000;  //pointer to beginning of KSEG0 flash memory space
                                                                     //must use KSEG0 virtual address space (0x9D0xxxxx)to allow user (i.e. program code) to access flash memory
                                                                     //other virtual address spaces of flash are reserved for kernel access only and can't be accessed by user code
            //unsigned int flash_data;
            //unsigned int flash_data1;

            //flash_data = *(flash_ptr);
            //flash_data1 = *(flash_ptr + 1); //increment unsigned int pointer by 1 advances 4 bytes in memory because unsigned int is 4 bytes long)
    //**************************************
}

void create_flash_record()
 {
    flash_adrs = FIRST_FLASH_ADRS;  // base adrs of logged data
                                    // Locating flash log data at 1D011000 allows:
                                    //    69KB of program code
                                    //    188KB of flash log data
                                    // note that last 16 bytes of Flash starting at 1D03FFF0 must be 0xFF to indicate eid of file for input parcer
                                    // BD000000 is KSEG1 virtual adrs of beginning of Flash, 1D000000 is phys adrs of same location)
    write_flash(flash_adrs, flash_num_log_parameters);        // store number of logged parameters
    write_flash(flash_adrs+=4, flash_log_sample_hz);          // store sample rate

    flash_record_created = true;    // header created, set flag to begin logging parameter data

    flash_adrs+=8;                  // advance adrs 3 words to allow header data growth
    flash_log_index = 100;                // set flash_log_index to large value to disable loggind until flash_log_data array is filled
}

unsigned int write_flash(unsigned int adrs, float data)
{                                           // convert flash log data to 32bit "signed" integer
                                            // bits 0:31 are unipolar data
                                            // bit 32 is sign bit (1 if negative)
                                            // input data is multiplied by 10000 to allow floating point decimal content to be stored in integer format
                                            // returns 0 if write completed successfully
    unsigned int flash_data;


    // if(log_index < 31) data = FLAHS_LOG_SCALE_FACTOR * data;  // do not scale baro data
    data = FLASH_LOG_SCALE_FACTOR * data;  // scale floating point fraction for conversion to integer;

    if (data < 0) // check to see if data is negative (2's compliment format)
    {
       data = -data;  // convert to positive number (to clear sign bit)
       flash_data = (unsigned int)data | 0x80000000;   //convert to int and set MSB to 1 if negative number (used by TQ_Plot excel routine to determine if number is negative)
       if(flash_data == 0x80000000) flash_data = 0;    //clear sign bit if data = 0 to eliminate errors in excel TQ_Plot macro;
    }
    else flash_data = (unsigned int)data;   //data is positive so MSB sign bit is 0

    if(adrs < LAST_FLASH_ADRS)
    {
        unsigned int write_status = NVMWriteWord((void*)adrs, flash_data);  // write flash data if flash is not full
    }                            // NVMWriteWord routine returns unsigned int for status (0=write completed successfully)
                                // temporary variable "write_status" is used to recieve write routine status bit
    else flash_full = true; // set flash full flag to stop logging
}

void erase_flash(void)
{
   unsigned int erase_status;                      // 0 = successful page erase
   unsigned int page_adrs = FIRST_FLASH_ADRS;      // address of first page of NVM memory

   while (page_adrs < (LAST_FLASH_ADRS - 0x3FF))           // loop erase routine until all pages of NVM are erased
   {
       erase_status = NVMErasePage((void*)page_adrs);   // erase 1 page (1024 bytes) each page erase takes ~2.5msec
                                                        // NVMErasePage returns 0 if completed sucessfully
       page_adrs += 0x400;                              // increment adress by 1 page (1024 bytes)
   }
   erasing_flash = true;    // blink unit status to indicate flash is erased
   while(true);             // flash erase caused frame overflow, so hang in endless loop until power cycle to clear frame overrun
}

void fastPutsUART2(const char* buffer)    //variant of putsUART2() modified to write to UART tx buffer without waiting for data to tx
{                                         //standard putsUART() is slow because it waits for each char to be transmitted before writing each char to the UART tx register
                                          //PIC UART tx buffer is 8 deep so must limit transmitted string size to 8 char or less to prevent tx buffer overflow and loss of data

    #define fastPutcUART2(c) fastWriteUART2((int)(c))  //macro variant of putcUART2() modified to write to UART tx buffer without waiting for data to tx
                                                       //PIC UART tx buffer is 8 deep so must limit transmitted string size to 8 char or less to prevent loss of data
    #define fastWriteUART2(data) U2TXREG = (data)      //macro variant of WriteUART2() modified to write to UART tx buffer without waiting for data to tx

    while(*buffer !=0)
    {
        fastPutcUART2(*buffer++);
    }
}

void datalog_uart()  //log data to RS-232 COM port (UART2)             
{
    //datalogger format for plotting multiple channels of data to MakerPlog WIN software is <nnn, nnn, nnn, nnn cr> so insert comma after each channel of data and a carrage return <cr> (\r) after the last channel of data
    uart_log_sample_hz = 20;        // log sample frequency
    uart_num_log_parameters = 10;   // MakerPlot max is 10 analog parameters

    if((absolute_time - last_uart_sample_time) > (1.0/uart_log_sample_hz - 0.001))   // sample parameters at specified rate (subtract 0.001 from desired sample interval due to rtc quantization)
    {
        // uart data list 
        // data is sent in 8 byte packets.  1 byte for sign, 1 byte for separator and 6 bytes of data
        // multiply data by appropriate power of 10 to get 6 digits of displayed data
        uart_log_data[1]  = yaw_angle * 10000;
        uart_log_data[2]  = x_velocity * 10000;
        uart_log_data[3]  = y_velocity * 10000;
        uart_log_data[4]  = tether_pitch_angle * 10000;
        uart_log_data[5]  = tether_roll_angle  * 10000;
        uart_log_data[6]  = pitch_angle * 10000;
        uart_log_data[7]  = roll_angle * 10000;
        uart_log_data[8]  = integrator_hold;
        uart_log_data[9]  = platform_x_accel_angle_lpf.out * 10000;
        uart_log_data[10]  = platform_y_accel_angle_lpf.out * 10000;

        last_uart_sample_time = absolute_time;
        uart_log_index = 1;
    }

    // write log data to uart
    if(uart_log_index < uart_num_log_parameters + 1)
    {
       build_uart_string();
       fastPutsUART2(uart_string); //write entire string to PIC UART tx buffer at once (no delays)
                                   //PIC UART tx buffer is 8 deep so must limit transmitted string size to 8 char or less to prevent loss of data
    }
    uart_log_index++;
}

void build_uart_string()  //create an 8 byte ASCII string with
                          //   first byte= sign
                          //   last byte separator (comma or carriage return)
                          //   middle bytes (byte 2 - 7) data (6 bytes of data)
{
    uart_data = (int) uart_log_data[uart_log_index];  // convert floating point log data to integer

    if (uart_data < 0) uart_string[0] = 45;  //negative number, set sign byte negative (ASCII "-")
    else uart_string[0] = 48; //positive number so set sign byte to zero (ASCII "0")

    uart_data = abs(uart_data); //strip off sign bit for conversion of integer to ASCII next
    
    if (uart_log_index == (uart_num_log_parameters)) uart_string[7] = 13;  //final parameter so set last character to carriage return (ASCII "CR")
    else uart_string[7] = 44; //not final parameter, so set last character to comma (ASCII ",")

    for(int i=6; i>0; i--)  //convert data payload to ASCII one character at a time
    {
        uart_string[i] = (uint8_t) ((uart_data % 10UL) + '0');  //convert each byte to ASCII number
        uart_data /= 10;                                        //keep dividing by 10 to get all digits
    }
}