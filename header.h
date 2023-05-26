// File:   header.h
//
// c 2015 IowaKAPer / Mike LeDuc
//

#ifndef HEADER_H
#define	HEADER_H

////////////////////////////////////////////
// Global Variable / Constant Declarations
////////////////////////////////////////////

    #define SYS_FREQ        40000000.0                // make sure this is not defined in another header file
    #define SYS_CLK_FREQ    SYS_FREQ                  // see system config #pragma settings above for sysclk
    #define PB_CLK_FREQ     20000000.0                // #pragma for FPBDIV sets PB clock divider
    #define ITER_PERIOD     0.0023                    // seconds, iteration frame rate

    #define DEFAULT_US_FREQ 1.0                       // hz, default unit status LED frequency
    #define NO_RADIO_FREQ 0.4
    #define SENSOR_FAILURE_FREQ 0.2
    #define LOGGING_US_FREQ 2.0
    #define FLASH_FULL_US_FREQ 0.3
    #define DATALOG_DISABLE_FREQ 3.0
    #define S95_FREQ 3.0
    #define ERASING_FLASH_FREQ 5.0
    #define GYRO_OFFSET_ERROR_FREQ 10

    float unit_status_freq = DEFAULT_US_FREQ;           // hz, unit status LED blink frequency

                                                        // T1_PS_1_X must be adjusted when timer1 is opened to match this prescale value
    #define T1_PRESCALE     1                           // timer1 prescaler for frame interrupt  (sets timer to 20 Mhz clk)
    #define T1_CLK_FREQ     (PB_CLK_FREQ / T1_PRESCALE) // timer1 timer clock frequency
    #define T1_PRELOAD      (ITER_PERIOD * T1_CLK_FREQ) // rollover time of timer. (timer1 starts at 0000 and counts up to T1_PRELOAD,
                                                        // and then generates an interrupt
    #define ESC_PULSE_PERIOD  0.002  // seconds, ESC PWM period
    #define T2_PRESCALE 4                                                 // timer2 prescaler from pb clock (sets timer2 to 5 Mhz clk)
    #define T2_PRELOAD  (ESC_PULSE_PERIOD * (PB_CLK_FREQ / T2_PRESCALE))  // timer2 rolls over at the servo pulse period
    #define T2_SEC_PER_BIT (1.0 / (PB_CLK_FREQ/T2_PRESCALE))              // timer2 seconds per bit

    #define T3_PRESCALE 8                                       // timer3 prescaler from pb clock (sets timer3 to 2.5 MHz clk)
    #define T3_PRELOAD 0xFFFF                                   // preload with max rollover period (26.2 msec)
    #define T3_SEC_PER_BIT (1.0 / (PB_CLK_FREQ/T3_PRESCALE))    // timer3 seconds per bit

    #define RX_MIN_VALID_PULSE 0.0008   // minimum accepted radio rx pulse width
    #define RX_MAX_VALID_PULSE 0.0023    // maximum accepted radio rx pulse width
    #define RX_MIN_PULSE_LIMIT 0.0009
    #define RX_MAX_PULSE_LIMIT 0.0021    // default pulse width to set when radio off
    #define RX_ACTIVE_TIMEOUT 600       // frame count to determine if radio transmitter has been turned off or rx reception has been lost
    #define THROTTLE_START_VALUE 1.0     // percent, throttle start pulsewidth threshold (no motor movement when joystick cmd is less than this pw
    #define MIN_PWR_UP_THROTTLE_VALUE 10.0    // percent, minimum throttle value during power up to allow motors to be enabled
                                             // helps insure that platform doesn't power up commanding motors if throttle is in non-minimum position

    unsigned int throttle_low_count = 0;    // number of consecutive frames that the throttle is less than the min power up value

    #define MIN_MOTOR_FLIGHT_CMD 17.5            // percent, minimum motor cmd during flight to ensure motors don't stop for small PID values
    float min_mtr_lim = MIN_MOTOR_FLIGHT_CMD;

    #define COMP_FILTR_COEFF_INIT 0.99              // platform and tether complimentary filter coeff for gyro and accel sensor fusion during power up
    #define COMP_FLTR_COEFF_STEADY_STATE 0.999      // platform and tether complimentary filter coefficient for gyro and accel sensor fusion steady state
                                                    // comp filter relies more on accel during power up to initialize angle calculation faster
    float comp_fltr_coeff;                          // complementary filter coefficient for sensor fusion
                                                    // comp filter time constant = (ITER_PERIOD * FLTR_COEFF)/(1-FLTR_COEFF)
                                                    // --> 0.999 FLTR_COEFF results in approx 2 sec comp filter time constant at 500 hz iter rate
                                                    // Note that values less than 0.999 for tether results in positive feedback from accelerometers in tether C/L

    // MPU-9250 register addresses
    #define PLATFORM_MPU_ADDRESS 0x68      // primary MPU-9250 i2c device address 110100 (assume AD0=1)- primary sensors
    #define TETHER_MPU_ADDRESS 0x69        // tether MPU-9250 i2c device address 110101 (assume AD0=0) - tether sensors
    #define SMPLRT_DIV_REG  25
    #define CONFIG_REG      26
    #define GYRO_CONFIG_REG 27
    #define ACCEL_CONFIG_REG 28
    #define ACCEL_CONFIG2_REG 29
    #define MPU_DATA_REG 59         // first register of MPU data buffer (zccel x/y/z, temperature, gyro x/y/z)
    #define PWR_MGMT1_REG 107
    #define I2C_MST_CTRL 36
    #define I2C_SLAVE_ADDR 37
    #define I2C_SLV0_REG 38
    #define I2C_SLV0_CTRL 39
    #define INT_PIN_CFG 55
    #define USER_CTRL 106
    #define MPU_whoAmI_REG 117      // 75 MPU-9250 WhoAmI register
    #define PLATFORM_MAG_ADDRESS    0x0C   // primary i2c magnetometer AK8963 I2C slave address
    #define TETHER_MAG_ADDRESS      0x0E   // tether i2c magnetometer address
    #define MAG_ID_REG  0x00        // Magnetometer ID register (should return 0x48 when read)
    #define I2C_MST_DELAY_CTRL 0x67
    #define PRIMARY_I2C_CLK_SPEED 0x16  // 0x16 sets PIC I2C speed to approx 400 Kbps

    #define BARO_ADRS 0x76          // MS5611 Barometer registers / commands
    #define BARO_RESET_CMD 0x1E
    #define START_BARO_ADC_CMD 0x48
    #define START_TEMP_ADC_CMD 0x58
    #define BARO_ADC_REG 0x00

    // MPU9250 constants
    #define GYRO_FS 1000.0               // degrees per second, gyro full scale
    #define ACCEL_FS 4.0                 // g, accel full scale
    #define TEMPERATURE_SCALE 333.87     // lsb/deg C, temperature sensor
    #define ROOM_TEMP_OFFSET  21.0       // degC, MPU temperature sensor offset at 21C
    #define RAD_TO_DEG (180.0/3.14159)   // radians to degrees conversion factor
    #define DEG_TO_RAD (3.14149 / 180.0)

    float platform_x_gyro_offset = 0.0;             // degrees per sec
    #define PLATFORM_X_GYRO_TC_SLOPE -.0142         // temperature coefficient slope
    #define PLATFORM_X_GYRO_TC_Y_INT 0.7511         // temperature coefficient y intercept
    #define PLATFORM_X_GYRO_GAIN 1.0

    float platform_y_gyro_offset = 0.0;             // degrees per sec
    #define PLATFORM_Y_GYRO_TC_SLOPE -0.0142        // temperature coefficient slope
    #define PLATFORM_Y_GYRO_TC_Y_INT 0.7511         // temperature coefficient y intercept
    #define PLATFORM_Y_GYRO_GAIN 1.0

    float platform_z_gyro_offset = 0.0;             // degrees per sec
    #define PLATFORM_Z_GYRO_TC_SLOPE -0.0016        // temperature coefficient slope
    #define PLATFORM_Z_GYRO_TC_Y_INT 0.5641         // temperature coefficient y intercept
    #define PLATFORM_Z_GYRO_GAIN 0.991

    float platform_x_accel_offset = 0.0;            // g
    #define PLATFORM_X_ACCEL_TC_SLOPE -0.0005       // temperature coefficient slope
    #define PLATFORM_X_ACCEL_TC_Y_INT 0.0070         // temperature coefficient y intercept
    #define PLATFORM_X_ACCEL_GAIN 1.0

    float platform_y_accel_offset = 0.0;            // g
    #define PLATFORM_Y_ACCEL_TC_SLOPE 0.0006        // temperature coefficient slope
    #define PLATFORM_Y_ACCEL_TC_Y_INT -0.019       // temperature coefficient y intercept
    #define PLATFORM_Y_ACCEL_GAIN 1.0

    float platform_z_accel_offset = 0.0;            // g
    #define PLATFORM_Z_ACCEL_TC_SLOPE 0.0           // temperature coefficient slope
    #define PLATFORM_Z_ACCEL_TC_Y_INT 0.0           // temperature coefficient y intercept
    #define PLATFORM_Z_ACCEL_GAIN 1.0

    float tether_x_gyro_offset = 0.0;               // degrees per sec
    #define TETHER_X_GYRO_TC_SLOPE 0.0              // temperature coefficient slope
    #define TETHER_X_GYRO_TC_Y_INT -1.1085815       // temperature coefficient y intercept
    #define TETHER_X_GYRO_GAIN 1.0

    float tether_z_gyro_offset = 0.0;               // degrees per sec
    #define TETHER_Z_GYRO_TC_SLOPE 0.0              // temperature coefficient slope
    #define TETHER_Z_GYRO_TC_Y_INT -0.14959717      // temperature coefficient y intercept
    #define TETHER_Z_GYRO_GAIN 1.0

    float tether_x_accel_offset = 0.0;              // g
    #define TETHER_X_ACCEL_TC_SLOPE -0.0002         // temperature coefficient slope
    #define TETHER_X_ACCEL_TC_Y_INT -0.009465      // temperature coefficient y intercept
    #define TETHER_X_ACCEL_GAIN 1.0

    float tether_z_accel_offset = 0.0;              // g
    #define TETHER_Z_ACCEL_TC_SLOPE 0.0009          // temperature coefficient slope
    #define TETHER_Z_ACCEL_TC_Y_INT -0.03291    // temperature coefficient y intercept
    #define TETHER_Z_ACCEL_GAIN 1.0

    unsigned char roll_ctrl_enable = false;         // power up with motors disabled
    unsigned char pitch_ctrl_enable = false;
    unsigned char yaw_ctrl_enable = false;

    // Pitch PID constants
    #define PITCH_RATE_KP       0.4    //inner rate loop PID constants
    #define PITCH_RATE_KI       0.0     
    #define PITCH_RATE_KD       0.002  //0.004
    #define PITCH_RATE_I_LIM    0.0     
    #define PITCH_RATE_GAIN     1.0     

    #define PITCH_ATTITUDE_KP           3.0       //outer position loop PID constants
    #define PITCH_ATTITUDE_KI           3.0       //3 results in ~ 5 sec capture
    #define PITCH_ATTITUDE_KD           0.05
    #define PITCH_ATTITUDE_I_LIM        10.0      //15.0 used to overcome long term platform imbalance, motor torque mismatch and wind

    // Roll PID constants
    #define ROLL_RATE_KP       0.4   // inner rate loop PID constants
    #define ROLL_RATE_KI       0.0
    #define ROLL_RATE_KD       0.002 //0.004
    #define ROLL_RATE_I_LIM    0.0
    #define ROLL_RATE_GAIN     1.0

    #define ROLL_ATTITUDE_KP           3.0      //4 // outer position loop PID constants
    #define ROLL_ATTITUDE_KI           3.0      // 3 results in ~ 5 sec capture
    #define ROLL_ATTITUDE_KD           0.05
    #define ROLL_ATTITUDE_I_LIM        10.0     // used to overcome long term platform imbalance, motor torque mismatch and wind

    // Yaw PID constants
    #define YAW_RATE_KP       2.0   // inner rate loop PID constants
    #define YAW_RATE_KI       0.0
    #define YAW_RATE_KD       0.00
    #define YAW_RATE_I_LIM    0.0
    #define YAW_RATE_GAIN     1.0

    #define YAW_ATTITUDE_KP           5.0       // outer position loop PID constants
    #define YAW_ATTITUDE_KI           0.0
    #define YAW_ATTITUDE_KD           0.0
    #define YAW_ATTITUDE_I_LIM        0.0

//*********************************************************************
   // xyz damper control law constants

    #define LIN_ACCEL_LIM 20.0  // meters/sec/sec limit
    #define VEL_LIM 2.0        // meters/sec

    float DAMPER_P_GAIN = 5.0;
    float DAMPER_I_GAIN = 15.0;
    #define DAMPER_LIM 10.0
    #define DAMPER_LPF_TAU 0.01
    #define DAMPER_GAIN 1.0

    #define TETHER_COMP_FLTR_COEFF 0.999
 
    // tether hold control law constants
    #define T_MIN_PITCH_RATE_GAIN   0.3
    #define T_MAX_PITCH_RATE_GAIN   0.6
    #define T_PITCH_RATE_KNEE       10.0
    #define T_PITCH_RATE_SLOPE      0.1

    #define T_MIN_ROLL_RATE_GAIN    0.3
    #define T_MAX_ROLL_RATE_GAIN    0.6
    #define T_ROLL_RATE_KNEE        10.0
    #define T_ROLL_RATE_SLOPE       0.1

    // Tether PID constants
    #define T_RATE_KP       1.0      // inner rate loop PID constants
    #define T_RATE_KI       0.0
    #define T_RATE_KD       0.0
    #define T_RATE_I_LIM    0.0

    #define T_ATTITUDE_KP           0.5    // outer position loop PID constants
    #define T_ATTITUDE_KI           0.0 
    #define T_ATTITUDE_KD           0.0
    #define T_ATTITUDE_I_LIM        0.0

    #define TETHER_ANGLE_LPF_TAU 0.2
    #define TETHER_RATE_LPF_TAU 0.1
    #define TETHER_RATE_GAIN_LPF_TAU 0.05

    #define DEFAULT_TETHER_ANGLE_RATE_LIMIT 20.0    //20.0 // deg/sec max allowed control law rate input
    #define DEFAULT_TETHER_RATE_LIMIT 20.0          // deg/sec max allowed control output of tether PID

    #define TETHER_GAIN 1.0
//********************************************************************

     // accel z hold PID constants
    #define Z_ACCEL_KP       0.0   // loop PID constants
    #define Z_ACCEL_KI       0.0
    #define Z_ACCEL_KD       0.0
    #define Z_ACCEL_I_LIM    0.0
 
    // global variables
    
    float tether_pitch_rate_gain = 0.0;
    float tether_roll_rate_gain = 0.0;
    
    volatile float absolute_time = 0.0;	// seconds, time in seconds since power up
    float high_water = 0.0;             // timer bits, longest interrupt duration
    float int_time = 0.0;               // seconds, elapsed time to execute this interrupt routine
    float max_int_time = 0.0;		// seconds, max time taken to execute interrupt procedure (high water mark)
					// this max time should be at least 0.5ms less than iteration period to allow
                                        // adequate background execution time
    unsigned char frame_overflow = false;	// true indicates a frame overflow has occured
    unsigned char overflow_count = 0;
    unsigned int frame_count = 1;  // sub frame counter for multi-rate execution

    unsigned char platform_initialized = false; // true indicates all platform functions have been initialized and rx, sensors, control laws can start

    float us_toggle_time = 0.0;           // absolute time of last unit status LED toggle

    float pitch_angle = 0.0;              // degrees, platform angles
    float roll_angle = 0.0;               // degrees
    float yaw_angle = 0.0;                // degrees

    float sin_yaw_delta = 0.0;
    float cos_yaw_delta = 1.0;
    float sin_yaw = 0.0;
    float cos_yaw = 1.0;
  
    float tether_pitch_angle = 0.0;       // degrees, tether angles
    float tether_roll_angle = 0.0;        // degrees

    unsigned char throttle_enable = false;      // disable throttle control at power up until stick is in minimum throttle position
    volatile float throttle_cmd = 5.0;    // percent, throttle esc pulse width command  (initialize to non zero value so that motors don't
    volatile float throttle_rx_cmd = 5.0; // arm until throttle is brought to min stick position
                                          // power up initialized value should be >  MIN_PWR_UP_THROTTLE_VALUE to keep motor off if powered up with
                                          // non zero joystick value
 
    volatile float pitch_rx_cmd_raw = 0.0; // degrees, rx pitch cmd before yaw compensation
    volatile float roll_rx_cmd_raw = 0.0; // degrees, rx pitch cmd before yaw compensation 
    volatile float pitch_rx_cmd = 0.0;     // degrees, desired platform pitch angle from radio compensated for yaw angle
    volatile float roll_rx_cmd = 0.0;      // degrees, desired platform roll angle from radio compensated for yaw angle
    volatile float yaw_rx_rate_cmd = 0.00;  // deg/sec, yaw rate command from radio tx joystick position
    volatile float yaw_rx_angle_cmd = 0.00; // degrees, yaw angle command from radio tx joystick from integrated yaw rate command

    volatile float ch5_rx_cmd;              // percent, aux 1 input from radio

    volatile float pitch_rx_offset = 0.0;    // degrees, power up joystick position
    volatile float roll_rx_offset = 0.0;
    volatile float yaw_rx_offset = 0.0;

    volatile float pitch_cmd = 0.0;
    volatile float roll_cmd = 0.0;
    volatile float yaw_cmd = 0.0;

    float rx_integrator_delta = 0.0;         // used to integrate pitch and roll rx angle cmd for tether control

    #define PITCH_RX_DEAD_ZONE 4.0    // percent, pitch joystick deadzone
    #define ROLL_RX_DEAD_ZONE  4.0    // percent, roll joystick deadzone
    #define YAW_RX_DEAD_ZONE   8.0  // percent, yaw joystick deadzone (set to 1.0 if not using yaw trim to enable flight mode)
                                 
    float tether_pitch_cmd = 0.0;  // tether pitch cmd;
    float tether_roll_cmd = 0.0;  // tether roll cmd;

    float pitch_front_mtr_cmd = 0;      // percent motor cmd
    float pitch_rear_mtr_cmd = 0;       // percent motor cmd
    float roll_left_mtr_cmd = 0;        // percent motor cmd
    float roll_right_mtr_cmd = 0;       // percent motor cmd

    volatile float vib_throttle_pw = 0.001;          // seconds, pulse width of throttle for platform vibration test (0.001 - 0.002)
 
     // sensor offset data sampling parameters
    #define MAX_OFFSET_SAMPLES 300 // 300
    float offset_buf[13][MAX_OFFSET_SAMPLES];
    unsigned int offset_sample_count = 0;
    unsigned char offset_trigger = false;
    float last_offset_sample_time = 0.0; 
    unsigned char get_sensor_offsets_enable = false;
     
    unsigned char calibrate_ESCs_enable = false;        // set to true in main to calibrate ESCs on power up
    unsigned char measure_platform_vib_enable = false;  // set to true in main to measure platform vibration on power up
    unsigned char measure_step_response_enable = false;        // set to true in main to measure platform step response on power up

    unsigned char rx_override = false;                  // true indicates that platform tests will control rx values

    float init_pressure = 0;

    unsigned char tether_MPU_installed = false;

    unsigned char tether_mode = false;

    float x_linear_accel = 0.0;     // g, linear acceleration in x axis
    float y_linear_accel = 0.0;     // g, linear acceleration in y axis
    float z_linear_accel = 0.0;     // g, linear acceleration in z axis

    float x_linear_accel_last;
    float y_linear_accel_last;

    float sin_pitch;
    float sin_roll;
   
    float x_velocity = 0.0;     // platform linear velocities without integrator drift compensation
    float y_velocity = 0.0;    

    float x_damper = 0.0;       // platform linear movement damper control law outputs
    float y_damper = 0.0;

    float tether_angle_rate_limit = DEFAULT_TETHER_ANGLE_RATE_LIMIT; // deg/sec, adjust as function of altitude
    float tether_rate_limit = DEFAULT_TETHER_RATE_LIMIT; // deg/sec, adjust as function of altitude

    float altitude = 1.2;  // meters, platform altitude
    float tether_y_velocity;
    float tether_x_velocity;
    #define DPS_TO_MPS (2*3.1415/360)  //factor for converting deg/sec to meters/sec

    #define UP 2        // tx aux switch up position state  (tether mode)
    #define MIDDLE 1    // tx aux switch middle position state (damper mode)
    #define DOWN 0      // tx aux switch down position state (manual mode)

    unsigned char mode_sw_posn = DOWN;  // tx aux switch b position

    float tether_pitch_rate_compensated;
    float tether_roll_rate_compensated;

    float tx_altitude_lim = 1.5;  //meters, Tx VR knob on transmitter (0 = full ccw, 100 meters = full cw

    float pitch_trim = 0.0;   //degrees, tx joystick trim pot
    float roll_trim = 0.0;    //degrees, tx joystick trim pot

    unsigned char ground_altitude_calculated = false; // flag to indicate if ground altitude determined
    float ground_altitude = 0.0;  //meters, baro altitude at launch point
    float platform_altitude = 0.0; //meters,

    volatile unsigned char pan_mode = false;

    volatile float pan_cmd = 0.0;

    float ctrl_stick = 0.0; //percent, tx throttle joystick position (used to control various modes of TQ)

    unsigned int yaw_sync_count = 0;
    unsigned int yaw_sync_arm = false;

    float pan_increment = 0.0;       // degrees, increment to pan platform wihen in pan_mode

    unsigned char integrator_hold = false;
    
    volatile float flight_throttle_value = 66.0;  //throttle value for propter platform lift based on camera payload

    volatile unsigned char erasing_flash = false;    //flag to indicate flash memory is being erased
    volatile unsigned char sensors_valid = true;

    float tilt_cmd = 0;    //degrees, tilt servo angle cmd
    float tilt_svo_pw;

    float plat_x_gyro_pu_offset = 0.0;
    float plat_y_gyro_pu_offset = 0.0;
    float plat_z_gyro_pu_offset = 0.0;
    float tether_x_gyro_pu_offset = 0.0;
    float tether_z_gyro_pu_offset = 0.0;
    unsigned char gyro_pu_offset_error = false;

    int abort_I2C = false;
    int abort_I2C_count = 0;
    int last_abort_I2C_cause = 0;

    unsigned char thrust_test_enable = false;

    #define RICOH_GR 0
    #define CANON_S95 1
    unsigned char camera_type = RICOH_GR;

    //datalogging parameters
    char datalogging_enabled = true;
    unsigned char annunciate_datalog_disable = false;

    unsigned int uart_num_log_parameters;
    unsigned int uart_log_sample_hz;
    float last_uart_sample_time = 0.0;
    float uart_log_data[10];     // array to store data to be logged to uart
    unsigned int uart_log_index;
    int uart_data;
    char uart_string[8];

    unsigned char trigger = false;
    float last_flash_sample_time = 0.0;
    unsigned int flash_num_log_parameters;
    unsigned int flash_log_sample_hz;
    char flash_record_created = false;
    char flash_full = false;
    unsigned int flash_erase_cmd_counter = 0;
    float flash_log_data[33];     // array to store data to be logged in flash
    unsigned int flash_log_index;

    unsigned int flash_adrs;
    #define FIRST_FLASH_ADRS 0x1D011000  // first address of flash memory to log data into
    #define LAST_FLASH_ADRS  0x1D03F000  // this provides 188 KB of log data. pages above 0x1D03F000 address are used by the system
                                        // pgm memory spacelimited to 69632 (0x11000)!!!!  programs larger than this will crash when datalogging
    #define FLASH_LOG_SCALE_FACTOR 10000      // multiplier used to convert float data to int for flash log storage

    #define TILT_OFFSET 10              // degrees, tilt servo rigging offset
    char vert_pic_arm = false;          // flag to take vertical picture during first pan sequence

    char sensor_test_in_progress = false;

#endif	/* HEADER_H */

