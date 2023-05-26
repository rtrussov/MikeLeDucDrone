////////////////////////////////////////////
// Class Declarations
////////////////////////////////////////////

// c 2015 IowaKAPer / Mike LeDuc

class MPULib // Invensense MPU-9250 library functions
    {
    public:
        MPULib();
        void initialize(unsigned char MPU_address, unsigned char mag_address);     // initialize MPU
        void get_MPU_data();   // read MPU gyro, accel and temperature data into i2c buffer
        void get_mag_data();   // read MPU magnetometer data into i2c buffer
        void get_MPU_WhoAmI();
        void get_mag_WhoAmI();
        void bypass_mode_enable();
        void bypass_mode_disable();
       
        float local_x_rate;       // raw 16 bit sensor value of rate gyro output
        float local_y_rate;
        float local_z_rate;
        float local_x_accel;      // raw 16 bit sensor value of acceleration output
        float local_y_accel;
        float local_z_accel;
        float local_temperature;      // degC, filtered MPU temperature

        float slave_x_rate;       // raw 16 bit sensor value of rate gyro output
        float slave_y_rate;
        float slave_z_rate;
        float slave_x_accel;      // raw 16 bit sensor value of acceleration output
        float slave_y_accel;
        float slave_z_accel;
        float slave_temperature;      // degC, filtered MPU temperature
 
        unsigned char MPU_adrs;
        unsigned char mag_adrs;
        unsigned char MPU_WhoAmI;
        unsigned char mag_WhoAmI;
        unsigned char num_slave_bytes;
        unsigned char MPU_initialized;  //flag to indicate MPU has been initialized and it can be used

    };

class Gyro  // data structure for MPU gyro information
    {
    public:
        Gyro();
        void update(float raw_rate, float offset, float gain_adj);

        float delta;    // degrees, angle change since last sample
        float rate;     // deg/sed, current rate
        float rate_last;// deg/sec rate from last iteration
        float average;  // deg/sec power up average of gyro rate with offset compensation
        float max;      // deg/sec maximum recorded gyro rate since last power up
        float max_chg;  // deg/sec maximum difference between samples (used to determine noise in system)
        float raw_average; // deg/sec running average of gyro rate without offset compensation

    private:
        unsigned int num_samples;  // number of samples used in power up average calculation
        float accum;   // accumulated sum of all rate values (used to calculate average rate)
        float raw_accum;
    };

class Accel  // data structure for MPU accelerometer information
    {
    public:
        Accel();
        void update(float raw_accel, float offset, float gain_adj);

        float angle;      // degrees, current accelerometer angle
        float accel;      // g, current accelerometer scaled in g's
        float accel_last; // g, accelerometer value from previous frame
        float average;    // g, power up average of accel g with offset compensation
        float max;        // g, maximum recorded accel g's since last power up
        float max_chg;    // g, maximum difference between samples (used to determine noise in system)
        float raw_average; // deg/sec running average of accel without offset compensation


    private:
        unsigned int num_samples;  // number of samples used in power up average calculation
        float accum;      // accumulated sum of all g values (used to calculate average g force)
        float raw_accum;
    };

class PID // PID Control Law
    {
    public:
        PID();
        void set_parameters(float kp, float ki, float kd, float i_lim);    // set PID parameters
        void reset(void);
        void compute(float cmd_in, float fdbk_in);         // PID (Proportional, Integral, Differential) controller
        float error;
        float P;
        float I;
        float D;
        float out;
        
    private:
        float Kp;  // PID proportional gain constant
        float Ki;  // PID integral gain constant
        float Kd;  // PID differential or damping gain constant
        float Ilim;  // PID differential rate limit
        float fdbk_in_last;      // fdbk_in value from last iteration
    };       

 class radio_rx // radio receiver library
    {
    public:
        radio_rx();
        void get_pw(unsigned char data_ready, int rx_FIFO);     // get the latest rx pulse width if available
        float pulse_width;       // seconds, pulse width of latest received data
        unsigned char channel_active;   // flag to indicate that the radio is receiving command pulses from the transmitter
        unsigned int no_rx_data_count;  // number of times this routine has been called and no rx data is present
       
        int pulse_start_time; // seconds, timer 3 value when pulse beginning edge detected
        int pulse_start_time_last;
        int pulse_end_time; // seconds, timer 3 value when pulse end edge detected
        float timer_interval;   // timer count of current pulse
    };

class low_pass_filter // low pass filter
    {
    public:
        low_pass_filter();
        void compute(float in, float time_constant);
        void initialize(float initial_value);  // sets filter to initial value, filter time constant in seconds
        float out;       // filter output  (store as double precision for increased resolution for next iteration)

    private:
        double out_last; // previous output (store as double precision for increased resolution for next iteration)
        float in_last;   // previous input
        float alpha;        //filter constant
    };

class hi_pass_filter // high pass filter
    {
    public:
        hi_pass_filter();
        void compute(float in, float time_constant);
        void initialize(float initial_value);  // sets filter to initial value, time constant in seconds
        float out;       // filter output (store as double precision for increased resolution for next iteration)

    private:
        double out_last; // previous output (store as double precision for increased resolution for next iteration)
        float in_last;  // previous input
        float alpha;       // filter constant
    };

class BaroAltLib // MS-5611 barometer library functions (** assumes 25C for all baro calculations)
    {
    public:
        BaroAltLib();
        void reset();                   // initialize baro
        void start_baro_conversion();   // initiate adc conversion of baro data
        void start_temp_conversion();   // initiate adc conversion of temperature data
        void get_baro_data();           // read baro data into i2c buffer
        void get_temp_data();           // read temperature data into i2c buffer
        void get_cal_data();            // read baro calibration data and calculate correction factors
 
        unsigned int raw_baro;              // raw 24 bit sensor value of baro output
        unsigned int raw_temp;              // raw 24 bit baro temperature
        unsigned int C1, C2, C3, C4, C5, C6;    // 16 bit baro PROM calibration constants
        long long dT;                           // temperature calibration factor (see MS511 datasheet)
        long long off;                          // calculated offset factor
        long long sens;                         // calculated sensitivity factor
        long long temp;                         // integer value of compensated temperature
        long long p;                            // integer value of compensated pressure
        float temperature;                      // deg C, floating point compensated temperature
        float pressure;                         // mbar, floating point compensated baro
        float altitude;                         // feet, compensated altitude
        unsigned char baro_initialized;          // flag to indicate baro has been initialized and it can be used

    };

class rate_limit // signal rate limiter
    {
    public:
        rate_limit();
        void compute(float in, float limit);    // input value limited to limit value delta each time executed
        float out;
    };

// declare classes globally
MPULib platform_MPU9250;       // primary MPU data struture
MPULib tether_MPU9250;       // tether MPU data structure
BaroAltLib baro_alt;    // baro altimeter data structure

Gyro platform_x_gyro;    // gyro data structures
Gyro platform_y_gyro;
Gyro platform_z_gyro;
Accel platform_x_accel;  // accelerometer data structures
Accel platform_y_accel;
Accel platform_z_accel;

Gyro tether_x_gyro;    // gyro data structures
Gyro tether_y_gyro;
Gyro tether_z_gyro;
Accel tether_x_accel;  // accelerometer data structures
Accel tether_y_accel;
Accel tether_z_accel;

PID pitch_rate_PID;     // inner rate loop PID controllers
PID roll_rate_PID;
PID yaw_rate_PID;

PID pitch_attitude_PID; // outer position loop PID controllers
PID roll_attitude_PID;
PID yaw_attitude_PID;

PID tether_pitch_rate_PID;     // inner rate loop PID controllers
PID tether_roll_rate_PID;

PID tether_pitch_attitude_PID; // outer position loop PID controllers
PID tether_roll_attitude_PID;

PID x_accel_PID;
PID y_accel_PID;
PID z_accel_PID;

radio_rx pitch_rx;
radio_rx roll_rx;
radio_rx yaw_rx;
radio_rx throttle_rx;
radio_rx ch5_rx;

low_pass_filter platform_x_accel_angle_lpf;
low_pass_filter platform_y_accel_angle_lpf;

low_pass_filter tether_x_accel_angle_lpf;
low_pass_filter tether_z_accel_angle_lpf;

low_pass_filter x_damper_lpf;
low_pass_filter y_damper_lpf;

low_pass_filter tether_pitch_angle_lpf;
low_pass_filter tether_roll_angle_lpf;

low_pass_filter tether_pitch_rate_lpf;
low_pass_filter tether_roll_rate_lpf;

low_pass_filter tether_pitch_rate_lpf2;
low_pass_filter tether_roll_rate_lpf2;

low_pass_filter tether_pitch_rate_gain_lpf;
low_pass_filter tether_roll_rate_gain_lpf;

low_pass_filter baro_alt_lpf;

low_pass_filter sin_yaw_lpf;
low_pass_filter cos_yaw_lpf;

low_pass_filter yaw_cmd_lpf;
low_pass_filter yaw_rate_lpf;

low_pass_filter ch5_lpf;

rate_limit tether_pitch_angle_rlm;
rate_limit tether_roll_angle_rlm;

rate_limit tether_pitch_rate_rlm;
rate_limit tether_roll_rate_rlm;

rate_limit tether_pitch_PID_rlm;
rate_limit tether_roll_PID_rlm;

rate_limit yaw_cmd_rlm;
rate_limit tilt_cmd_rlm;

////////////////////////////////////////////
// Prototype Function Declarations
////////////////////////////////////////////

    void configure_io(void);            // configure PIC IO

    void configure_interrupts(void);    // configure PIC interrupts

    void delay(float x);            // delays for x seconds

    void get_high_water(void);      // calculate max interrupt duration to determine thruput margin

    void unit_status(void);         // blinks units status LED
   
    void update_MPU_sensors(void);  // read and process MPU_1 i2c data gyro, accel and temperature data

    void update_baro_data(void);
    
    void compute_platform_attitude(void);    // compute attitude (sensor fusion) using latest MPU sensor data

    void compute_tether_attitude(void);

    void compute_platform_acceleration(void);

    void drive_servos(void);

    void initialize_rx(void);

    void initialize_baro(void); // must run before platform_initialized = true because it uses absolute_time

    void initialize_PIDs(void);

    void reset_PIDS(void);

    void calibrate_ESCs(void);

    void get_rx_cmds(void);

    void primary_control_laws(void);

    void tether_hold_control_laws(void);

    void xyz_damper_control_laws(void);

    void drive_motors(void);

    float scale(float in_value, float in_min, float in_max, float out_min, float out_max);

    float limit(float in_value, float lower_limit, float upper_limit);

    float washout (float in_value, float washout_rate);

    void measure_platform_vib(void);

    void measure_step_response(void);

    int readI2c(unsigned char slaveAddress, unsigned char slaveRegister, unsigned int numBytes, unsigned char i2cData[]);

    void writeI2c(unsigned char slaveAddress, unsigned char slaveRegister, unsigned char writeData);

    void writeCommandI2c(unsigned char slaveAddress, unsigned char i2cCommand);

    void initialize_MPUs(void);

    void calculate_platform_MPU_offsets(void);
    void calculate_tether_MPU_offsets(void);

    void get_sensor_offsets(void);

    void datalog_flash(void);
    void datalog_uart(void);

    void create_flash_record();

    unsigned int write_flash(unsigned int, float);

    void compute_altitude(void);

    void reset_tether_cl(void);
    void reset_damper_cl(void);
    void reset_platform_cl(void);
    void sync_yaw(void);

    void set_modes(void);

    void wait_pan(float interval);
    void pan_platform(float angle);

    void erase_flash(void);

    void half_press(void);
    void full_press(void);
    void release(void);

    void check_sensors(void);
    
    void take_picture_wait_pan_trigger(void);
    void take_picture(void);

    void disable_datalogging(void);

    void get_gyro_pu_offsets(void);

    void delay_usec(float time_delay);
    void resetI2c(void);

    void thrust_test(void);

    void take_s95_picture(void);
    void take_s95_picture_wait_pan_trigger(void);

    void ricoh_gr_routine(void);
    void canon_s95_routine(void);

    void fastPutsUART2(const char*);

    void build_uart_string(void);

    void manual_sensor_test(void);
