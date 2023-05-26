/////////////////
// TetherQuad //
/////////////////
//
// c 2015, 2016 IowaKAPer / Mike LeDuc
//
// Rev 9/17/17
// V9.8
//
//////////////////////////////////////////////////////////////////////////////
// Version History
//
// V1.0 4/3/15:  Initial configuration
// V1.1 4/13/15: Added i2c functionality
// V1.2 4/20/15: Added sensor fusion and initial servo pulse width control
// V1.3 4/30/15: Added initial PID and single channel radio rx routines
// V1.4 5/1/15:  Separated files into separate categories, changed project name from PIC32MX250_Template to TetherQuad
// V1.5 5/16/15: Working pitch axis stabilization with XTR 20A ESCs
// V1.6 5/25/15: Switched from PIC32MX128F128B to PIC32MX270F256B processor
// V1.7 5/27/15: Initial pitch/roll PID, first quad frame integration with Tiger Motor Air Gear 350 components
// V1.8 6/16/15: Pitch and roll control laws with 4 channel radio rx function
// V1.9 6/21/15: First tethered flight, 500Hz iteration, 36in tether, 43.65oz
// V2.0 6/23/15: First outdoor flight
// V2.1 6/27/15: Adds prelim altitude hold mode based on z axis accelerometer
// V2.2 7/3/15:  Added filter routines, ADC routine and yaw potentiometer input
// V2.3 7/16/15: Added washout and platform vibration measurement functions
// V2.4 7/16/15: Added step response function
// V2.5 7/17/15: Tuned Pitch / Roll PID Loops
// V2.6 7/28/15: Added initial line tension sensor pot, added yaw_rx_rate_cmd integrator
// V2.7 8/1/15:  Added Pitch/Roll joystick line angle sensor
// V2.8 8/8/15:  Added Baro Sensor
// V2.9 8/22/15: Added initial Tether MPU control laws
// V3.0 8/26/15: Increased compilier optimization to "3" (corrects issue passing class values to other classes)
//               Added accelerometer x/y position hold control laws
// V3.1 8/29/15  Improved tether tension pot with tether attitude MPU
// V3.2 9/19/15  Improved tether tension control laws.
// V3.3 9/20/15  Good x/y hold.  Improved tether attitude hold
// V3.4 9/21/15  Good tether rate hold
// V3.5 9/26/15  Improved tether hold
// V3.6 9/28/15  Separated accel and tether C/Ls
// V3.7 9/30/15  First working tether (under damped tether angle but stable)
// V3.8 10/3/15  Improved sensor fusion complementary filter (lpf accel and reduce COMP_FLTR_COEFF)
// V3.9 10/4/15  Improved tether and accel C/L constants
// V4.0 10/8/15  First very stable tether with tether rx angle cmd and linear acceleration integrator washout
// V4.1 10/10/15 Added tether angle rate limit to reduce high frequency tether noise into tether control law (150ft Tethered Flight)
// V4.2 10/11/15 Adjusted control laws, reduced platform attitude PID I limit
// V4.3 10/14/15 Adjusted control laws
// V4.4 10/17/15 More Improvements
// V4.5 10/11/15 Linear rate damper drift compensation
// V4.6 10/27/15 Variable tether rate damping
// V4.7 11/1/15 Refined tether rate damping (gain scale based on tether rate)
// V4.8 11/5/15 Added temperature compensated MPU sensor offset calculations (y=mx+b)
// V4.9 11/29/15 Changed accel FS to 2g.  Improved temp compensation and added new xyz damper C/L. Updated lpf algorithm
//               Added initial flash log data routine
// V5.0 12/13/15 Refined flash log routine
// V5.1 12/30/15 New Radio, PID adjustments, flash memory full protection, unit status frequency based on flash full
// V5.2 1/3/16   Interleave I2C reading of tether MPU sensors and baro. Reduced Timer 3 clk to 2.5 MHz to account for 15 msec radio rate.
//               Added damper output limit, filtering/rlm to damper tether rate input.
// V5.3 1/4/16   Misc PID adjustments
// V5.4 1/10/16  Added tx trim inputs to tether mode, add yaw compensation to tether angles / control laws
// V5.5 1/30/16  Outdoor flight test
// V5.6 1/31/16  Outdoor flight test (50, 75 ft)
// V5.7 2/5/16   Iteration rate changed to 400 hz to increase thruput for additional compass I2C
//               Baro altitude calculation correction
//               Baro Alt used in damper C/L, VR Knob used as damper altitude limit
//               Damper tether velocity filter changed to double pole 0.05 second, velocity comp filter coeff = 0.999
// V5.8 2/12/16  Yaw Rx compensation, ITER_RATE changed to 0.0021 sec to reduce platform jitter
// V5.9 2/16/16  First Pan Control from main 
// V6.0 3/9/16   Updated pan mode, added tether velocity limit vs. altitude, removed tension control laws, updated rx control modes.
//               VEL_LIM reduced to 2 m/s, damper and yaw PID limits reduced, moved datalogging to separate .cpp file
// V6.1 3/12/16  Adds S95 shutter control
// V6.2 3/19/16  Reduced yaw_rate_PID output negative limit, added rate limit to yaw_cmd, added minimum motor speed (min_mtr_lim) during flight
//               Increased throttle cmd to 65% (S95 and 2200mah battery)
// V6.3 3/27/16  Updated pitch and roll platform PID gains to reduce high freq osc.
// V6.4 4/9/16   Added throttle lockout function.  Added pitch/roll Tx trim function.  Updated high watermark
// V6.5 4/14/16  Added throttle gain schedule, added velocity integrator hold while panning
// V6.6 4/15/16  Changed yaw_cmd rate limit from 30 to 80.  Fixed issue with throttle gain schedule
// V6.7 4/21/16  Reduced DAMPER_I GAIN and DAMPER_P_GAIN during pan mode to reduce linear drift during panning.
//               Added yaw joystick control of shutter in pan mode.
// V6.8 4/22/16  Added throttle hold during pan.  Updated shutter control during pan
// V6.9 4/23/16  Replaced tether MPU and updated offset calibration factors
// V7.0 4/27/16  Adds Ricoh GR shutter control
// V7.1 4/28/16  Dual camera selection with tailored flight throttle value (based on camera type)
// V7.2 4/29/16  Swap Throttle and VR_knob functions (Throttle now on VR_knob, Throttle used for altitude limit cintrol)
// V7.3 4/30/16  Changed VR_knob to ctrl_stick.  Increased yaw_sync oneshot delay to 1 sec.  Moved get_camera_type() to main
//               Added power-up sensor monitoring.  Added power-up erase flash routine. Added high watermark reset function to yaw_sync()
// V7.4 5/8/16   Adjusted shutter routiine to provide second quick shot for tilt photo
// V7.5 5/12/16  Switched to internal fast rc oscillator.  added drive_servos() routine for tilt servo. Added offset to x gyro.rate to reduce
//               y_damper offset (see PID Tuning Notes.xls)
// V7.6 5/15/16  Added tilt servo and take_picture routines.  Removed offset from x gyro.rate, previous 1.8 deg/sec are no longer being seen on this sensor (tbd why).
//               Removed dual camera selection (GR is only camera supported).
// V7.7 5/29/16  Adjusted roll motor mechanical mounting to improve vertical alignment. Yaw C/L offset significantly reduced, changed yaw_rate_PID limit to be symetric as a result.
//               Added power-up option to disable datalog (yaw joystick outboard at power-up).
// V7.8 5/31/16  Added platform gyro power up offset calibration
// V7.9 6/1/16   Add logging of temperature and gyro offsets when throttle is low (capture constants at beginning and ending of flight
// V8.0 6/25/16  New Carbon Fiber Frame
// V8.1 7/1/16   Small PID adjustments, platform x/y accel offset adjustment, updates to measure_step_response routine
//               Reduced flight_throttle_value for lighter frame
// V8.2 7/8/16   Adjust pitch/roll PID values to reduce carbon platform shake.  Improved throttle power up fail safe.  Adjusted platform_x/y_accel_offsets code
//               Updated tether accel offsets for new tether gimbal
// V8.3 7/10/16  Added watchdog timer,  added delay shadow write of slave (tether) data
// V8.4 7/24/16  Added extensive error handling to readI2C() routine. Added additional vibration damping (moon gel) to platform CPU.  New tether mounting bracket/gimbal.
//               Set RA3 and RB11 to port outputs (to eliminate floating spare inputs.  Set port pins on I2C SCL/SDA as inputs to ensure pins were free for I2C
//               Add ground jumper to floating pot analog input on DP32 board to eliminate floating input. Added ground strap between DP32 board and carbon frame.  Shortened platform MCU ground wire to PIC.
//               Added compilier optimization switch __attribute__((optimize("O0"))) to some functions to turn off optimization for sequential control code.  Increased ITER_PERIOD to 0.0023 for thruput margin.
// V8.5 7/26/16  Added second MPU sensor initialization routine call at startup to correct improper slave MPU initialization
//               Added power up test of tether angle sensors
// V8.6 8/19/16  Added thrust_test() routine for coaxial motor tests.  decreased datalog rate to 2Hz due to longer flights from 4000mah battery
// V8.7 9/13/16  Added power up selection for S95 Camera and S95 camera routines (actual S95 routines were created and flown in TetherQuad.X 9-3-16.zip but are being officially released in this version).
//               Yaw stick full left at power up switches to S95 (default is GR).
// V8.8 1/6/17   Add UART datalogging. Program memory constrained to 98k otherwise FLASH datalooging will corrupt pgm memory.
//               Iteration period increased to 2.4 msec until UART DMA can be added.
//               Added gain adjustment factor to accel and gyro classes
//               Added log_offsets_uart() routine for sensor temperature calibraton support
// V8.9 2/10/17  Additional UART routine optimizations
// V9.0 2/18/17  Re-wrote uart int to ascii routine to eliminate need for sprintf() routines to save program memory.
//               Reverted to 2.3msec ITER_PERIOD, reverted to 192KB flash datalogging (pgm memory limited to <65KB)
//               UART logging and FLASH logging can now run concurrently.
// V9.1 3/30/17  Reduced minimum ground baro altitude to 50M (baro pressure range is greater than originally expected)
//               Updated tether gyro power up offset calibration (tether must not be moving at power up for accurate calibration)
//               Added tether C/L output limit
//               Updated Platform and Tether Accel temp calib constants
//               Disable platform z accel processing until alt hold C/L is developed.
// V9.2 3/30/17  Added yaw platform angle compensation
// V9.3 6/10/17  Added tilt servo offset constant.  provisional code for vertical shot during first pan sequence.
// V9.4 6/11/17  Adjusted GR camera rigging values
// V9.5 6/17/17  Adjust GR camera rigging values (for padded case)
// V9.6 8/12/17  Reduced XYZ Damper Limit from 15 to 10 degrees to limit authority if sensor gyro drift is excessive.
// V9.7 8/13/17  Reduced Pitch and Roll PID attitude KP from 4.0 to 3.0 to reduce platform oscillation due to lighter P400 frame.
//               Added tether gyro datalogging at beginning and end of flight (log parameters 20-24 when ctrl stick and throttle down).
// V9.8 9/17/17  Added manual platform and tether angle sensor test at startup
//////////////////////////////////////////////////////////////////////////////

#include <xc.h>             // Defines special funciton registers, CP0 regs
#include <plib.h>           // Include to use PIC32 peripheral libraries
#include <stdint.h>         // For UINT32 definition
#include <stdbool.h>        // For true/false definition
#include <math.h>
#include <exception>        // Includes C++ try/catch functions

#include "header.h"
#include "platform config.h"
#include "declarations.cpp"
#include "interrupts.cpp"
#include "i2cLib.cpp"
#include "MPULib class.cpp"
#include "BaroAltLib class.cpp"
#include "gyro class.cpp"
#include "accel class.cpp"
#include "PID class.cpp"
#include "filter classes.cpp"
#include "radio_rx class.cpp"
#include "platform functions.cpp"
#include "rate limit class.cpp"
#include "datalog.cpp"

//using namespace std;        // use the standard c++ namespace

//////////////////////////////////
// Application Functions
//////////////////////////////////

void compute_platform_attitude(void) // perform sensor fusion to compute platform attitude
{
    sin_yaw_delta = sin(platform_z_gyro.delta * DEG_TO_RAD);
    cos_yaw_delta = cos(platform_z_gyro.delta * DEG_TO_RAD);

    platform_y_accel_angle_lpf.compute(platform_y_accel.angle, 0.1);  // filter noise and linear movement spikes and from accelerometer angle
    platform_x_accel_angle_lpf.compute(platform_x_accel.angle, 0.1);  

    if(absolute_time < 3.0) comp_fltr_coeff = COMP_FILTR_COEFF_INIT; // use more accel in sensor fusion during power up to settle initial platform angle faster
    else comp_fltr_coeff = COMP_FLTR_COEFF_STEADY_STATE;             // use more gyro in sensor fusion after power up to reduce influence of accel noise and linear accelerations
                                                                     // *** platform must not be moving during power until sensor fusion settles

    //pitch_angle = pitch_angle + platform_x_gyro.delta;  // integrate gyro rate to get attitude angle
    pitch_angle = ((pitch_angle + platform_x_gyro.delta) * cos_yaw_delta) - ((roll_angle - platform_y_gyro.delta) * sin_yaw_delta);  // integrate gyro rate to get attitude angle
    pitch_angle = (comp_fltr_coeff * pitch_angle)+((1-comp_fltr_coeff) * platform_y_accel_angle_lpf.out);     // use complementary filter to combine gyro and accel data
          
    //roll_angle = roll_angle - platform_y_gyro.delta;   // integrate gyro rate to get attitude angle, sign is negative due to sensor coordinate system orientation
    roll_angle = ((roll_angle - platform_y_gyro.delta) * cos_yaw_delta) + ((pitch_angle + platform_x_gyro.delta) * sin_yaw_delta);   // integrate gyro rate to get attitude angle, sign is negative due to sensor coordinate system orientation
    roll_angle = (comp_fltr_coeff * roll_angle)+((1-comp_fltr_coeff) * platform_x_accel_angle_lpf.out);      // use complementary filter to combine gyro and accel data

    yaw_angle = yaw_angle + platform_z_gyro.delta;  // integrate gyro rate to get attitude angle

    sin_yaw = sin(yaw_angle * DEG_TO_RAD);
    cos_yaw = cos(yaw_angle * DEG_TO_RAD);
    sin_yaw_lpf.compute(sin_yaw, 0.1);
    cos_yaw_lpf.compute(cos_yaw, 0.1);
    yaw_rate_lpf.compute(platform_z_gyro.rate, 0.01);
 
    // add magnetometer complementary filter to yaw angle for sensor fusion
}

void compute_tether_attitude(void) // perform sensor fusion to compute platform attitude
{
    tether_x_accel_angle_lpf.compute((tether_x_accel.angle), 0.1);  // filter noise and linear movement spikes and from accelerometer angle
    tether_z_accel_angle_lpf.compute((tether_z_accel.angle), 0.1);

    if(absolute_time < 3.0) comp_fltr_coeff = COMP_FILTR_COEFF_INIT; // use more accel in sensor fusion during power up to settle initial tether angle faster
    else comp_fltr_coeff = COMP_FLTR_COEFF_STEADY_STATE;             // use more gyro in sensor fusion after power up to reduce influence of accel noise and linear accelerations

    tether_pitch_angle = ((tether_pitch_angle - tether_x_gyro.delta) * cos_yaw_delta) - ((tether_roll_angle - tether_z_gyro.delta) * sin_yaw_delta);  // gyro integrator
    tether_pitch_angle = (comp_fltr_coeff * tether_pitch_angle)+((1-comp_fltr_coeff) * tether_z_accel_angle_lpf.out);    // use complementary filter to combine gyro and accel data

    tether_roll_angle = ((tether_roll_angle - tether_z_gyro.delta) * cos_yaw_delta) + ((tether_pitch_angle - tether_x_gyro.delta) * sin_yaw_delta);   // gyro integrator
    tether_roll_angle = (comp_fltr_coeff * tether_roll_angle)+((1-comp_fltr_coeff) * -tether_x_accel_angle_lpf.out);      // use complementary filter to combine gyro and accel data
}

void compute_platform_acceleration(void)
{
    sin_pitch = sin(pitch_angle * DEG_TO_RAD);
    sin_roll = sin(roll_angle * DEG_TO_RAD);

    x_linear_accel_last = x_linear_accel;  //save last acceleration
    y_linear_accel_last = y_linear_accel;

    x_linear_accel = (platform_x_accel.accel - sin_roll) * 9.8;  // translate to earth reference (subtract gravity influence on axis) and scale from g to meters/sec
    y_linear_accel = (platform_y_accel.accel - sin_pitch) * 9.8; // translate to earth reference (subtract gravity influence on axis)

    x_linear_accel = limit(x_linear_accel, -LIN_ACCEL_LIM, LIN_ACCEL_LIM);  // limit acceleration
    y_linear_accel = limit(y_linear_accel, -LIN_ACCEL_LIM, LIN_ACCEL_LIM);

    //z_factor = sqrt(1-sin_pitch*sin_pitch - sin_roll*sin_roll);
    //z_linear_accel = platform_z_accel.accel * z_factor;  // translate to earth reference (subtract x/y influence on z axis)
    //z_linear_accel_lpf.compute(z_linear_accel , 0.01);
}

void get_rx_cmds(void)
{
    //*********************************************************************************************
    // Futaba T6K Radio Setup (Model "TQ VR-TH")
    //  Ch 1: Roll (1.0 - 2.0msec) Stick Left = 1.0msec
    //  Ch 2: Pitch (1.0 - 2.0msec) Stick Up = 1.0msec
    //  Ch 3: Not connected
    //  Ch 4: Yaw (1.0 - 2.0msec) Stick Left = 1.0msec
    //  Ch 5: VR knob (used as TQ throttle)
    //  Ch 6: Tx Throttle Stick mixed with Switch B (used for "Mode Control Stick and Mode Switch)
    //
    //  Switch A: (not assigned) (future datalink, "morse code")
    //  VR Knob:  Throttle ctrl (1.0 - 2.0msec). full ccw = 0 throttle. Assigned to Tx channel 5 (Aux)
    //  Swtich B: Mode switch, connected to Channel 6 (MOD) of radio.  Mixed with channel 3 (mode control stick) to control various modes
    //            (See mode control table below)
    //  Switch C: Telemetry voice enabled
    //  Switch D: Reset Throttle Timer
    //
    //  Rx (R3008SB) Mapping to PIC Read Capture
    //     Ch 1 - mIC1ReadCapture (roll)
    //     Ch 2 - mIC2ReadCapture (pitch)
    //     Ch 3 - Not Connected
    //     Ch 4 - mIC4ReadCapture (yaw)
    //     Ch 5 - mIC3ReadCapture (throttle
    //     Ch 6 - mIC5ReadCapture (mode ctrl stick and mode switch)
    //     Ch 7 - Not Connected
    //     Ch 8 - Battery
    // *******************************************************************************************

    //throttle rx cmd
    throttle_rx.get_pw(mIC3CaptureReady(), mIC3ReadCapture());
    //if(throttle_rx.channel_active) throttle_rx_cmd = throttle_rx.pulse_width; // scale throttle between 0 and 100%

    if(throttle_rx.channel_active) throttle_rx_cmd = scale(throttle_rx.pulse_width, 0.001, 0.002, 0, 100); // scale throttle between 0 and 100%
    throttle_rx_cmd = limit(throttle_rx_cmd, 0, 100);

    if(throttle_rx_cmd < MIN_PWR_UP_THROTTLE_VALUE) ++throttle_low_count;  // count consecutive frames that throttle is low  (consecutive counts used to filter joystick noise from being misintrepreted as low stick during flight)
    else throttle_low_count = 0;                                           // reset counter if throttle up

    if((throttle_low_count > 100) & (ctrl_stick < 10)) throttle_enable = false;  // disable motors if throttle low and ctrl_stick low (safety feature to not allow power up into non-zero throttle)

    if((throttle_rx.channel_active) & (throttle_low_count > 100) & (ctrl_stick > 10) & (absolute_time > 3)) throttle_enable = true;   // ** safety feature ** wait for throttle to be placed in minimum stick position
                                                                                                                                // throttle lockout function: ctrl_stick must be > 10 after power up to enable throttle
    if((!throttle_rx.channel_active)& absolute_time < 30) throttle_enable = false;    // channel inactivity indicates radio is turned off, disable throttle to make sure                                                                                                                       // after tx power up
                                                                                         // it it doesn't drive motors until the throttle stick is down
                                                                                         // -- don't disable motors if radio contact is lost after 30 seconds of operation
                                                                                         // -- this ensures the unit can still function on tether if radio contact is lost at higher altitudes
    // pitch and roll rx cmds
    pitch_rx.get_pw(mIC2CaptureReady(), mIC2ReadCapture());                              // get pitch and roll rx pulse width
    roll_rx.get_pw(mIC1CaptureReady(), mIC1ReadCapture());

    // adjust radio tx sub trims to remove radio offsets (null stick should result in 0 deg pitch/roll_rx_cmd_raw
    pitch_rx_cmd_raw = scale(pitch_rx.pulse_width, 0.001, 0.002, -27, 27);               // map rx pulse widths to angle cmds
    roll_rx_cmd_raw = scale(roll_rx.pulse_width, 0.001, 0.002, -27, 27);

    //set pitch/roll trim values
    if((throttle_rx_cmd < THROTTLE_START_VALUE) || ((mode_sw_posn == DOWN) & (ctrl_stick < 10) & (platform_altitude < 15)))
    {
        if (abs(pitch_rx_cmd_raw) < 3.5) pitch_trim = limit(pitch_rx_cmd_raw, -3.5, 3.5);
        else pitch_trim = 0.0;

        if (abs(roll_rx_cmd_raw) < 3.5) roll_trim = limit(roll_rx_cmd_raw, -3.5, 3.5);
        else roll_trim = 0.0;
    }

    if(abs(yaw_cmd) > 0.1) // adjust pitch and roll rx commands based on yaw if yaw_cmd > 0
    {
        pitch_rx_cmd = (pitch_rx_cmd_raw - pitch_trim) * cos_yaw_lpf.out - (roll_rx_cmd_raw - roll_trim) * sin_yaw_lpf.out + pitch_trim;
        roll_rx_cmd = (roll_rx_cmd_raw - roll_trim) * cos_yaw_lpf.out + (pitch_rx_cmd_raw - pitch_trim) * sin_yaw_lpf.out + roll_trim;
    }
    else
    {
        pitch_rx_cmd = pitch_rx_cmd_raw;
        roll_rx_cmd = roll_rx_cmd_raw;
    }

    pitch_rx_cmd = limit(pitch_rx_cmd, -27, 27);                                            // limit pitch and roll rx authority
    roll_rx_cmd = limit(roll_rx_cmd, -27, 27);

    // yaw rx cmd
    yaw_rx.get_pw(mIC4CaptureReady(), mIC4ReadCapture());

    yaw_rx_rate_cmd = scale(yaw_rx.pulse_width, 0.001, 0.002, 15, -15); // map rx signal to rate cmd

    if(abs(yaw_rx_rate_cmd) < YAW_RX_DEAD_ZONE) yaw_rx_rate_cmd = 0.0; // yaw joystick dead zone
    yaw_rx_rate_cmd = limit(yaw_rx_rate_cmd, -15, 15);

    if(pan_mode == false) yaw_rx_angle_cmd = yaw_rx_angle_cmd + (yaw_rx_rate_cmd * ITER_PERIOD * 4);  // integrate yaw rx stick to create yaw angle cmd
                                                                                                      // do not integrate when in pan mode since yaw joystick is used to trigger shutter
    // add power up stick calibration

    ch5_rx.get_pw(mIC5CaptureReady(), mIC5ReadCapture());   //get rx channel 5 input (aux switch b)
    
    ch5_rx_cmd = ch5_rx.pulse_width;
    ch5_rx_cmd = limit(ch5_rx_cmd, 0.000920, 0.002075);

    if(ch5_rx_cmd <  0.0012) mode_sw_posn = DOWN;    // set switch b position based on rx cmds (note rx fail safe is set to "switch up" if tx signal lost)
    if(ch5_rx_cmd >= 0.0012) mode_sw_posn = MIDDLE;
    if(ch5_rx_cmd >  0.0019) mode_sw_posn = UP;
    if(ch5_rx_cmd >  0.00215) mode_sw_posn = DOWN;   // set to down if excessive pulse width

    ch5_lpf.compute(ch5_rx_cmd, 0.3);
}

void set_modes(void)
{
    // set flight modes based on mode switch position;
    //***************************************************************************************************************************
    //      Inputs		            Outputs
    //ctrl_stick SW-B	        Damper	        Tether	Pan	Pan Sync   Velocity Integrator  Altitude      pitch_trim/roll_trim
    //--------------------------------------------------------------------------------------------------------------------------
    // < 10%	 Down      ctrl_stick Alt Lim	Disable	Disable  One Shot    Run Integrator	<15 Meters    trim = pitch/roll joystick values
    //   '        '                '              '       '         '             '             >15 Meters    saved value at 0 throttle or joystick value @<15 meters altitude
    // >10%	 Down      ctrl_stick Alt Lim   Enable  Disable  Disable     Run Integrator     Don't Care    saved value above
    //   '       Center    ctrl_stick Alt Lim   Enable	Enable   Disable     Hold while panning Don't Care    saved value above
    //   '       Up	   ctrl_stick Alt Lim   Disable	Disable  Disable     Run Integrator     Don't Care    saved value above
    //***************************************************************************************************************************
    
    if(mode_sw_posn == UP)
    {
        ctrl_stick = scale(ch5_rx_cmd, 0.001908, 0.002072, 0.0, 100.0); // Tx ctrl_stick position from 0% (CCW) to 100% (CW)
        ctrl_stick = limit(ctrl_stick, 0.0, 100.0);  // limit VR knob
 
        tether_mode = false;
        pan_mode = false;
        vert_pic_arm = true;    // arm flag to take vertical pan pictures during first pan sequence

        yaw_sync_count = 0;
        yaw_sync_arm = true;

        DAMPER_P_GAIN = 5;  //restore damper gains when not panning
        DAMPER_I_GAIN = 15;
    }

    if(mode_sw_posn == MIDDLE)
    {
        ctrl_stick = scale(ch5_rx_cmd, 0.001427, 0.001594, 0.0, 100.0); // Tx VR knob position from 0% (CCW) to 100% (CW)
        ctrl_stick = limit(ctrl_stick, 0.0, 100.0);  // limit VR knob

        tether_mode = true;
        pan_mode = true;

        yaw_sync_count = 0;     // reset yaw sync frame counter
        yaw_sync_arm = true;    // arm yaw sync oneshot

        DAMPER_P_GAIN = 4;  // reduce damper gains slightly to reduce linear acceleration errors causing platform to move when panning
        DAMPER_I_GAIN = 10;
    }

    if(mode_sw_posn == DOWN)
    {
        ctrl_stick = scale(ch5_rx_cmd, 0.0009319, 0.0011, 0.0, 100.0); // Tx VR knob position from 0% (CCW) to 100% (CW)
        ctrl_stick = limit(ctrl_stick, 0.0, 100.0);  // limit VR knob

        if(ctrl_stick >= 10.0)
        {
            tether_mode = true;
            pan_mode = false;
            vert_pic_arm = true;    // arm flag to take vertical pan pictures during first pan sequence

            yaw_sync_count = 0;     // reset yaw sync frame counter
            yaw_sync_arm = true;    // arm yaw sync oneshot

            DAMPER_P_GAIN = 5;  //restore damper gains when not panning
            DAMPER_I_GAIN = 15;
        }

        if(ctrl_stick < 10) ++yaw_sync_count;    // increment yaw sync frame counter if VR knob is below sync threshold and switch B is down
        if(yaw_sync_count == 500 & yaw_sync_arm) // perform yaw sync oneshot if VR knob has been < threshold for required number of consecutive frames
        {
            tether_mode = false;
            pan_mode = false;

            reset_tether_cl();          // oneshot reset tether and damper control laws
            reset_damper_cl();
            sync_yaw();                // sync yaw angle with rx cmd

            yaw_sync_count = 0;         // reset yaw sync frame counter
            yaw_sync_arm = false;       // disarm oneshot until next arm cycle
            
            high_water = 0.0;           // reset high water mark to increase visibility of frame variations
        }
    }
}

void reset_tether_cl(void)
{ 
    tether_pitch_angle_lpf.initialize(tether_pitch_angle);
    tether_pitch_rate_lpf.initialize(-tether_x_gyro.rate);
    tether_pitch_rate_lpf2.initialize(-tether_x_gyro.rate);
    tether_pitch_attitude_PID.reset();
    tether_pitch_rate_gain_lpf.initialize(tether_pitch_rate_gain);
    tether_pitch_rate_PID.reset();;
    tether_pitch_PID_rlm.out = tether_pitch_rate_PID.out;
    tether_pitch_cmd = 0.0;
    tether_pitch_angle_rlm.out = tether_pitch_angle;

    tether_roll_angle_lpf.initialize(tether_roll_angle);
    tether_roll_rate_lpf.initialize(-tether_z_gyro.rate);
    tether_roll_rate_lpf2.initialize(-tether_z_gyro.rate);
    tether_roll_attitude_PID.reset();
    tether_roll_rate_gain_lpf.initialize(tether_roll_rate_gain);
    tether_roll_rate_PID.reset();;
    tether_roll_PID_rlm.out = tether_roll_rate_PID.out;
    tether_roll_cmd = 0.0;
    tether_roll_angle_rlm.out = tether_roll_angle;
}

void reset_damper_cl(void)
{
    y_velocity = 0.0;
    y_damper_lpf.initialize(0.0);

    x_velocity = 0.0;
    x_damper_lpf.initialize(0.0);
}

void reset_platform_cl(void)
{
    reset_PIDS();

    reset_tether_cl();
    reset_damper_cl();

    sync_yaw();
}

void sync_yaw(void)     // synchronize reset yaw angle and synchronize with tx pitch and roll joystick commands
{
    yaw_angle = 0.0;            // reset yaw integrators and pan reference control points
    yaw_rx_angle_cmd = 0.0;
    sin_yaw_delta = 0.0;
    cos_yaw_delta = 1.0;
    sin_yaw = 0.0;
    cos_yaw = 1.0;
    sin_yaw_lpf.initialize(0.0);
    cos_yaw_lpf.initialize(1.0);
    yaw_cmd_lpf.initialize(0.0);
    yaw_cmd_rlm.out = yaw_cmd_lpf.out;
    pan_cmd = 0.0;
    yaw_rate_lpf.initialize(0.0);
}

void compute_altitude(void)
{
    // add baro range error checking
    baro_alt_lpf.compute(baro_alt.altitude, 0.3);  // filter baro altitude to remove noise

    if(!ground_altitude_calculated)
    {
        if((throttle_rx_cmd < THROTTLE_START_VALUE) & (absolute_time > 2)) ground_altitude = baro_alt_lpf.out;  // continue to sample baro for ground alt until throttle up
        else ground_altitude_calculated = true;  // use baro altitude at time of throttle up as ground altitude
    }
                                                                                      // (1.5 meters min for compatibility with lab tether rigging)
    if(ground_altitude_calculated)
    {
        platform_altitude = limit((baro_alt_lpf.out - ground_altitude), 1.5, 100.0);     // limit min and maximum altitude in meters
    }
    tx_altitude_lim = scale(ctrl_stick, 0.0, 100, 1.5, 100.0); // set altitude limit based on Tx ctrl stick altitude limit from 1.5 meters (ccw) to 100 meters (cw)
}
 void tether_hold_control_laws(void)
{
    // pitch tether control law
    tether_pitch_cmd = 0;

    //enable the following code to provide integrator controlled tether position
        //rx_integrator_delta = pitch_rx_cmd;
        //if(abs(rx_integrator_delta) < PITCH_RX_DEAD_ZONE) rx_integrator_delta = 0.0; // joystick dead zone, enabled if in tether mode to eliminate tether cmd integrator drift
        //tether_pitch_cmd = tether_pitch_cmd + (rx_integrator_delta * ITER_PERIOD * 0.2);  // integrate rx stick to create tether angle cmd
        //tether_pitch_cmd = (tether_pitch_cmd * cos_yaw_delta) - (tether_roll_cmd * sin_yaw_delta);
        //tether_pitch_cmd = limit(tether_pitch_cmd, -25, 25);

    tether_pitch_angle_rlm.compute(tether_pitch_angle, tether_angle_rate_limit*ITER_PERIOD);   // rate limit and filter to reduce noise from moving tether
    tether_pitch_angle_lpf.compute(tether_pitch_angle_rlm.out, TETHER_ANGLE_LPF_TAU);      //smooth tether loop response to fast tether movement

    tether_pitch_rate_lpf.compute(limit(-tether_x_gyro.rate, -tether_angle_rate_limit, tether_angle_rate_limit), TETHER_RATE_LPF_TAU);

    tether_pitch_attitude_PID.compute(tether_pitch_cmd, tether_pitch_angle_lpf.out);  // position outer loop (deg/sec output)

    if(abs(tether_pitch_rate_lpf.out) < T_PITCH_RATE_KNEE) tether_pitch_rate_gain = T_MIN_PITCH_RATE_GAIN;
    else tether_pitch_rate_gain = limit(T_MIN_PITCH_RATE_GAIN + ((abs(tether_pitch_rate_lpf.out) - T_PITCH_RATE_KNEE)*T_PITCH_RATE_SLOPE), T_MIN_PITCH_RATE_GAIN, T_MAX_PITCH_RATE_GAIN);

    tether_pitch_rate_gain_lpf.compute(tether_pitch_rate_gain, TETHER_RATE_GAIN_LPF_TAU);
    tether_pitch_rate_compensated = (tether_pitch_rate_lpf.out * tether_pitch_rate_gain_lpf.out);

    tether_pitch_rate_PID.compute(tether_pitch_attitude_PID.out, tether_pitch_rate_compensated);  // add inner rate loop
    tether_pitch_PID_rlm.compute(tether_pitch_rate_PID.out, tether_rate_limit*ITER_PERIOD); // rate limit output to reduce noise from rapid tether movement

    tether_pitch_PID_rlm.out = limit(tether_pitch_PID_rlm.out, -15.0, 15.0);

    // roll tether control law
    tether_roll_cmd = 0.0;

    //enable the following code to provide integrator controlled tether position
        //rx_integrator_delta = roll_rx_cmd;
        //if(abs(rx_integrator_delta) < ROLL_RX_DEAD_ZONE) rx_integrator_delta = 0.0; // joystick dead zone, enabled if in tether mode to eliminate tether cmd integrator drift
        //tether_roll_cmd = tether_roll_cmd + (rx_integrator_delta * ITER_PERIOD * 0.2);  // integrate rx stick to create tether angle cmd
        //tether_roll_cmd = (tether_roll_cmd * cos_yaw_delta) + (tether_pitch_cmd * sin_yaw_delta);
        //tether_roll_cmd = limit(tether_roll_cmd, -25, 25);

    tether_roll_angle_rlm.compute(tether_roll_angle, tether_angle_rate_limit*ITER_PERIOD);  // rate limit and filter to reduce noise from moving tether
    tether_roll_angle_lpf.compute(tether_roll_angle_rlm.out, TETHER_ANGLE_LPF_TAU);    // smooth tether loop response to fast tether movement
    tether_roll_rate_lpf.compute(limit(-tether_z_gyro.rate, -tether_angle_rate_limit, tether_angle_rate_limit), TETHER_RATE_LPF_TAU);

    tether_roll_attitude_PID.compute(tether_roll_cmd, tether_roll_angle_lpf.out);  // position outer loop (deg/sec output)

    if(abs(tether_roll_rate_lpf.out) < T_ROLL_RATE_KNEE) tether_roll_rate_gain = T_MIN_ROLL_RATE_GAIN;
    else tether_roll_rate_gain = limit(T_MIN_ROLL_RATE_GAIN + ((abs(tether_roll_rate_lpf.out) - T_ROLL_RATE_KNEE)*T_ROLL_RATE_SLOPE), T_MIN_ROLL_RATE_GAIN, T_MAX_ROLL_RATE_GAIN);

    tether_roll_rate_gain_lpf.compute(tether_roll_rate_gain, TETHER_RATE_GAIN_LPF_TAU);
    tether_roll_rate_compensated = (tether_roll_rate_lpf.out * tether_roll_rate_gain_lpf.out);

    tether_roll_rate_PID.compute(tether_roll_attitude_PID.out, tether_roll_rate_compensated);  // add inner rate loop
    tether_roll_PID_rlm.compute(tether_roll_rate_PID.out, tether_rate_limit*ITER_PERIOD); // rate limit output to reduce noise from rapid tether movement

    tether_roll_PID_rlm.out = limit(tether_roll_PID_rlm.out, -15.0, 15.0);
 }

void xyz_damper_control_laws(void)     // c/l to dampen linear movement in xyz axis
                                       // this c/l calculates linear velocity by integrating the linear acceleration.
                                       // integrator drift compensation is used to reduce the effects of accelerometer offsets
{
    // pitch
    tether_pitch_rate_lpf2.compute(tether_pitch_rate_lpf.out, 0.05);
    tether_y_velocity = tether_pitch_rate_lpf2.out * DPS_TO_MPS * limit(platform_altitude, 1.5,  tx_altitude_lim);  // determine tether linear "velocity" at platform in meters/sec
 
    tether_y_velocity = limit(tether_y_velocity, -VEL_LIM, VEL_LIM);

    if((abs(yaw_rate_lpf.out) > 1.5) & (mode_sw_posn == MIDDLE)) integrator_hold = true;  //hold velocity integrator output constant while panning
    else integrator_hold = false;                                                         //to reduce damper output error output from inaccurate accelerator output during panning
                                                                                         //damper error causes platform to drift in x/y axis during pan if integrator value is not held.
    if(integrator_hold == false)
    {
        y_velocity += ((y_linear_accel + y_linear_accel_last)/2 * ITER_PERIOD);  //integrate linear accel to determine velocity
        y_velocity = (TETHER_COMP_FLTR_COEFF * y_velocity) + ((1-TETHER_COMP_FLTR_COEFF) * tether_y_velocity);
        y_velocity = limit(y_velocity, -VEL_LIM, VEL_LIM);  // limit velocity authority into the primary control law
    }
    y_damper = ((DAMPER_P_GAIN * y_linear_accel) + (DAMPER_I_GAIN * y_velocity));  // PI control law
    y_damper = limit(y_damper, -DAMPER_LIM, DAMPER_LIM);

    y_damper_lpf.compute(y_damper, DAMPER_LPF_TAU);   // filter control law output to reduce noise
    
    // roll
    tether_roll_rate_lpf2.compute(tether_roll_rate_lpf.out, 0.05);
    tether_x_velocity = tether_roll_rate_lpf2.out * DPS_TO_MPS * limit(platform_altitude, 1.5, tx_altitude_lim);  // determine tether linear "velocity" at platform in meters/sec
    tether_x_velocity = limit(tether_x_velocity, -VEL_LIM, VEL_LIM);

    if(integrator_hold == false)
    {
        x_velocity += ((x_linear_accel + x_linear_accel_last)/2 * ITER_PERIOD);  //integrate linear accel to determine velocity
        x_velocity = (TETHER_COMP_FLTR_COEFF * x_velocity) + ((1-TETHER_COMP_FLTR_COEFF) * tether_x_velocity);
        x_velocity = limit(x_velocity, -VEL_LIM, VEL_LIM);  // limit velocity authority into the primary control law
    }
    x_damper = ((DAMPER_P_GAIN * x_linear_accel) + (DAMPER_I_GAIN * x_velocity));  // PI control law
    x_damper = limit(x_damper, -DAMPER_LIM, DAMPER_LIM);

    x_damper_lpf.compute(x_damper, DAMPER_LPF_TAU);   // filter control law output to reduce noise
}

void primary_control_laws(void)
{
    // throttle control law

    //throttle gain schedule
    throttle_cmd = throttle_rx_cmd;
    if(throttle_rx_cmd <= 50) throttle_cmd = scale(throttle_rx_cmd, 0,50, 0, flight_throttle_value);
    if(throttle_rx_cmd > 50) throttle_cmd = flight_throttle_value;                                         // throttle deadzone between 50% and 70%
    if(throttle_rx_cmd > 70) throttle_cmd = scale(throttle_rx_cmd, 70,100, flight_throttle_value, 90);     // limit throttle cmd to 90% to allow pitch/roll content

    if(throttle_rx_cmd < THROTTLE_START_VALUE)  reset_platform_cl(); // reset all integrators, filters, rate limits and PIDs if throttle is down
    
    // pitch attitude control law
    if(pitch_ctrl_enable)
    {
        if(tether_mode) pitch_cmd = pitch_rx_cmd + (TETHER_GAIN* tether_pitch_PID_rlm.out) - (DAMPER_GAIN * y_damper_lpf.out);
        else pitch_cmd = pitch_rx_cmd - (DAMPER_GAIN * y_damper_lpf.out);

        if(measure_step_response_enable) pitch_cmd = pitch_rx_cmd;

        pitch_cmd = limit(pitch_cmd, -25, 25);
        pitch_attitude_PID.compute(pitch_cmd, -pitch_angle);  // position outer loop
        pitch_attitude_PID.out = limit(pitch_attitude_PID.out, -50, 50);

//!!!!
//pitch_attitude_PID.I = (pitch_attitude_PID.I * cos_yaw_delta) - (roll_attitude_PID.I * sin_yaw_delta);

        pitch_rate_PID.compute(pitch_attitude_PID.out, (-platform_x_gyro.rate * PITCH_RATE_GAIN));  // add inner rate loop
        pitch_rate_PID.out = limit(pitch_rate_PID.out, -50, 50);
        pitch_rate_PID.out = limit(pitch_rate_PID.out, -(throttle_rx_cmd-THROTTLE_START_VALUE), (throttle_rx_cmd-THROTTLE_START_VALUE));  //limit PID output during throttle up/dn
    }
    else 
    {
        pitch_rate_PID.reset(); // disable axis control if not enabled
        pitch_attitude_PID.reset();
    }

    // roll attitude control law 
    if(roll_ctrl_enable)
    {
        if(tether_mode) roll_cmd = roll_rx_cmd + (TETHER_GAIN * tether_roll_PID_rlm.out) - (DAMPER_GAIN * x_damper_lpf.out);
        else roll_cmd = roll_rx_cmd - (DAMPER_GAIN * x_damper_lpf.out);

        if(measure_step_response_enable) roll_cmd = roll_rx_cmd;

        roll_cmd = limit(roll_cmd, -25, 25);
        roll_attitude_PID.compute(roll_cmd, -roll_angle);  // position outer loop
        roll_attitude_PID.out = limit(roll_attitude_PID.out, -50, 50);

//!!!!
//roll_attitude_PID.I = (roll_attitude_PID.I * cos_yaw_delta) + (pitch_attitude_PID.I * sin_yaw_delta);

        roll_rate_PID.compute(roll_attitude_PID.out, (platform_y_gyro.rate * ROLL_RATE_GAIN));  // add inner rate loop
        roll_rate_PID.out = limit(roll_rate_PID.out, -50, 50);
        roll_rate_PID.out = limit(roll_rate_PID.out, -(throttle_rx_cmd-THROTTLE_START_VALUE), (throttle_rx_cmd-THROTTLE_START_VALUE));  //limit PID output during throttle up/dn
    }
    else 
    {
        roll_rate_PID.reset(); // disable axis control if not enabled
        roll_attitude_PID.reset();
    }
    // yaw control laws
    if(yaw_ctrl_enable)
    {
        yaw_cmd = yaw_rx_angle_cmd + pan_cmd;

        yaw_cmd_rlm.compute(yaw_cmd, 60 * ITER_PERIOD);

        yaw_cmd_lpf.compute(yaw_cmd_rlm.out, 0.8);
        yaw_attitude_PID.compute(yaw_cmd_lpf.out, yaw_angle);  // position outer loop
        yaw_attitude_PID.out = limit(yaw_attitude_PID.out, -40, 40);

        yaw_rate_PID.compute((yaw_attitude_PID.out), (platform_z_gyro.rate * YAW_RATE_GAIN));  // add inner rate loop
        yaw_rate_PID.out = limit(yaw_rate_PID.out, -20, 20);  // limit to eliminate motor cmd saturation.
        yaw_rate_PID.out = limit(yaw_rate_PID.out, -(throttle_rx_cmd-THROTTLE_START_VALUE), (throttle_rx_cmd-THROTTLE_START_VALUE));  //limit PID output during throttle up/dn
    }
    else 
    {
        yaw_rate_PID.reset(); // disable axis if not enabled
        yaw_attitude_PID.reset();
    }
}

void drive_motors(void)  // motor drive algorithm assumes "+" frame construction
{
    // drive pitch esc outputs
    min_mtr_lim = 0.0;  // allow minimum motor cmd of zero if not in flight
    if (throttle_rx_cmd > MIN_MOTOR_FLIGHT_CMD) min_mtr_lim = MIN_MOTOR_FLIGHT_CMD;  // limit minimum motor cmd during flight to ensure motors don't stop for small PID outputs
 
    if(pitch_ctrl_enable & throttle_enable & (throttle_rx_cmd > THROTTLE_START_VALUE))
    {
        pitch_front_mtr_cmd = throttle_cmd - pitch_rate_PID.out - yaw_rate_PID.out; // mix throttle and control law outputs to create motor commands
        pitch_rear_mtr_cmd = throttle_cmd + pitch_rate_PID.out - yaw_rate_PID.out;

        pitch_front_mtr_cmd = limit(pitch_front_mtr_cmd, min_mtr_lim, 100);
        pitch_rear_mtr_cmd = limit(pitch_rear_mtr_cmd, min_mtr_lim, 100);

        pitch_front_mtr_cmd = scale(pitch_front_mtr_cmd, 0, 100, 0.001, 0.00198);  // scale motor cmd to milliseconds pulse width for esc
        pitch_rear_mtr_cmd = scale(pitch_rear_mtr_cmd, 0, 100, 0.001, 0.00198);    // do not exceed 0.00198 msec of .002 msec frame (limit to 98% pulse width)

        SetDCOC1PWM(pitch_front_mtr_cmd/T2_SEC_PER_BIT); // pitch front pwm output
        SetDCOC2PWM(pitch_rear_mtr_cmd/T2_SEC_PER_BIT); // pitch rear pwm output
    }
    else
    {
        if(!calibrate_ESCs_enable & !measure_platform_vib_enable)   // set esc pulse width to minimum if axis not enabled and not measuring vib or calibrating ESCs
        {
            SetDCOC1PWM(0.001/T2_SEC_PER_BIT);
            SetDCOC2PWM(0.001/T2_SEC_PER_BIT);
        }
    }

    // drive roll esc outputs
    if(roll_ctrl_enable & throttle_enable & (throttle_rx_cmd > THROTTLE_START_VALUE))
    {
        roll_left_mtr_cmd = throttle_cmd + roll_rate_PID.out + yaw_rate_PID.out; // mix throttle and control law outputs to create motor commands
        roll_right_mtr_cmd = throttle_cmd - roll_rate_PID.out + yaw_rate_PID.out;
        
        roll_left_mtr_cmd = limit(roll_left_mtr_cmd, min_mtr_lim, 100); // limit cmd between 0 and 98%  (100% exceeds ESC pulse width)
        roll_right_mtr_cmd = limit(roll_right_mtr_cmd, min_mtr_lim, 100);

        roll_left_mtr_cmd = scale(roll_left_mtr_cmd, 0, 100, 0.001, 0.00198);  // scale motor cmd to milliseconds pulse width for esc
        roll_right_mtr_cmd = scale(roll_right_mtr_cmd, 0, 100, 0.001, 0.00198);  // do not exceed 0.00198 msec of .002 msec frame (limit to 98% pulse width)
        
        SetDCOC4PWM(roll_left_mtr_cmd/T2_SEC_PER_BIT); // roll left pwm output
        SetDCOC3PWM(roll_right_mtr_cmd/T2_SEC_PER_BIT); // roll right pwm output
    }
    else
    {
        if(!calibrate_ESCs_enable & !measure_platform_vib_enable)   // set esc pulse width to minimum if axis not enabled and not measuring vib or calibrating ESCs
        {
            SetDCOC4PWM(0.001/T2_SEC_PER_BIT);
            SetDCOC3PWM(0.001/T2_SEC_PER_BIT);
        }
    }
}

//////////////////////////////////
// Main Application
//////////////////////////////////

int __attribute__((optimize("O0"))) main(void)  // __attribute__((optimize("O0"))) compiler attribute to turn off optimization for a specific function
{                                               // see XC32 Compilier Users Guide Section 12.2.1 for further info
    configure_io();
    configure_interrupts();
    initialize_MPUs();
    initialize_MPUs();      //i2c slave sensor power up issue requires initialization of slave twice (otherwise results in slave accel and gyro gain errors and eroneous data)
    initialize_baro();
    initialize_PIDs();
    delay(0.1);
    platform_initialized = true;
    delay(0.2);

    // test utility control
    thrust_test_enable = false;
    calibrate_ESCs_enable = false;
    measure_platform_vib_enable = false;
    measure_step_response_enable = false;
    get_sensor_offsets_enable = false;
    
    if (thrust_test_enable) thrust_test();
    if (calibrate_ESCs_enable) calibrate_ESCs();
    if (measure_platform_vib_enable) measure_platform_vib();
    if (measure_step_response_enable) measure_step_response();

    // mode control
    pitch_ctrl_enable = true;
    roll_ctrl_enable = true;
    yaw_ctrl_enable = true;

    while(absolute_time < 5.0);  // delay to let platform stabilize
  
    if(yaw_rx_rate_cmd > 10 & throttle_rx_cmd < 1)  erase_flash();   // moving yaw joystick full right (inboard) during power up erases flash memory
    if(yaw_rx_rate_cmd < -10 & throttle_rx_cmd < 1) camera_type = CANON_S95;  // moving yaw joystick full left (outboard) during power up switches to Canon S95
   
    check_sensors();

    pan_increment = -30.0; //30 degrees used for 28mm lens (same for Ricoh GR and Canon S95

    while(true)  // main background loop
    {
        if(camera_type == RICOH_GR) ricoh_gr_routine();
        if(camera_type == CANON_S95) canon_s95_routine();
    }
}
