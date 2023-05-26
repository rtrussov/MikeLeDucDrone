// File:   platform config.h
//
// c 2015 IowaKAPer / Mike LeDuc
//

#ifndef PLATFORM_CONFIG_H
#define	PLATFORM_CONFIG_H

///////////////////////////////////////////////////////////
// PIC Power-Up Settings  (see file PIC32ConfigSet.pdf)
///////////////////////////////////////////////////////////

    // DEVCFG3
    // USERID = No Setting
    #pragma config PMDL1WAY = OFF            // Peripheral Module Disable Configuration (Allow only one reconfiguration)
    #pragma config IOL1WAY = OFF             // Peripheral Pin Select Configuration (Allow only one reconfiguration)
    #pragma config FUSBIDIO = OFF            // USB USID Selection (Controlled by the Port)
    #pragma config FVBUSONIO = OFF           // USB VBUS ON Selection (Controlled by Port)

    // DEVCFG2 (set sysclk to 40 mhz)
    #pragma config FPLLIDIV = DIV_2         // PLL Input Divider
    #pragma config FPLLMUL = MUL_20         // PLL Multiplier
    #pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider
    #pragma config UPLLEN = OFF             // USB PLL Enable (Disabled and Bypassed)
    #pragma config FPLLODIV = DIV_2         // System PLL Output Clock Divider

    // DEVCFG1 (sets pbclk to sysclk/4)
    #pragma config FNOSC = FRCPLL           // Oscillator Selection Bits - FRCPLL (internal RC osc with PLL) or PRILL (primary external osc)
    #pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (disabled)
    #pragma config IESO = OFF               // Internal/External Switch Over (two speed startup) ON or OFF (use OFF for internal osc)
    #pragma config POSCMOD = OFF            // Primary Oscillator Configuration - XT or OFF (use OFF for internal osc)
    #pragma config OSCIOFNC = OFF           // CLKO Output Signal Disabled on the OSCO Pin
    #pragma config FPBDIV = DIV_2           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/2)
                                            // this prescaler is used in #define for PB_CLK_FREQ below
    #pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
    #pragma config WDTPS = PS8              // PS8 Watchdog Timer Postscaler (1:8), 8 msec timeout
    #pragma config WINDIS = OFF             // Watchdog Timer Window Enable (Watchdog Timer is in Non-Window Mode)
    #pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT disabled at power up and enabled after interrups are turned on)
    #pragma config FWDTWINSZ = WINSZ_25     // Watchdog Timer Window Size (not used)

    // DEVCFG0
    #pragma config DEBUG = OFF
    #pragma config JTAGEN = OFF             // JTAG Enable (JTAG Port Disabled)
    #pragma config ICESEL = ICS_PGx1        // ICE/ICD Comm Channel Select (Communicate on PGEC1/PGED1)
    #pragma config PWP = OFF                // Program Flash Write Protect (Disable)
    #pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
    #pragma config CP = OFF                 // Code Protect (Protection Disabled)

#endif	/* PLATFORM_CONFIG_H */

