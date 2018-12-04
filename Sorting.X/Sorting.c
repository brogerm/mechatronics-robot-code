/*
 * File:   Sorting.c
 * Author: bcarlso4
 *
 * Created on November 29, 2018, 8:29 AM
 */


#include "xc.h"
#define FCY 4000000UL
#include <libpic30.h>

// Select oscillator
#pragma config FNOSC = FRC       // 8 MHz FRC oscillator

void config_pwm() {
    // Clear control bits initially
    OC2CON1 = 0;
    OC2CON2 = 0;
   
    // Set period and duty cycle
    OC2R = 8000;                // Set Output Compare value to achieve
                                // desired duty cycle. This is the number
                                // of timer counts when the OC should send
                                // the PWM signal low. The duty cycle as a
                                // fraction is OC1R/OC1RS.
    OC2RS = 39999;               // Period of OC1 to achieve desired PWM 
                                // frequency, FPWM. See Equation 15-1
                                // in the datasheet. For example, for
                                // FPWM = 1 kHz, OC1RS = 3999. The OC1RS 
                                // register contains the period when the
                                // SYNCSEL bits are set to 0x1F (see FRM)
    
    // Configure OC1
    OC2CON1bits.OCTSEL = 0b111; // System (peripheral) clock as timing source
    OC2CON2bits.SYNCSEL = 0x1F; // Select OC1 as synchronization source
                                // (self synchronization) -- Although we
                                // selected the system clock to determine
                                // the rate at which the PWM timer increments,
                                // we could have selected a different source
                                // to determine when each PWM cycle initiates.
                                // From the FRM: When the SYNCSEL<4:0> bits
                                // (OCxCON2<4:0>) = 0b11111, they make the
                                // timer reset when it reaches the value of
                                // OCxRS, making the OCx module use its
                                // own Sync signal.
    OC2CON2bits.OCTRIG = 0;     // Synchronizes with OC1 source instead of
                                // triggering with the OC1 source
    OC2CON1bits.OCM = 0b110;    // Edge-aligned PWM mode
}

void config_ad()
{
    _TRISA3 = 1;		// TRISA/B, pg. 45 datasheet
    _ANSA3 = 1;			// ANSA/B, pg. 136-137
    _CSS14 = 1;			// AD1CSSH/L, pg. 217
 



	/*** Select Voltage Reference Source ***/
	// use AVdd for positive reference
	_PVCFG = 0b00;		// AD1CON2<15:14>, pg. 212-213 datasheet
	// use AVss for negative reference
	_NVCFG = 0;			// AD1CON2<13>


	/*** Select Analog Conversion Clock Rate ***/
	// make sure Tad is at least 600ns, see Table 29-41 datasheet
	_ADCS = 0b00000011;	// AD1CON3<7:0>, pg. 213 datasheet


	/*** Select Sample/Conversion Sequence ***/
	// use auto-convert
	_SSRC = 0b0111;		// AD1CON1<7:4>, pg. 211 datasheet
	// use auto-sample
	_ASAM = 1;			// AD1CON1<2>
	// choose a sample time >= 1 Tad, see Table 29-41 datasheet
	_SAMC = 0b00001;		// AD1CON3<12:8>


	/*** Choose Analog Channels to be Used ***/
	// scan inputs
	_CSCNA = 1;			// AD1CON2<10>

	/*** Select How Results are Presented in Buffer ***/
	// set 12-bit resolution
	_MODE12 = 1;		// AD1CON1<10>
	// use absolute decimal format
	_FORM = 0b00;			// AD1CON1<9:8>
	// load results into buffer determined by converted channel, e.g. ch AN12 
    // results appear in ADC1BUF12
	_BUFREGEN = 1;		// AD1CON2<11>


	/*** Select Interrupt Rate ***/
	// interrupt rate should reflect number of analog channels used, e.g. if 
    // 5 channels, interrupt every 5th sample
	_SMPI = 0b00001;		// AD1CON2<6:2>


	/*** Turn on A/D Module ***/
	_ADON = 1;			// AD1CON1<15>
}

int main(void) {
    config_pwm();
    config_ad();
    
    while (1)
     {
//        int p_diode_v = ADC1BUF14;

//        if (p_diode_v <= 1000) {
//            OC2R = 5500;
//        }                                      
//        else if (p_diode_v <= 2750) {     // Black ball
//            OC2R = 4000;
//        }
//        else {                            // White ball
//            OC2R = 7000;
//        }
        
//        OC2R = 4000;
//        __delay_ms(1000);
//        OC2R = 2000;
//        __delay_ms(1000);
//        OC2R = 4000;
//        __delay_ms(1000);
     }
    
    return 0;
}
