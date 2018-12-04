/*
 * File:   Base_Mobility_V1.c
 * Author: shadow35
 *
 * Created on November 1, 2018, 8:45 AM
 */


#include "xc.h"

#pragma config FNOSC = FRC       // 8 MHz FRC oscillator

#define DIR1 _LATB9
#define DIR2 _LATB12

int steps_taken = 0;


void _ISR _OC1Interrupt(void)
{
    _OC1IF = 0;
    steps_taken++;
}



int main(void) {
    
    // configure i/o
    _TRISB12 = 0; // pins 15, 13 connected to motor direction
    _TRISB9 = 0;
    // Turn off analog
    _ANSB12 = 0;
    _TRISB7 = 0;
   
     // Configure OC1
    // Clear control bits initially
    OC1CON1 = 0;
    OC1CON2 = 0;
   // Map potentiometer value to duty cycle
    OC1R = 6000;
    OC1RS = 7999;               // Period of OC1 to achieve desired PWM 
                                // frequency, FPWM. See Equation 15-1
                                // in the datasheet. For example, for
                                // FPWM = 1 kHz, OC1RS = 3999. The OC1RS 
                                // register contains the period when the
                                // SYNCSEL bits are set to 0x1F (see FRM)
    
    // Configure OC1
    OC1CON1bits.OCTSEL = 0b111; // System (peripheral) clock as timing source
    OC1CON2bits.SYNCSEL = 0x1F; // Select OC1 as synchronization source
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
    OC1CON2bits.OCTRIG = 0;     // Synchronizes with OC1 source instead of
                                // triggering with the OC1 source
    OC1CON1bits.OCM = 0b110;    // Edge-aligned PWM mode
    
    _OC1IE = 1; // Enable Interrupt
    _OC1IF = 0; // Clear interrupt flag (IFS1 register)
    
    
    while (1)
    {
        /*
        if (steps_taken <= 1600) {
            DIR1 = 0;
            DIR2 = 0;
        } 
        
        else if (steps_taken <= 2000) {
            DIR2 = 1; // change direction 
        }
        else if (steps_taken <= 2800) {
            DIR2 = 0; // change direction 
        } 
        else {
            OC1R = 0;
        }
         */
    }
    
    return 0;
}
