/*
 * File:   servo_madness.c
 * Author: brogerm
 *
 * Created on December 1, 2018, 3:27 PM
 */


#include "xc.h"
#define FCY 4000000UL
#include <libpic30.h>
// Select oscillator
#pragma config FNOSC = FRC       // 8 MHz FRC oscillator

void timer2config() {
    //_RCDIV = 0;
    // Configure Timer1
    T2CONbits.TON = 1;       // Turn Timer2 on
    T2CONbits.TCKPS = 0b01;  // 1:8 prescaling
    T2CONbits.TCS = 0;       // Internal clock source (FOSC/2)
    TMR2 = 0;       // Reset Timer1
}

int main(void) {
    timer2config();
    
     // Clear control bits initially
    OC2CON1 = 0;
    OC2CON2 = 0;
   
    // Set period and duty cycle
    OC2R = 500;                
    OC2RS = 9999;              
    
    // Configure OC1
    OC2CON1bits.OCTSEL = 0b000; // System (peripheral) clock as timing source
    OC2CON2bits.SYNCSEL = 0x1F; // Select OC1 as synchronization source
                                
    OC2CON2bits.OCTRIG = 0;     // Synchronizes with OC1 source instead of
                                // triggering with the OC1 source
    OC2CON1bits.OCM = 0b110;    // Edge-aligned PWM mode
    __delay_ms(1000);
     while (1) {
         OC2R = 500;
         __delay_ms(1000);
         OC2R = 750;
         __delay_ms(1000);
         OC2R = 250;
         __delay_ms(1000);
     }
    
    return 0;
}
