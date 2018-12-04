/*
 * File:   Servo_Test.c
 * Author: shadow35
 *
 * Created on November 27, 2018, 8:55 AM
 */


#include <xc.h>
#define FCY 4000000UL
#include <libpic30.h>

// Select oscillator
#pragma config FNOSC = FRC       // 8 MHz FRC oscillator

int main()
{
    //-----------------------------------------------------------
    // CONFIGURE PWM1 USING OC1 (on pin 14)
    
    // Clear control bits initially
    OC1CON1 = 0;
    OC1CON2 = 0;
   
    // Set period and duty cycle
    OC1R = 6000;                // Set Output Compare value to achieve
                                // desired duty cycle. This is the number
                                // of timer counts when the OC should send
                                // the PWM signal low. The duty cycle as a
                                // fraction is OC1R/OC1RS.
    OC1RS = 39999;               // Period of OC1 to achieve desired PWM 
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
    
    
    //-----------------------------------------------------------
    // RUN

    // Wait and let the PWM do its job behind the scenes
    while(1) {
        OC1R = 4000;                // Right (Black)
        __delay_ms(1000);
        OC1R = 5500;                // Middle
        __delay_ms(1000);
        OC1R = 8000;                // Left (White)
        __delay_ms(1000);
    }

    return 0;
}
