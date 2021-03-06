/*
 * File:   Directed_Navigation.c
 * Author: shadow35
 *
 * Created on November 1, 2018, 8:45 AM
 */


#include "xc.h"
#define FCY 4000000UL
#include <libpic30.h>

#pragma config FNOSC = FRC       // 8 MHz FRC oscillator

#define DIR1 _LATB12
#define DIR2 _LATB9
#define BUMP1 _RB7
#define BUMP2 _RB8
int CENTER = 3;
int RIGHT = 4;
int LEFT = 2;

int steps_taken = 0;
int time = 0;

void config_ad()
{
    // Pin 9 (IR Sensor)
    _TRISA2 = 1;		// TRISA/B, pg. 45 datasheet
    _ANSA2 = 1;			// ANSA/B, pg. 136-137
    _CSS13 = 1;			// AD1CSSH/L, pg. 217
    
    // Pin 8 (Phototransistor)
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
	_SMPI = 0b00010;		// AD1CON2<6:2>


	/*** Turn on A/D Module ***/
	_ADON = 1;			// AD1CON1<15>
}

void timer1config(void) {
    // Configure Timer1
    T1CONbits.TON = 1;           // Turn on Timer1
    T1CONbits.TCS = 0;           // Select internal clock as source for
                        // Timer1 (this means that it will use
                        // the internal oscillator that runs at
                        // Fosc with instructions being executed
                        // at a rate of Fcy = Fosc/2)
    T1CONbits.TCKPS = 0b11;      // Divide-by-64 prescaling for Timer1
                        // (although the timer would normally
                        // increment at a rate of Fcy, this setting
                        // causes it to be incremented at a rate
                        // of Fcy/64)

    // Configure Timer1 interrupt
    _T1IP = 4;          // Select Timer1 interrupt priority
    _T1IE = 1;          // Enable Timer1 interrupt
    _T1IF = 0;          // Clear Timer1 interrupt flag
    
    // Select the period at which Timer1 interrupts occur. Whenever
    // TMR1 = PR1, the interrupt flag is set high and the program
    // is sent to the Timer1 interrupt service routine (ISR). The
    // actual elapsed time between interrupts obviously depends on
    // how fast the timer is being incremented, which obviously
    // depends on the selected oscillator, postscaling, and
    // prescaling.
    PR1 = 15625;        // Set period for Timer1 interrupt to occur
                        // PR1 = 15625 corresponds to a 4 Hz
                        // interrupt, which is equivalent to a 2 Hz
                        // blink rate because the state of the LED
                        // changes at 4 Hz, going on only half that
                        // often

    // Reset Timer1 count to zero initially
    TMR1 = 0;
}

void _ISR _OC1Interrupt(void)
{
    _OC1IF = 0;
    steps_taken++;
}

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void)
{
    // Clear Timer1 interrupt flag so that the program doesn't
    // just jump back to this function when it returns to the
    // while(1) loop.
    _T1IF = 0;

    time = 1;
}

void timer2config() {
    //_RCDIV = 0;
    // Configure Timer1
    T2CONbits.TON = 1;       // Turn Timer2 on
    T2CONbits.TCKPS = 0b01;  // 1:8 prescaling
    T2CONbits.TCS = 0;       // Internal clock source (FOSC/2)
    TMR2 = 0;       // Reset Timer1
}


int main(void) {
    
    timer1config();
    timer2config();
    _TRISA0 = 0;
    _ANSA0 = 0;
    config_ad();
    while(time != 1) {}
    // configure i/o
    _TRISB12 = 0; // pins 15, 13 connected to motor direction
    _TRISB9 = 0;
//    _TRISB7 = 1; // Bump Sensor (pin 11)
//    _TRISB8 = 1; // Bump Sensor (pin 12)
    // Turn off analog
    _ANSB12 = 0;
   
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
    
    // Clear control bits initially
    OC2CON1 = 0;
    OC2CON2 = 0;
   
    // Set period and duty cycle
    OC2R = 500;                // Set Output Compare value to achieve
                                // desired duty cycle. This is the number
                                // of timer counts when the OC should send
                                // the PWM signal low. The duty cycle as a
                                // fraction is OC1R/OC1RS.
    OC2RS = 9999;               // Period of OC1 to achieve desired PWM 
                                // frequency, FPWM. See Equation 15-1
                                // in the datasheet. For example, for
                                // FPWM = 1 kHz, OC1RS = 3999. The OC1RS 
                                // register contains the period when the
                                // SYNCSEL bits are set to 0x1F (see FRM)
    
    // Configure OC1
    OC2CON1bits.OCTSEL = 0b000; // System (peripheral) clock as timing source
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
    
     int ir_sensor = ADC1BUF13;
     //int p_diode_v = ADC1BUF14;
     int p_diode_v = 0;
     
    OC1R = 6000;
    DIR1 = 0;
    DIR2 = 0;
    while(ir_sensor <= 1700) {
        ir_sensor = ADC1BUF13;
    }
     steps_taken = 0;
     while(steps_taken < 325){}
     
     DIR2 = 1;
     steps_taken = 0;
     
     while(steps_taken <= 2500) {}
//     while(BUMP1 == 1 || BUMP2 == 1) {}
     OC1R = 0;
     
     int num_balls = 0;
     while (1)
     {
         OC2R = 500;
         __delay_ms(1000);
         OC2R = 750;
         __delay_ms(1000);
         OC2R = 500;
         __delay_ms(1000);
         OC2R = 250;
         __delay_ms(1000);
//         p_diode_v = ADC1BUF14;
//         if (p_diode_v <= 3000) {
//            OC2R = 500;
//        }
//        else if (p_diode_v <= 3500) {
//            OC2R = 750;
//        }
//        else {
//            OC2R = 250;
//        }
     }
     
    
    int at_middle = 0;
    int position = CENTER;
    int previous_position = CENTER;
    while (at_middle != 1)
    {
        if (steps_taken <= 700) {
            DIR1 = 1;
            DIR2 = 0;
        } 
        
//        else if (steps_taken <= 2000) {
//            DIR2 = 1; // change direction 
//        }
//        else if (steps_taken <= 2800) {
//            DIR2 = 0; // change direction 
//        } 
        else {
            OC1R = 0;
            at_middle = 1;
            steps_taken = 0;
        } 
    }
    
    while (1)
     {
        ir_sensor = ADC1BUF15;
        steps_taken = 0;
        
        
        if (ir_sensor >= 300) //originally 1861
        {
            
            OC1R = 0;
        }
        else 
        {
            if(position == CENTER) {
                while(steps_taken < 200) {
                    OC1R = 6000;
                    if(previous_position == LEFT) {
                        DIR1 = 1;
                        DIR2 = 1;
                        position = RIGHT;
                    }
                    else {
                        DIR1 = 0;
                        DIR2 = 0;
                        position = LEFT;
                    }
                }
                steps_taken = 0;
                previous_position = CENTER;
                
            }
            else if(position == LEFT) {
                while(steps_taken < 200) {
                    OC1R = 6000;
                    DIR1 = 1;
                    DIR2 = 1;
                }
                steps_taken = 0;
                previous_position = LEFT;
                position = CENTER;
            }
            else if(position == RIGHT) {
                while(steps_taken < 200) {
                    OC1R = 6000;
                    DIR1 = 0;
                    DIR2 = 0;
                }
                steps_taken = 0;                                                                                                            
                previous_position = RIGHT;
                position = CENTER;
            }
        }
     }
    
    return 0;
}
