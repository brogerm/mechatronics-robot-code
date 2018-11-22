/*
 * File:   number_one_draft_pic.c
 * Authors: Roger Black, Ryan Clark, Brian Carlson, Mitch Slater
 *
 * Created on November 21, 2018, 5:09 PM
 */


#include "xc.h"

// Select oscillator
#pragma config FNOSC = FRC       // 8 MHz FRC oscillator

// Define the possible states
enum { FIND_DISPENSER, REVERSE, FORWARD, LOADING_AND_SORTING, SHOOTING, CONFIGURATION } state;

int main(void) {
    
    while(1)
	{
	}
    
    return 0;
}
