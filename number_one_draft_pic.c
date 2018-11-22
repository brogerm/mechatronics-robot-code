/*
 * File:   number_one_draft_pic.c
 * Authors: Roger Black, Ryan Clark, Brian Carlson, Mitch Slater
 *
 * Created on November 21, 2018, 5:09 PM
 */


#include "xc.h"

// Select oscillator
#pragma config FNOSC = FRC       // 8 MHz FRC oscillator

// ------------------------- CONFIGURE VARIABLES -------------------------
enum { FIND_DISPENSER, REVERSE, FORWARD, LOADING_AND_SORTING, SHOOTING, CONFIGURATION } state; // defines possible states
state = FIND_DISPENSER; // sets initial state
int white_ball_max = 30;
int black_ball_max = 20;
int white_ball_count = 0;
int black_ball_count = 0;
int rotations_to_shooting_position = 20; 

// ------------------------- FUNCTIONS -------------------------
int find_dispenser (void) {
	// Initialize IR scanner
	// rotate robot
	// while (dispenser not found) {
		// continue rotating
	// }
	state = REVERSE;
	return 0;
}

int reverse (void) {
	while (wall_sensor1 == 0 || wall_sensor2 ==0) {
		// reverse both motors
	}
	state = LOADING_AND_SORTING;
	return 0;
}

int loading_and_sorting (void) {
	// interrupt laser
	// sense color
	// sort appropriately
	// increment ball count until capacity for either white or black has been reached
	return 0;
}

int forward (void) {
	while (forward_rotations != rotations_to_shooting_position) {
		// drive forward
	}
	state = CONFIGURATION;
	return 0;
}

int main(void) {
	// ------------------------- PIN CONFIGURATION -------------------------
		// configure pins here
		
	// ------------------------- CN INTERRUPT CONFIGURATION -------------------------
		// configure change notification interrupt here
		
	// ------------------------- TIMER INTERRUPT CONFIGURATION -------------------------
		// configure timer interrupt here
		
		
	// Start timer
	 
	
    while(1)
	{
		switch(state) {
			case FIND_DISPENSER: 
				// go to find_dispenser function
			case REVERSE:
				// go to reverse function
			case LOADING_AND_SORTING:
				// go to loading and sorting function
			case FORWARD:
				// go to forward function
			case CONFIGURATION:
				// go to configuration function
			case SHOOTING:
				// go to shooting function	
		}
	}
    
    return 0;
}
