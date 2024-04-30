// Init_Ports.c
// Runs on MSP432
// Provide functions that initialize ports 
// Prof. Christman
// 1/8/2023

/* These functions will be used in all of the labs for CPET253

There is an initialization function for each of the ports 1-10 and J.

The initializations will be different for each lab.
*/

#include "msp.h"


// ------------Port1_Init------------
// First set the pin directions
// Enable resistors for inputs
// Set input resistors for pull-up or pull-down
// Enable alternate pin functions, if appliable
// Drive initial values on outputs
// Input: none
// Output: none
void Port1_Init(void){
    P1DIR &= 0b11101101;
    P1REN |= 0b00010010;
    P1OUT |= 0b00010010;
  // Use the port direction register to configure pins as inputs and output
  // 	-A 1 sets a pin as an output. Use |= with a bit mask to set 1's.
  // 	-A 0 sets a pin as an input. Use &= with a bit mask to set 0's.
  // Use the resistor enable register to enable resistors for inputs that need them
  // Use the port output register to configure the resistors as pull-ups or pull-downs
  // Use the PxSEL0 and PxSEL1 registers to enable alternate pin functions, if applicable
  // Use the port output register to put outputs to an initial state
}

// ------------Port2_Init------------
// First set the pin directions
// Enable resistors for inputs
// Set input resistors for pull-up or pull-down
// Enable alternate pin functions, if appliable
// Drive initial values on outputs
// Input: none
// Output: none
void Port2_Init(void){
    P2DIR  |=  0b11000000;   //set pin 6 and 7 as outputs
    P2OUT  &= ~0b11000000;   //set those pins low
    P2SEL1 &= ~0b11000000;   //set them as their primary function
    P2SEL0 |=  0b11000000;
  // Use the port direction register to configure pins as inputs and output
  // 	-A 1 sets a pin as an output. Use |= with a bit mask to set 1's.
  // 	-A 0 sets a pin as an input. Use &= with a bit mask to set 0's.
  // Use the resistor enable register to enable resistors for inputs that need them
  // Use the port output register to configure the resistors as pull-ups or pull-downs
  // Use the PxSEL0 and PxSEL1 registers to enable alternate pin functions, if applicable
  // Use the port output register to put outputs to an initial state
}

// ------------Port3_Init------------
// First set the pin directions
// Enable resistors for inputs
// Set input resistors for pull-up or pull-down
// Enable alternate pin functions, if appliable
// Drive initial values on outputs
// Input: none
// Output: none
void Port3_Init(void){
    P3DIR |= 0b11000000;    //set the nSLEEP pins as outputs
    P3OUT &= ~0b11000000;   //set the nSLEEP pins to 0 (sleep mode, needs 1 to move)
  // Use the port direction register to configure pins as inputs and output
  // 	-A 1 sets a pin as an output. Use |= with a bit mask to set 1's.
  // 	-A 0 sets a pin as an input. Use &= with a bit mask to set 0's.
  // Use the resistor enable register to enable resistors for inputs that need them
  // Use the port output register to configure the resistors as pull-ups or pull-downs
  // Use the PxSEL0 and PxSEL1 registers to enable alternate pin functions, if applicable
  // Use the port output register to put outputs to an initial state
}

// ------------Port4_Init------------
// First set the pin directions
// Enable resistors for inputs
// Set input resistors for pull-up or pull-down
// Enable alternate pin functions, if appliable
// Drive initial values on outputs
// Input: none
// Output: none
void Port4_Init(void){
  // Use the port direction register to configure pins as inputs and output
  // 	-A 1 sets a pin as an output. Use |= with a bit mask to set 1's.
  // 	-A 0 sets a pin as an input. Use &= with a bit mask to set 0's.
  P4DIR |= 0xFF;   //set all as outputs
  P4OUT &= ~0xFF;  //turn off initially
}

// ------------Port5_Init------------
// First set the pin directions
// Enable resistors for inputs
// Set input resistors for pull-up or pull-down
// Enable alternate pin functions, if appliable
// Drive initial values on outputs
// Input: none
// Output: none
void Port5_Init(void){
    P5DIR |= 0b00110000;    //set PH pins as outputs
    P5OUT &= ~0b00110000;   //set PH pins to 0 (forward mode). needs 1 for reverse.
    P5DIR |= BIT3;    //set set pin3 as output (CTRL EVEN)
    P5OUT &= ~BIT3;   //set to 0
  // Use the port direction register to configure pins as inputs and output
  // 	-A 1 sets a pin as an output. Use |= with a bit mask to set 1's.
  // 	-A 0 sets a pin as an input. Use &= with a bit mask to set 0's.
  // Use the resistor enable register to enable resistors for inputs that need them
  // Use the port output register to configure the resistors as pull-ups or pull-downs
  // Use the PxSEL0 and PxSEL1 registers to enable alternate pin functions, if applicable
  // Use the port output register to put outputs to an initial state
}

// ------------Port6_Init------------
// First set the pin directions
// Enable resistors for inputs
// Set input resistors for pull-up or pull-down
// Enable alternate pin functions, if appliable
// Drive initial values on outputs
// Input: none
// Output: none
void Port6_Init(void){
  // Use the port direction register to configure pins as inputs and output
  // 	-A 1 sets a pin as an output. Use |= with a bit mask to set 1's.
  // 	-A 0 sets a pin as an input. Use &= with a bit mask to set 0's.
  // Use the resistor enable register to enable resistors for inputs that need them
  // Use the port output register to configure the resistors as pull-ups or pull-downs
  // Use the PxSEL0 and PxSEL1 registers to enable alternate pin functions, if applicable
  // Use the port output register to put outputs to an initial state
}

// ------------Port7_Init------------
// First set the pin directions
// Enable resistors for inputs
// Set input resistors for pull-up or pull-down
// Enable alternate pin functions, if appliable
// Drive initial values on outputs
// Input: none
// Output: none
void Port7_Init(void){
    P7DIR |= 0xFF;  //set to all outputs initially
    P7OUT &= ~0xFF; //turn off
  // Use the port direction register to configure pins as inputs and output
  // 	-A 1 sets a pin as an output. Use |= with a bit mask to set 1's.
  // 	-A 0 sets a pin as an input. Use &= with a bit mask to set 0's.
  // Use the resistor enable register to enable resistors for inputs that need them
  // Use the port output register to configure the resistors as pull-ups or pull-downs
  // Use the PxSEL0 and PxSEL1 registers to enable alternate pin functions, if applicable
  // Use the port output register to put outputs to an initial state
}

// ------------Port8_Init------------
// First set the pin directions
// Enable resistors for inputs
// Set input resistors for pull-up or pull-down
// Enable alternate pin functions, if appliable
// Drive initial values on outputs
// Input: none
// Output: none
void Port8_Init(void){
  // Use the port direction register to configure pins as inputs and output
  // 	-A 1 sets a pin as an output. Use |= with a bit mask to set 1's.
  // 	-A 0 sets a pin as an input. Use &= with a bit mask to set 0's.
  // Use the resistor enable register to enable resistors for inputs that need them
  // Use the port output register to configure the resistors as pull-ups or pull-downs
  // Use the PxSEL0 and PxSEL1 registers to enable alternate pin functions, if applicable
  // Use the port output register to put outputs to an initial state
}

// ------------Port9_Init------------
// First set the pin directions
// Enable resistors for inputs
// Set input resistors for pull-up or pull-down
// Enable alternate pin functions, if appliable
// Drive initial values on outputs
// Input: none
// Output: none
void Port9_Init(void){
    P9DIR |= BIT2;  //set as an output
    P9OUT &= ~BIT2; //turn off
  // Use the port direction register to configure pins as inputs and output
  // 	-A 1 sets a pin as an output. Use |= with a bit mask to set 1's.
  // 	-A 0 sets a pin as an input. Use &= with a bit mask to set 0's.
  // Use the resistor enable register to enable resistors for inputs that need them
  // Use the port output register to configure the resistors as pull-ups or pull-downs
  // Use the PxSEL0 and PxSEL1 registers to enable alternate pin functions, if applicable
  // Use the port output register to put outputs to an initial state
}

// ------------Port10_Init------------
// First set the pin directions
// Enable resistors for inputs
// Set input resistors for pull-up or pull-down
// Enable alternate pin functions, if appliable
// Drive initial values on outputs
// Input: none
// Output: none
void Port10_Init(void){
  // Use the port direction register to configure pins as inputs and output
  // 	-A 1 sets a pin as an output. Use |= with a bit mask to set 1's.
  // 	-A 0 sets a pin as an input. Use &= with a bit mask to set 0's.
  // Use the resistor enable register to enable resistors for inputs that need them
  // Use the port output register to configure the resistors as pull-ups or pull-downs
  // Use the PxSEL0 and PxSEL1 registers to enable alternate pin functions, if applicable
  // Use the port output register to put outputs to an initial state
}

// ------------PortJ_Init------------
// First set the pin directions
// Enable resistors for inputs
// Set input resistors for pull-up or pull-down
// Enable alternate pin functions, if appliable
// Drive initial values on outputs
// Input: none
// Output: none
void PortJ_Init(void){
  // Use the port direction register to configure pins as inputs and output
  // 	-A 1 sets a pin as an output. Use |= with a bit mask to set 1's.
  // 	-A 0 sets a pin as an input. Use &= with a bit mask to set 0's.
  // Use the resistor enable register to enable resistors for inputs that need them
  // Use the port output register to configure the resistors as pull-ups or pull-downs
  // Use the PxSEL0 and PxSEL1 registers to enable alternate pin functions, if applicable
  // Use the port output register to put outputs to an initial state
}
