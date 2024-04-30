#include <stdint.h>
#include <math.h>
#include "msp.h"
#include "piezo_buzzer.h"

void piezo_init(void){
    // sets SMCLK and stop mode
    TA0CTL |=  0x0200;
    TA0CTL &= ~0x01F0;
    // set CCR1 to reset/set
    TA0CCTL1 |= 0x00E0;

    // initializes P2.4 as a PWM output
    P2DIR |=  BIT4;
    P2OUT &= ~BIT4;
    P2SEL1 &= ~BIT4;
    P2SEL0 |=  BIT4;
}

void play_note(volatile uint16_t note){
    // sets CCR0 to note period
    TA0CCR0 = note;
    // sets CCR1 to half of the note period, to make a square wave
    TA0CCR1 = (note / 2);
    // set in up mode
    TA0CTL |= 0x0010;
    // delay so its a short beep
    Clock_Delay1ms(50);
    // set in stop mode
    TA0CTL &= ~0x0030;
    // instructions suggest a 50ms delay
    Clock_Delay1ms(50);
}
