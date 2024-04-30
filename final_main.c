// *****************************************************************************
// Name: Final Project
// File: final_main.c
// Author: Janos Banoczi-Ruof
// Pins:
//  Push Button:
//    P1.5
//    GND
//  Buzzer:
//    P2.4
//    GND
//  OLED:
//    GND: GND
//    VIN: 3.3V
//    Clk: P9.5
//    Data:P9.7
//    DC:  P9.6
//    CS:  P9.4
//    Rst: P9.3
//    3Vo: disconnected
//  Sensor:
//    VIN: 3.3V
//    3V:  disconnected
//    G:   GND
//    SCL: P6.3
//    SDA: P6.4
//    CS_A:P6.0
//    CS_M:P6.1
//    SDO (left): P6.5  (MISO AG)
//    SDO (right):P6.5  (MISO M)
// *****************************************************************************
// standard includes
#include "msp.h"
#include <stdint.h>
#include <stdbool.h>
// inc folder includes
#include "../inc/Init_Ports.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/SSD1306.h"
// local/custom includes
#include "LSM9DS1.h"
#include "piezo_buzzer.h"


#define CR   0x0D   // carriage return code

// globally accessable and redefineable variables
volatile int16_t x, y, z;
volatile bool wasInterrupt = false;

/**
 * Display x,y, and z starting on the second line
 * @param {int16_t} x axis*10 (-9999 to 9999)
 * @param {int16_t} y axis*10 (-9999 to 9999)
 * @param {int16_t} z axis*10 (-9999 to 9999)
 * @returns void
 */
void displayData(int16_t x, int16_t y, int16_t z, enum ss_t device){
    SSD1306_SetCursor(0,2);
    SSD1306_OutString("X Axis: ");
    SSD1306_OutSFix1(x);
    // if device = a, display G, if device = g, display dps, if device = m, display mGs
    if (device == A) {
        SSD1306_OutString("G");
    } else if (device == G) {
        SSD1306_OutString("dps");
    } else if (device == M) {
        SSD1306_OutString("mGs");
    } else {
        SSD1306_OutString(" ");
    }
    SSD1306_OutChar(CR);
    SSD1306_OutChar(CR);
    SSD1306_OutString("Y Axis: ");
    SSD1306_OutSFix1(y);
    if (device == A) {
        SSD1306_OutString("G");
    } else if (device == G) {
        SSD1306_OutString("dps");
    } else if (device == M) {
        SSD1306_OutString("mGs");
    } else {
        SSD1306_OutString(" ");
    }
    SSD1306_OutChar(CR);
    SSD1306_OutChar(CR);
    SSD1306_OutString("Z Axis: ");
    SSD1306_OutSFix1(z);
    if (device == A) {
        SSD1306_OutString("G");
    } else if (device == G) {
        SSD1306_OutString("dps");
    } else if (device == M) {
        SSD1306_OutString("mGs");
    } else {
        SSD1306_OutString(" ");
    }
}

void getAccelData(){
    x = LSM9DS1_XA();
    y = LSM9DS1_YA();
    z = LSM9DS1_ZA();
}

void getGyroData(){
    x = LSM9DS1_XG();
    y = LSM9DS1_YG();
    z = LSM9DS1_ZG();
}

void getMagData(){
    x = LSM9DS1_XM();
    y = LSM9DS1_YM();
    z = LSM9DS1_ZM();
}

void getTempData(){
    x = LSM9DS1_TMP();
}

void PBInt_Init(void){
    P1DIR  &= ~BIT5;  //put a 0 on P1.4 and 1.1 to make them inputs
    P1REN  |=  BIT5;  // enable resistors on P1.4 and P1.1
    P1OUT  |=  BIT5;  // make it a pull-up resistor
    P1SEL0 &= ~BIT5;  //use P1.4 and P1.1 as GPIO
    P1SEL1 &= ~BIT5;

    P1IE  |= BIT5;     //enable interrupts on P1.4 and P1.1
    P1IES |= BIT5;     //make them falling edge interrupts
    NVIC -> ISER[1] |= 0x008;      //enable the port 1 interrupt
    P1IFG &= ~BIT5;    //clear the flags in case there is a random button push
    EnableInterrupts();//global interrupt enable
    return;
}

void PORT1_IRQHandler(void){
    wasInterrupt = true;   // relay interrupt
    P1IFG &= 0x00;      // clear interrupt flags
}

// main
void main(void){
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer
	Clock_Init48MHz();

	// init PB
	DisableInterrupts();
	PBInt_Init();

	// init OLED
	SSD1306_Init(SSD1306_SWITCHCAPVCC); // 3.3V power
	SSD1306_ClearBuffer();
	SSD1306_DisplayBuffer();
	Clock_Delay1ms(500);

	// LSM9DS1 init
	LSM9DS1_Init();

    // init buzzer
    piezo_init();

    enum states {ACCELEROMETER, GYROSCOPE, MAGNETOMETER, THERMOMETER} state, prevState;
    state = ACCELEROMETER;             // start state
    prevState = GYROSCOPE;  // used to know when the state has changed.
    bool isNewState;            // true when the state has switched

	// basically copy and paste the following 3 lines for each state
//	SSD1306_SetCursor(0,0);
//	SSD1306_OutString("Accelerometer");
//	displayData(1235, 8735, 3345);


    while(1){
        isNewState = (state != prevState);
        prevState = state;  // save state for next time

        switch (state) {
        case ACCELEROMETER:
            // entry housekeeping
            if (isNewState) {
                SSD1306_ClearBuffer();
                SSD1306_DisplayBuffer();
                SSD1306_SetCursor(0,0);
                SSD1306_OutString("Accelerometer");
                play_note(HG);
            }
            // case housekeeping
            getAccelData();
            displayData((x / 100), (y / 100), (z / 100), A);   // turn MG to G with 1 decimal place
            // exit housekeeping
            if (wasInterrupt) {
                wasInterrupt = false;
                state = GYROSCOPE;
            }
            break;
        case GYROSCOPE:
            // entry housekeeping
            if (isNewState) {
                SSD1306_ClearBuffer();
                SSD1306_DisplayBuffer();
                SSD1306_SetCursor(0,0);
                SSD1306_OutString("Gyroscope");
                play_note(HG);
            }
            // case housekeeping
            getGyroData();
            displayData((x / 100), (y / 100), (z / 100), G);   // turn mdp to dps with 1 decimal place
            // exit housekeeping
            if (wasInterrupt) {
                wasInterrupt = false;
                state = MAGNETOMETER;
            }
            break;
        case MAGNETOMETER:
            // entry housekeeping
            if (isNewState) {
                SSD1306_ClearBuffer();
                SSD1306_DisplayBuffer();
                SSD1306_SetCursor(0,0);
                SSD1306_OutString("Magnetometer");
                play_note(HG);
            }
            // case housekeeping
            getMagData();
            displayData((x * 10), (y * 10), (z * 10), M);   // keep in mGuass with 1 decimal place
            // exit housekeeping
            if (wasInterrupt) {
                wasInterrupt = false;
                state = THERMOMETER;
            }
            break;
        case THERMOMETER:
            // entry housekeeping
            if (isNewState) {
                SSD1306_ClearBuffer();
                SSD1306_DisplayBuffer();
                SSD1306_SetCursor(0,0);
                SSD1306_OutString("Thermometer");
                play_note(HG);
            }
            // case housekeeping
            x = LSM9DS1_TMP();
            SSD1306_SetCursor(0,2);
            SSD1306_OutString("Temperature: ");
            SSD1306_OutSFix1(x);    // already in C with 1 decimal place
            // exit housekeeping
            if (wasInterrupt) {
                wasInterrupt = false;
                state = ACCELEROMETER;
            }
            break;
        default:
            state = ACCELEROMETER;
            break;
        }
        Clock_Delay1ms(100);    // "operate" at ~10Hz
    }
}
