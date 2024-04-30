#include <stdint.h>
#include <math.h>
#include "msp.h"
#include "LSM9DS1.h"

// available eUSCI_B1 pins (all primary pin function, so SEL must be changed):
// P6.2 - UCB1STE      bottom of board      slave select
// P6.3 - UCB1CLK      bottom of board      serial clock
// P6.4 - UCB1SIMO     left side of board   master out slave in
// P6.5 - UCB1SOMI     left side of board   master in slave out

// GPIO P6.0 and P6.1 are used for the slave select of the two subchips
// 6.0 - CSAG
// 6.1 - CSM

// peripheral pin definitions (all level shifted so you can use 3-5V logic in and has 3V logic out):
// SCL      - SPI clock pin.                            SPC in the datasheet.
// SDA      - SPI MOSI pin.                             SDI/SDO in the datasheet.
// CSAG     - Accelerometer+Gyro subchip Chip Select.   CS_A/G in the datasheet.
// CSM      - Magnetometer subchip Select.              CS_M in the datasheet. 
// SDOAG    - Accelerometer+Gyro subchip MISO pin       SDO_A/G in the datasheet.
// SDOM/DOM - Magnetometer subchip MISO pin             SDO_M in the datasheet.

// use standard GPI for slave select, and have UCB1STE just reflect an OR of the two slave selects

// LSM9DS1 AG (accelerometer and gyroscope) registers (https://cdn-learn.adafruit.com/assets/assets/000/038/883/original/LSM9DS1.pdf)
#define ACT_THS             0x04   // W  - Activity threshold
#define ACT_DUR             0x05   // W  - Inactivity duration
#define INT_GEN_CFG_XL      0x06   // W  - Accelerometer interrupt generator configuration. See datasheet page 42
#define INT_GEN_THS_X_XL    0x07   // W  - Accelerometer X interrupt threshold
#define INT_GEN_THS_Y_XL    0x08   // W  - Accelerometer Y interrupt threshold
#define INT_GEN_THS_Z_XL    0x09   // W  - Accelerometer Z interrupt threshold
#define INT_GEN_DUR_XL      0x0A   // W  - Accelerometer interrupt duration
#define REFERENCE_G         0x0B   // RW - Gyroscope reference value for interrupt generation
#define INT1_CTRL           0x0C   // W  - INT1 pin control. See datasheet page 44
#define INT2_CTRL           0x0D   // W  - INT2 pin control. See datasheet page 44
#define WHO_AM_I            0x0F   // R  - Device identification. Returns 0x68
#define CTRL_REG1_G         0x10   // W  - Gyroscope control register 1. G output rate/enable, full scale selection, and bandwidth selection. See datasheet page 45-46
#define CTRL_REG2_G         0x11   // W  - Gyroscope control register 2. Output and interupt selection. See datasheet page 47
#define CTRL_REG3_G         0x12   // W  - Gyroscope control register 3. Low power mode and high pass filter. See datasheet page 47-48
#define ORIENT_CFG_G        0x13   // RW - Gyroscope sign and orientation. See datasheet page 48
#define INT_GEN_SRC_G       0x14   // R  - Gyroscope interrupt source. See datasheet page 49
#define OUT_TEMP_L          0x15   // R  - Temperature output low byte. Two's complement, right-justified.
#define OUT_TEMP_H          0x16   // R  - Temperature output high byte. Two's complement, right-justified.
#define STATUS_REG_0        0x17   // R  - Status register. Interupt info and new data avail. See datasheet page 50
// If CTRL_REG8 (22h)(IF_ADD_INC) bit is ‘1’, then the address is incremented by 1, allowing XXh and (XX+1)h to be read in a single read operation of XXh.
#define OUT_X_L_G           0x18   // R  - Gyroscope X output low byte. Two's complement, right-justified. 
#define OUT_X_H_G           0x19   // R  - Gyroscope X output high byte. Two's complement, right-justified. 
#define OUT_Y_L_G           0x1A   // R  - Gyroscope Y output low byte. Two's complement, right-justified.
#define OUT_Y_H_G           0x1B   // R  - Gyroscope Y output high byte. Two's complement, right-justified.
#define OUT_Z_L_G           0x1C   // R  - Gyroscope Z output low byte. Two's complement, right-justified.
#define OUT_Z_H_G           0x1D   // R  - Gyroscope Z output high byte. Two's complement, right-justified.
#define CTRL_REG4           0x1E   // W  - Control register 4. Gyroscope output enabled. See datasheet page 51
#define CTRL_REG5_XL        0x1F   // W  - Accelerometer control register 5. Accelerometer output enable, and decimination. See datasheet page 51
#define CTRL_REG6_XL        0x20   // W  - Accelerometer control register 6. Accelerometer output rate, full scale selection, and bandwidth selection. See datasheet page 52
#define CTRL_REG7_XL        0x21   // W  - Accelerometer control register 7. Accelerometer high resolution mode, and filter stuff. See datasheet page 53
#define CTRL_REG8           0x22   // W  - Control register 8. Address auto increment, software reset. See datasheet page 53
#define CTRL_REG9           0x23   // W  - Control register 9. Gyro sleep, FIFO stuff, I2C disable. See datasheet page 54
#define CTRL_REG10          0x24   // W  - Control register 10. Accel and gyro self test. See datasheet page 54
#define INT_GEN_SRC_XL      0x26   // R  - Accelerometer interrupt source. See datasheet page 54-55
#define STATUS_REG          0x27   // R  - Status register. Interupt info and new data avail. See datasheet page 55
// If CTRL_REG8 (22h)(IF_ADD_INC) bit is ‘1’, then the address is incremented by 1, allowing XXh and (XX+1)h to be read in a single read operation of XXh.
#define OUT_X_L_XL          0x28   // R  - Accelerometer X output low byte. Two's complement, right-justified.
#define OUT_X_H_XL          0x29   // R  - Accelerometer X output high byte. Two's complement, right-justified.
#define OUT_Y_L_XL          0x2A   // R  - Accelerometer Y output low byte. Two's complement, right-justified.
#define OUT_Y_H_XL          0x2B   // R  - Accelerometer Y output high byte. Two's complement, right-justified.
#define OUT_Z_L_XL          0x2C   // R  - Accelerometer Z output low byte. Two's complement, right-justified.
#define OUT_Z_H_XL          0x2D   // R  - Accelerometer Z output high byte. Two's complement, right-justified.
#define FIFO_CTRL           0x2E   // W  - FIFO control. See datasheet page 56
#define FIFO_SRC            0x2F   // R  - FIFO status. Filled? threshold? Number of unread samples. See datasheet page 57
#define INT_GEN_CFG_G       0x30   // W  - Gyroscope interrupt generator configuration. See datasheet page 58
#define INT_GEN_THS_XH_G    0x31   // W  - Gyroscope X interrupt threshold high byte
#define INT_GEN_THS_XL_G    0x32   // W  - Gyroscope X interrupt threshold low byte
#define INT_GEN_THS_YH_G    0x33   // W  - Gyroscope Y interrupt threshold high byte
#define INT_GEN_THS_YL_G    0x34   // W  - Gyroscope Y interrupt threshold low byte
#define INT_GEN_THS_ZH_G    0x35   // W  - Gyroscope Z interrupt threshold high byte
#define INT_GEN_THS_ZL_G    0x36   // W  - Gyroscope Z interrupt threshold low byte
#define INT_GEN_DUR_G       0x37   // W  - Gyroscope interrupt duration. See datasheet page 59-61

// LSM9DS1 M (magnetometer) registers (https://cdn-learn.adafruit.com/assets/assets/000/038/883/original/LSM9DS1.pdf)
#define OFFSET_X_REG_L_M    0x05   // RW - Magnetometer X offset low byte. Used to conteract environmental effects.
#define OFFSET_X_REG_H_M    0x06   // RW - Magnetometer X offset high byte. Used to conteract environmental effects.
#define OFFSET_Y_REG_L_M    0x07   // RW - Magnetometer Y offset low byte. Used to conteract environmental effects.
#define OFFSET_Y_REG_H_M    0x08   // RW - Magnetometer Y offset high byte. Used to conteract environmental effects.
#define OFFSET_Z_REG_L_M    0x09   // RW - Magnetometer Z offset low byte. Used to conteract environmental effects.
#define OFFSET_Z_REG_H_M    0x0A   // RW - Magnetometer Z offset high byte. Used to conteract environmental effects.
#define WHO_AM_I_M          0x0F   // R  - Device identification. Returns 0x3D
#define CTRL_REG1_M         0x20   // W  - Magnetometer control register 1. Temp compensation, output rate, and power mode, self test. See datasheet page 63
#define CTRL_REG2_M         0x21   // W  - Magnetometer control register 2. Full scale selection, reboot, and soft reset. See datasheet page 64
#define CTRL_REG3_M         0x22   // W  - Magnetometer control register 3. Operating mode, I2C disable, SPI read disable, and low power mode. See datasheet page 64
#define CTRL_REG4_M         0x23   // W  - Magnetometer control register 4. Z-axis mode selection. See datasheet page 65
#define CTRL_REG5_M         0x24   // W  - Magnetometer control register 5. Fast read and block data update. See datasheet page 65
#define STATUS_REG_M        0x27   // R  - Status register. Interupt info and new data avail. See datasheet page 66
#define OUT_X_L_M           0x28   // R  - Magnetometer X output low byte. Two's complement, right-justified.
#define OUT_X_H_M           0x29   // R  - Magnetometer X output high byte. Two's complement, right-justified.
#define OUT_Y_L_M           0x2A   // R  - Magnetometer Y output low byte. Two's complement, right-justified.
#define OUT_Y_H_M           0x2B   // R  - Magnetometer Y output high byte. Two's complement, right-justified.
#define OUT_Z_L_M           0x2C   // R  - Magnetometer Z output low byte. Two's complement, right-justified.
#define OUT_Z_H_M           0x2D   // R  - Magnetometer Z output high byte. Two's complement, right-justified.
#define INT_CFG_M           0x30   // W  - Magnetometer interrupt configuration. See datasheet page 67
#define INT_SRC_M           0x31   // R  - Magnetometer interrupt source. See datasheet page 67
#define INT_THS_L           0x32   // W  - Magnetometer interrupt threshold low byte. See datasheet page 68
#define INT_THS_H           0x33   // W  - Magnetometer interrupt threshold high byte. See datasheet page 68

uint16_t RDxBuffer = 0;

void LSM9DS1_Init(void){

    // pin init
    P6SEL0 |= (BIT2 | BIT3 | BIT4 | BIT5); // configure P9.2-P9.5 as primary module function
    P6DIR |= (BIT0 | BIT1 | BIT2); // make P9.0 and P9.1 outputs for the two SS pins
    P6OUT |= (BIT0 | BIT1 | BIT2); // deassert both CS pins

    // UCB1 init
    EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_SWRST; // Put state machine in reset
    EUSCI_B1->CTLW0 = EUSCI_B_CTLW0_SWRST | // Remain in reset state
                        EUSCI_B_CTLW0_MST | // SPI master
                       EUSCI_B_CTLW0_SYNC | // Synchronous mode
                        EUSCI_B_CTLW0_MSB | // MSB first
                     EUSCI_B_CTLW0_MODE_1 | // 4-pin mode
                       EUSCI_B_CTLW0_STEM | // STE mode select
                       EUSCI_B_CTLW0_CKPH | // clock phase select. fuck yeah, im still unsure what this does, but it made it work :)
                  EUSCI_B_CTLW0_SSEL__ACLK; // ACLK
    EUSCI_B1->BRW = 1;  //fBitClock = fBRCLK/(UCBRx+1)
    EUSCI_B1->CTLW0 &= ~EUSCI_B_CTLW0_SWRST; // Enable eUSCI_B SPI
    
    // LSM init
    write_SPI(AG, CTRL_REG1_G, 0x20);   // enable gyro  @ 14.9hz
    write_SPI(AG, CTRL_REG6_XL, 0x40);  // enable accel @ 50hz
    write_SPI(AG, CTRL_REG5_XL, 0x38);  // enable accel output, was supposed to default to this, but didnt
    write_SPI(M, CTRL_REG3_M, 0x00);    // disable i2c, enable spi write operations, continuous conversion
    write_SPI(M, CTRL_REG1_M, 0x74);    // XY axis ultra-high performance mode (since this is wired so battery life isnt a concern) and 20Hz output rate
    write_SPI(M, CTRL_REG4_M, 0x0C);    // Z axis ultra-high performance mode (for the same reason)
};

uint8_t SPI_transfer(uint8_t data){

    // begin transmission
    EUSCI_B1->TXBUF = data;

    // wait while eUSCI_B is busy
    while(EUSCI_B1->STATW & EUSCI_B_STATW_SPI_BUSY){}

    // read recieved value
    return EUSCI_B1->RXBUF;
}

// if bytesToRead is 2, will read 2 bytes, otherwise it will read 1
void read_SPI(enum ss_t device, uint8_t address, uint8_t bytesToRead){
    if (device == AG) {
        P6OUT &= (~BIT0 & ~BIT2); // assert CSAG
        SPI_transfer(address | 0x80);
    } else if (device == M) {
        P6OUT &= (~BIT1 & ~BIT2); // assert CSM
        SPI_transfer(address | 0xC0);
    }

    RDxBuffer = (0x0000 | SPI_transfer(0));     // read first byte and put into global var

    if (bytesToRead == 2){
        RDxBuffer = (SPI_transfer(0) << 8) | RDxBuffer; // read second byte and put into global var
    }

    if (device == AG) {
        P6OUT |= (BIT0 | BIT2); // deassert CSAG
    } else if (device == M) {
        P6OUT |= (BIT1 | BIT2); // deassert CSM
    }
}

void write_SPI(enum ss_t device, uint8_t address, uint8_t data){
    if (device == AG) {
        P6OUT &= (~BIT0 & ~BIT2); // assert CSAG
        SPI_transfer(address);
    } else if (device == M) {
        P6OUT &= (~BIT1 & ~BIT2); // assert CSM
        SPI_transfer(address);
    }

    SPI_transfer(data);

    if (device == AG) {
        P6OUT |= (BIT0 | BIT2); // deassert CSAG
    } else if (device == M) {
        P6OUT |= (BIT1 | BIT2); // deassert CSM
    }
}



uint16_t LSM9DS1_WHO_AM_I(enum ss_t device){
    read_SPI(device, WHO_AM_I, 1);
    return RDxBuffer;
};

// used during testing/debugging
uint16_t LSM9DS1_TEST_CMD(void){
    read_SPI(AG, CTRL_REG5_XL, 1);
    return RDxBuffer;
};

// FIXME: find out how to convert this to a usable value

int16_t LSM9DS1_XA(void){
    read_SPI(AG, OUT_X_L_XL, 2);
    // 0.061 mg per LSB (for the mode i specified). see datasheet page 12
    int16_t accelX = (int16_t)RDxBuffer;
    accelX = (accelX * 61) / 1000;
    return accelX;
};

int16_t LSM9DS1_YA(void){
    read_SPI(AG, OUT_Y_L_XL, 2);
    // 0.061 mg per LSB (for the mode i specified). see datasheet page 12
    int16_t accelY = (int16_t)RDxBuffer;
    accelY = (accelY * 61) / 1000;
    return accelY;
};

int16_t LSM9DS1_ZA(void){
    read_SPI(AG, OUT_Z_L_XL, 2);
    // 0.061 mg per LSB (for the mode i specified). see datasheet page 12
    int16_t accelZ = (int16_t)RDxBuffer;
    accelZ = (accelZ * 61) / 1000;
    return accelZ;
};

int16_t LSM9DS1_XG(void){
    read_SPI(AG, OUT_X_L_G, 2);
    // 8.75 mdps per LSB (for the mode i specified). see datasheet page 12
    int16_t gyroX = (int16_t)RDxBuffer;
    gyroX = (gyroX * 875) / 100;
    return gyroX;
};

int16_t LSM9DS1_YG(void){
    read_SPI(AG, OUT_Y_L_G, 2);
    // 8.75 mdps per LSB (for the mode i specified). see datasheet page 12
    int16_t gyroY = (int16_t)RDxBuffer;
    gyroY = (gyroY * 875) / 100;
    return gyroY;
};

int16_t LSM9DS1_ZG(void){
    read_SPI(AG, OUT_Z_L_G, 2);
    // 8.75 mdps per LSB (for the mode i specified). see datasheet page 12
    int16_t gyroZ = (int16_t)RDxBuffer;
    gyroZ = (gyroZ * 875) / 100;
    return gyroZ;
};

int16_t LSM9DS1_XM(void){
    read_SPI(M, OUT_X_L_M, 2);
    // 0.14 mguass per LSB (for the mode i specified). see datasheet page 12
    int16_t magX = (int16_t)RDxBuffer;
    magX = (magX * 14) / 100;
    return magX;
};

int16_t LSM9DS1_YM(void){
    read_SPI(M, OUT_Y_L_M, 2);
    // 0.14 mguass per LSB (for the mode i specified). see datasheet page 12
    int16_t magY = (int16_t)RDxBuffer;
    magY = (magY * 14) / 100;
    return magY;
};

int16_t LSM9DS1_ZM(void){
    read_SPI(M, OUT_Z_L_M, 2);
    // 0.14 mguass per LSB (for the mode i specified). see datasheet page 12
    int16_t magZ = (int16_t)RDxBuffer;
    magZ = (magZ * 14) / 100;
    return magZ;    
};

// Temperature sensor output data. 
// The value is expressed as two’s complement sign extended on the MSB
int16_t LSM9DS1_TMP(void){
    read_SPI(AG, OUT_TEMP_L, 2);
    // 16 LSB/°C. see datasheet page 14
    int16_t temp = (int16_t)RDxBuffer;
    temp = (temp * 10) / 16; // so 1 decimal place is preserved
    return temp;
};
