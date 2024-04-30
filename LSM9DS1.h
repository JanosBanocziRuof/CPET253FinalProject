#include <stdint.h>

#define true 1
#define false 0

enum ss_t {AG, M, A, G};

void LSM9DS1_Init(void);

uint16_t LSM9DS1_WHO_AM_I(enum ss_t device);
uint16_t LSM9DS1_TEST_CMD();

int16_t LSM9DS1_XA(void);
int16_t LSM9DS1_YA(void);
int16_t LSM9DS1_ZA(void);

int16_t LSM9DS1_XG(void);
int16_t LSM9DS1_YG(void);
int16_t LSM9DS1_ZG(void);

int16_t LSM9DS1_XM(void);
int16_t LSM9DS1_YM(void);
int16_t LSM9DS1_ZM(void);

int16_t LSM9DS1_TMP(void);
