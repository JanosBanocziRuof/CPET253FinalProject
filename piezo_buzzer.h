#include <stdint.h>

#define LLF  15306
#define LLFS 14446
#define LG  13636
#define LGS 12870
#define LA  12148
#define LBF 11466
#define LB  10822
#define LC  10214
#define LCS 9642
#define LD  9100
#define LDS 8590
#define LE  8108
#define LF  7652
#define LFS 7222
#define MG  6818
#define MGS 6434
#define MA  6074
#define MBF 5732
#define MB  5410
#define MC  5106
#define MCS 4820
#define MD  4550
#define MDS 4294
#define ME  4054
#define MF  3826
#define MFS 3610
#define HG  3408

void piezo_init(void);
void play_note(volatile uint16_t note);