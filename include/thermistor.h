#pragma once
#include <stdint.h>

#ifndef PROGMEM
#define PROGMEM
#endif
#ifndef pgm_read_word
#define pgm_read_word(addr) (*(const short *)(addr))
#endif

float computeTemp(int rawADC);

extern const short temptable_100K[][2];
