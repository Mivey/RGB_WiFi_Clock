/*
 * cheapLibation.h
 *
 *  Created on: May 19, 2021
 *      Author: marka
 *
 *      Math 'clone' of the FastLED Lib8tion.h, Math8.h, scale8.h, and trig8.h.
 *      Better suited for 32 bit ops, no real optimization done as I'm retarded.
 */

#ifndef INC_CHEAPLIBATION_H_
#define INC_CHEAPLIBATION_H_

#include "stm32f4xx_hal.h"

#define R_L 3
#define B_L 1
#define G_L 2
#define NUMPIXELS 60
#define PIXELARR 3
#define NINETYPERFILL 0xDF
#define FIFTYPERFILL 0x7F
#define TENPERFILL 0x1F
#define TWENTYFIVEPERFILL 0x3F

#endif /* INC_CHEAPLIBATION_H_ */


// Sine Look-Up Table
// One degree increments, from 0 to 179, based on formula [sin(2 * pi * (N / 127)) + 127]
// Look up angle with example code:

//angle = phase + OFFSET * 6
//if (((angle % 360) < 180) == 0)
//	x = sLUT[angle/180];
//else
//	x = 254 - sLUT[angle/180];
//const uint8_t sLUT[] = {	0x7F,  0x81,  0x83,  0x86,  0x88,  0x8A,  0x8C,  0x8E,  0x91,  0x93,  0x95,  0x97,  0x99,  0x9C,  0x9E,  0xA0,  0xA2,  0xA4,  0xA6,  0xA8,
//							0xAA,  0xAD,  0xAF,  0xB1,  0xB3,  0xB5,  0xB7,  0xB9,  0xBB,  0xBD,  0xBF,  0xC0,  0xC2,  0xC4,  0xC6,  0xC8,  0xCA,  0xCB,  0xCD,  0xCF,
//							0xD1,  0xD2,  0xD4,  0xD6,  0xD7,  0xD9,  0xDA,  0xDC,  0xDD,  0xDF,  0xE0,  0xE2,  0xE3,  0xE4,  0xE6,  0xE7,  0xE8,  0xEA,  0xEB,  0xEC,
//							0xED,  0xEE,  0xEF,  0xF0,  0xF1,  0xF2,  0xF3,  0xF4,  0xF5,  0xF6,  0xF6,  0xF7,  0xF8,  0xF8,  0xF9,  0xFA,  0xFA,  0xFB,  0xFB,  0xFC,
//							0xFC,  0xFC,  0xFD,  0xFD,  0xFD,  0xFE,  0xFE,  0xFE,  0xFE,  0xFE,  0xFE,  0xFE,  0xFE,  0xFE,  0xFE,  0xFE,  0xFD,  0xFD,  0xFD,  0xFC,
//							0xFC,  0xFC,  0xFB,  0xFB,  0xFA,  0xFA,  0xF9,  0xF8,  0xF8,  0xF7,  0xF6,  0xF6,  0xF5,  0xF4,  0xF3,  0xF2,  0xF1,  0xF0,  0xEF,  0xEE,
//							0xED,  0xEC,  0xEB,  0xEA,  0xE8,  0xE7,  0xE6,  0xE4,  0xE3,  0xE2,  0xE0,  0xDF,  0xDD,  0xDC,  0xDA,  0xD9,  0xD7,  0xD6,  0xD4,  0xD2,
//							0xD1,  0xCF,  0xCD,  0xCB,  0xCA,  0xC8,  0xC6,  0xC4,  0xC2,  0xC0,  0xBF,  0xBD,  0xBB,  0xB9,  0xB7,  0xB5,  0xB3,  0xB1,  0xAF,  0xAD,
//							0xAA,  0xA8,  0xA6,  0xA4,  0xA2,  0xA0,  0x9E,  0x9C,  0x99,  0x97,  0x95,  0x93,  0x91,  0x8E,  0x8C,  0x8A,  0x88,  0x86,  0x83,  0x81
//}

// 	Fade Filter Short, FFSHORT applies filter to first element no longer lit and multiples the next 6 each run.
//	Fade Filter Long, or FFLONG applies 89.5% reduction to every element every iteration.
extern const uint8_t FFSHORT[];
//const uint8_t FFSHORTR[] = {0x0F, 0x1F, 0x3F, 0x7F, 0xBF, 0xDF, 0xFF};
extern const uint8_t FFLONG[];

extern const uint8_t FFPEN[];

extern const uint8_t FFINIT[];

//need: all 0x00 filter, all 0xDF filter, all 0x0F filter, all 0x7F
// for No filter, various fade in, fade out at 90, 10 and 50%
//generic filter fill? just call fill your palette... use 0xFF000000, 0xDF, etc.

// longer filter

//These need to change, will be 8 bit instead of 32
//uint32_t fadeAllBy(uint32_t a, uint8_t howmuch);
//uint32_t fadeOneBy(uint32_t a, uint8_t howmuch, uint8_t b);
//uint32_t fadeTwoBy(uint32_t a, uint8_t howmuch, uint8_t b, uint8_t c);
void fadeToBlackBy(uint8_t arr[NUMPIXELS][PIXELARR], uint8_t dim_ptr, uint8_t filter[], int elements, uint32_t rng, uint8_t flags);
int unfadeToBlack(uint16_t target, uint8_t filter);
//void fillYourPalette(uint8_t arr[NUMPIXELS][PIXELARR], uint32_t val, uint32_t parr[NUMPIXELS]);
void pointfill(uint8_t arr[NUMPIXELS][PIXELARR],uint8_t arrfb[NUMPIXELS][PIXELARR], uint32_t pval, int index);
void composite(uint8_t carr[NUMPIXELS + 2][PIXELARR + 1], uint8_t parr[NUMPIXELS][PIXELARR],uint8_t farr[NUMPIXELS][PIXELARR],
		int offsetp, int offsetf, uint8_t brite);
void fillYourPalette(uint8_t arr[NUMPIXELS][PIXELARR], uint32_t parr[NUMPIXELS]);
void fillColor( uint8_t arr[NUMPIXELS][PIXELARR], uint32_t val);
void alaCartePalette (uint32_t cust[], uint8_t parr[NUMPIXELS][PIXELARR]);
void filterPointReset (uint8_t farr[NUMPIXELS][PIXELARR], int index);


