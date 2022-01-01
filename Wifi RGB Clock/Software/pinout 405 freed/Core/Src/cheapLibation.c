/*
 * cheapLibation.c
 *
 *  Created on: May 21, 2021
 *      Author: marka
 */




#include "cheapLibation.h"

// 	Fade Filter Short, FFSHORT applies filter to first element no longer lit and multiples the next 6 each run.
//	Fade Filter Long, or FFLONG applies 89.5% reduction to every element every iteration.
const uint8_t FFSHORT[] = {0xFF, 0xDF, 0xBF, 0x7F, 0x3F, 0x1F, 0x0F, 0x0F, 0x0F, 0x0F};
//const uint8_t FFSHORTR[] = {0x0F, 0x1F, 0x3F, 0x7F, 0xBF, 0xDF, 0xFF};
const uint8_t FFLONG[] = {	0xFF, 0xE6, 0xE6, 0xE6, 0xE6, 0xE6, 0xE6, 0xE6, 0xE6, 0xE6, 0xE6,
		0xE6, 0xE6, 0xE6, 0xE6, 0xE6, 0xE6, 0xE6, 0xE6, 0xE6, 0xE6,
		0xE6, 0xE6, 0xE6, 0xE6, 0xE6, 0xE6, 0xE6, 0xE6, 0xE6, 0xE6,
		0xE6, 0xE6, 0xE6, 0xE6, 0xE6, 0xE6, 0xE6, 0xE6, 0xE6, 0xE6,
		0xE6, 0xE6, 0xE6, 0xE6, 0xE6, 0xE6, 0xE6, 0xE6, 0xE6, 0xE6,
		0xE6, 0xE6, 0xE6, 0xE6, 0xE6, 0xE6, 0xE6, 0xE6, 0xFF } ;

const uint8_t FFINIT[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

const uint8_t FFPEN[] = { 0x3F, 0x3F, 0x3F, 0xFF, 0x3F, 0x3F, 0x3F };

//need: all 0x00 filter, all 0xDF filter, all 0x0F filter, all 0x7F
// for No filter, various fade in, fade out at 90, 10 and 50%
//generic filter fill? just call fill your palette... use 0xFF000000, 0xDF, etc.

// longer filter

/*
 * Definitions of FadeToBlackBy
 * flags: 	0xn1 - Blue
 * 			0xn2 - Green
 * 			0xn4 - Red
 * 			0x1n - FFLONG
 * 			0x2n - FFSHORT
 * 			0x4n - FFPEN
 * 			0x8n - forward or reverse (1 forward)
 *
 * dim_ptr: starting position of array
 * *arr : array
 * count: how many LED
 * rng adds randomness to first 10 pixels of fade filter, set to 0 to turn off
 *
 * First clear pixel mask before calling, by setting dim_ptr to 0xFF.
 * Or don't. If you want a fade out.
 *
 */
void fadeToBlackBy(uint8_t arr[NUMPIXELS][PIXELARR], uint8_t dim_ptr, uint8_t filter[], int elements, uint32_t rng, uint8_t flags)
{
	int w = abs(elements) - 1;
	// get the index 'sign' of palette
	int ps = w/elements;
	int p = (ps == 1 ? 0 : w);

	for(uint8_t j = 0; j < 3; j++)
	{
//		if((flags & (0x01<< j)) == 0)
		{
			//generate r/g/b rng flags
			uint64_t crng = (rng + (rng * j)) * rng;
			//clear data from point

//			for (int i = dim_ptr; i < ((dim_ptr + w)%60); i++)
			for (int i = w; i >= 0; i--)
			{
				if ((crng % 0x0F ) < 0x0C)
					arr[(dim_ptr - i + 60) % 60][j] = ((uint16_t) arr[(dim_ptr - i + 60) % 60][j] *  filter[i]) >> 8;
				crng >>= 4;
				p = p - ps;
			}
			// reset value of p
			p = (ps == 1 ? 0 : w);

		}
	}
}

int unfadeToBlack(uint16_t target, uint8_t filter)
{
	return ( ((target * filter) >> 8) + target);

}

void fillYourPalette(uint8_t arr[NUMPIXELS][PIXELARR], uint32_t parr[NUMPIXELS])
{
	//use online defined palettes to generate pixels at runtime..?
	for(uint8_t i = 0; i < (NUMPIXELS); i++)
	{
		uint32_t x = parr[i];
		for(uint8_t j = 0; j < (PIXELARR); j++)
		{
			arr[i][j] = (0xFF & x);
			x >>= 8;
		}
	}
}

void alaCartePalette (uint32_t cust[], uint8_t parr[NUMPIXELS][PIXELARR])
{
	for ( int i = 1; i < 16; i++)
	{
		uint32_t tempa = cust[i-1];
		uint32_t tempb = cust[i];
		for (int j = 0; j< PIXELARR; j++)
		{
			//grab last byte from two colors
			uint8_t x = (tempa & 0xFF);
			uint8_t y = (tempb & 0xFF);

			if ( tempa > tempb)
			{
				uint8_t diffa = (tempa - tempb) / 4 ;
				uint8_t diffb = (tempa - tempb) * 2 / 4;
				uint8_t diffc = (tempa - tempb) * 3 / 4;
				parr[(i - 1) * 4 + 0][j] = tempa - diffa;
				parr[(i - 1) * 4 + 1][j] = tempa - diffb;
				parr[(i - 1) * 4 + 2][j] = tempa - diffc;
				parr[(i - 1) * 4 + 3][j] = tempb;
			}
			else if(tempa < tempb)
			{
				uint8_t diffa = (tempb - tempa) / 4 ;
				uint8_t diffb = (tempb - tempa) * 2 / 4;
				uint8_t diffc = (tempb - tempa) * 3 / 4;
				parr[(i - 1) * 4 + 0][j] = tempa + diffa;
				parr[(i - 1) * 4 + 1][j] = tempa + diffb;
				parr[(i - 1) * 4 + 2][j] = tempa + diffc;
				parr[(i - 1) * 4 + 3][j] = tempb;
			}
			else
			{
				parr[(i - 1) * 4 + 0][j] = tempa;
				parr[(i - 1) * 4 + 1][j] = tempa;
				parr[(i - 1) * 4 + 2][j] = tempb;
				parr[(i - 1) * 4 + 3][j] = tempb;
			}

			tempa >>=8;
			tempb >>= 8;
		}
	}

}

void fillColor( uint8_t arr[NUMPIXELS][PIXELARR], uint32_t val)
{
	for(uint8_t i = 0; i < (NUMPIXELS); i++)
	{
		//store value passed in function
		uint32_t x = val ;

		for(uint8_t j = 0; j < (PIXELARR); j++)
		{
			//place last 8 bits in array. then remove placed bits.
			arr[i][j] = (0xFF & x);
			x >>= 8;
		}
	}
}
void filterPointReset (uint8_t farr[NUMPIXELS][PIXELARR], int index)
{
	index %= 60;
	for (int j = 0; j<PIXELARR; j++)
		farr[index][j] = 0xFF;
}
void pointfill(uint8_t arr[NUMPIXELS][PIXELARR],uint8_t arrfb[NUMPIXELS][PIXELARR],
		uint32_t pval, int index)
{
	index %= 60;
	uint8_t temp = 0;
	for(uint8_t i = 0; i < PIXELARR ; i++)
	{
		temp = ( (pval & 0xff));
		arrfb[index][i] = 0xFF;
		arr[index][i] = temp; // place value of pixel here
		pval >>=8;

	}
}

void composite(uint8_t carr[NUMPIXELS + 2][PIXELARR + 1], uint8_t parr[NUMPIXELS][PIXELARR],uint8_t farr[NUMPIXELS][PIXELARR],
		int offsetp, int offsetf, uint8_t brite)
{
	// palette array offset index
	int p = abs(offsetp);
	// get the index 'sign' of palette
	int ps = p/offsetp;
	// filter array offset index
	int f = abs(offsetf);
	int fs = f/offsetf;
	p %= 60;
	f %= 60;

	//Fill carr[0][n] with 0x00
	carr[0][0] = 0x00;
	carr[0][1] = 0x00;
	carr[0][2] = 0x00;
	carr[0][3] = 0x00;

//	if (!(brite && 0xE0))
//	{
//		if ((brite & 0xE0) != 0)
//			brite>>=3;
//		brite |= 0xE0;
//	}

	for (int i = 1; i < (NUMPIXELS + 1); i++)
	{
		carr[i][0] = brite;
		for (int j = 0; j < (PIXELARR); j++)
			carr[i][j + 1] = (uint16_t) ((parr[p][j] * farr[f][j]) >> 8);

		// if offset is negative, then we need to go backwards through the array.
		p = ((p + ps + 60) % 60 );
		f = ((f + fs + 60) % 60 );
	}

	// not really needed (for HD107s), but the end frame.
	carr[61][0] = 0xFF;
	carr[61][1] = 0xFF;
	carr[61][2] = 0xFF;
	carr[61][3] = 0xFF;
}

///*
// * HSV psudo code
// */
//
//uint8_t conversionfactor = 182;
//uint16_t degMappedToHex = 0xFFF0; // conversionfactor * 360
//uint16_t hue;
//uint16_t sat;
//uint16_t val;
//
//uint16_t hues;// etc
//
//
//
//uint16_t hued = (hue > hues ? (hue - hues) : (hues - hue));
//
//uint16_t chroma = (sat * val)>> 8;
//uint16_t m = val - chroma;
//
////map hue to 360 * 182
////check if Hue is between 0-120, 120 - 240, or 240 to 360
//if ( (hue <= 0x2AA8) || (hue > 0xD548))
//{
//	//Since we know where hue is, assign to r, g or b now
//	r = (chroma + m) >> 8;
//	if (hue <= 0x2AA8)
//	{
//		//so was it more or less than the half way point of 60?
//		// the (1 - abs(H' % 2 - 1)) on Wiki's website is just a sawtooth function.
//		//That's why it's split into two here, removing the % 2
//		// The two lines making up the sawtooh function work out to be
//		// 1) Hue
//		// 2) 2 - Hue
//		// The 6 gets us to 360 some how... (we never divided by 60, as the formula dictated in wiki)
//		// So that's why it's just chroma * hue * 6
//		// The 16 bit value is treated as a big old floating point. Right shift by 16 to get MSB's
//		uint16_t x = (chroma * hue * 6)>>16;
//		// we can calculate the last two values.
//		g = (m + x)>>8;
//		b = m>>8;
//	}
//	else
//	{
//		//Same as above, but we use 0x1FFFF which is the floating point for 1.99999
//		// ... or 2. some inaccuracies here, but we only care about the first 8 bits (of 32), so...
//		uint16_t x = (chroma * (0x1FFFF - hue * 6))>>16;
//		g = m>>8;
//		b = (m + x)>>8;
//	}
//}
////two more if statements
//else
//{
//	//unknown, set all to 0
//	r = 0; g=0; b=0;
//}


