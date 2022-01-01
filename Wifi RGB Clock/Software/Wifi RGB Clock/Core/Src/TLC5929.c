/*
 * TLC5929.c
 *
 *  Created on: Feb 24, 2021
 *      Author: marka
 */


#include "TLC5929.h"

/*
 *
 * INITIALIZATION
 *
 */

void TLC5929_Init(TLC5929_HandleTypeDef * shift_reg,
											TIM_HandleTypeDef *PWM_Channel, uint32_t Clock,
											GPIO_TypeDef *LatchBank, uint16_t LatchPin,
											GPIO_TypeDef *BlankBank, uint16_t BlankPin,
											GPIO_TypeDef *Data_inBank, uint16_t Data_inPin,
											uint8_t Shift_Reg_Loc){
	shift_reg->PWM_Channel		= PWM_Channel;
	shift_reg->Clock			= Clock;
	shift_reg->LatchBank		= LatchBank;
	shift_reg->LatchPin			= LatchPin;
	shift_reg->Data_inBank		= Data_inBank;
	shift_reg->Data_inPin		= Data_inPin;
	shift_reg->BlankBank		= BlankBank;
	shift_reg->BlankPin			= BlankPin;
	shift_reg->Shift_Reg_Loc	= Shift_Reg_Loc;


	// Set flags (init flag, Set all blank)
	shift_reg->TLC_Reg		= 0x01U;

	// 5929 Ctrl data
	//shift_reg->data 		= TLC_REG;
	shift_reg->data			= 0x00;	// set all LEDs to off

	//Blank set High by Default, Latch low

	HAL_GPIO_WritePin(shift_reg->LatchBank, shift_reg->LatchPin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(shift_reg->Data_inBank, shift_reg->Data_inPin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(shift_reg->BlankBank, shift_reg->BlankPin, GPIO_PIN_RESET);// reset = ON

}
/* These two are for shifting data around the ring. So LED's go around in a circle.
 * For show bullshit
 * no need for curr/next frame
 * temp frame, and cd.data - will use cd.data to create contents of tempframe
 * mask off Hours and save to tempframe
 * save temp frame to cd.data.
 */
void TLC5929_ArrayShiftMin(uint16_t x[] , uint8_t flag)
{
	uint16_t temp[6] = {0,0,0,0,0,0};
//	uint16_t *y = &x
	if (flag == 0)
	{
		for (int i = 0; i < 6; i++)
		{
//			temp[i] = (x[i].data & 0x7676U) << 1;
//			temp[i] |= ((x[i].data & 0x0404U) << 2);
//			temp[i] |= ((x[i].data & 0x0040U) << 3);
//			temp[i] |= ((x[(i + 5) % 6].data & 0x4000U) >> 14);
//			temp[i] |= ((x[i].data & 0x0808U));
			temp[i] = (x[i] & 0x7676U) << 1;
			temp[i] |= ((x[i] & 0x0404U) << 2);
			temp[i] |= ((x[i] & 0x0040U) << 3);
			temp[i] |= ((x[(i + 5) % 6] & 0x4000U) >> 14);
			temp[i] |= ((x[i] & 0x0808U));
		}

		 // move to for loop
//		for (uint8_t t = 0; t< 6; t++)
//		{
//			if ((*CurrFrame[t] && MIN_0) != 0)
//				*NextFrame[t] |= MIN_1;
//			if ((*CurrFrame[t] && MIN_1) != 0)
//				*NextFrame[t] |= MIN_2;
//			if ((*CurrFrame[t] && MIN_2) != 0)
//				*NextFrame[t] |= MIN_3;
//			if ((*CurrFrame[t] && MIN_3) != 0)
//				*NextFrame[t] |= MIN_4;
//			if ((*CurrFrame[t] && MIN_4) != 0)
//				*NextFrame[t] |= MIN_5;
//			if ((*CurrFrame[t] && MIN_5) != 0)
//				*NextFrame[t] |= MIN_6;
//			if ((*CurrFrame[t] && MIN_6) != 0)
//				*NextFrame[t] |= MIN_7;
//			if ((*CurrFrame[t] && MIN_7) != 0)
//				*NextFrame[t] |= MIN_8;
//			if ((*CurrFrame[t] && MIN_8) != 0)
//				*NextFrame[t] |= MIN_9;
//			if (((*CurrFrame[t] && MIN_9) != 0) && (t != 5))
//				*NextFrame[t+1] |= MIN_0;
//		}
//		if ((*CurrFrame[5] && MIN_9) != 0)
//			*NextFrame[0] |= MIN_0;
	}
//	else if (flag == 1)
//	{
//		for (int i = 0; i < 6; i++)
//		{
//			temp[i] = (x[i].data & 0x7676U) >> 1;
//			temp[i] |= ((x[i].data & 0x1010U) >> 2);
//			temp[i] |= ((x[i].data & 0x0200U) >> 3);
//			temp[i] |= ((x[(i + 1) % 6].data & 0x0002U) << 14);
//			temp[i] |= ((x[i].data & 0x0808U));
//		}

//		for (uint8_t t = 0; t< 6; t++)
//		{
//
//			if (((*CurrFrame[t] && MIN_0) != 0) && (t != 0))
//				*NextFrame[t-1] |= MIN_9;
//			if ((*CurrFrame[t] && MIN_1) != 0)
//				*NextFrame[t] |= MIN_0;
//			if ((*CurrFrame[t] && MIN_2) != 0)
//				*NextFrame[t] |= MIN_1;
//			if ((*CurrFrame[t] && MIN_3) != 0)
//				*NextFrame[t] |= MIN_2;
//			if ((*CurrFrame[t] && MIN_4) != 0)
//				*NextFrame[t] |= MIN_3;
//			if ((*CurrFrame[t] && MIN_5) != 0)
//				*NextFrame[t] |= MIN_4;
//			if ((*CurrFrame[t] && MIN_6) != 0)
//				*NextFrame[t] |= MIN_5;
//			if ((*CurrFrame[t] && MIN_7) != 0)
//				*NextFrame[t] |= MIN_6;
//			if ((*CurrFrame[t] && MIN_8) != 0)
//				*NextFrame[t] |= MIN_7;
//			if ((*CurrFrame[t] && MIN_9) != 0)
//				*NextFrame[t] |= MIN_8;
//		}
//		if ((*CurrFrame[0] && MIN_0) != 0)
//			*NextFrame[5] |= MIN_9;
//	}
	for (int i = 0; i < 6; i++)
	{
		x[i] = temp[i];
	}
}

void TLC5929_ArrayShiftHour( TLC5929_HandleTypeDef x[], uint8_t flag)
{
	uint16_t temp[6] = {0,0,0,0,0,0};
	if (flag == 0)
	{
		for (int i = 0; i < 6; i++)
		{
			temp[i] = (x[i].data & 0x7676U);
			temp[i] |= ((x[i].data & 0x0008U) << 8);
			temp[i] |= ((x[(i + 5) % 6].data & 0x0800U) >> 8);
		}

//		for (uint8_t t = 0; t< 6; t++)
//		{
//			if ((*CurrFrame[t] && HOUR_EVEN) != 0)
//				*NextFrame[t] |= HOUR_ODD;
//			if (((*CurrFrame[t] && HOUR_ODD) != 0) && (t != 5))
//				*NextFrame[t+1] |= HOUR_EVEN;
//		}
//		if ((*CurrFrame[5] && HOUR_ODD) != 0)
//			*NextFrame[0] |= HOUR_EVEN;
	}
	else
	{
		for (int i = 0; i < 6; i++)
		{
			temp[i] = (x[i].data & 0x7676U);
			temp[i] |= ((x[i].data & 0x0800U) >> 8);
			temp[i] |= ((x[(i + 5) % 6].data & 0x0008U) << 8);
		}
//		for (uint8_t t = 0; t< 6; t++)
//		{
//			if (((*CurrFrame[t] && HOUR_ODD) != 0) && (t != 0))
//				*NextFrame[t-1] |= HOUR_EVEN;
//			if ((*CurrFrame[t] && HOUR_EVEN) != 0)
//				*NextFrame[t] |= HOUR_ODD;
//		}
//		if ((*CurrFrame[0] && HOUR_EVEN) != 0)
//			*NextFrame[5] |= HOUR_ODD;
	}
}
/* LED Channel Association */
/*				How This Should Work
 * 	1) Get time, minutes
 * 	2) Add 2, then MOD 60 (ie 59 + 2)%60 = 1
 * 	3) Divide by 10 to get position (ie 1/10) = 0
 * 	4) MSB - 0 ........... 9 - LSB
 * 	For Hours
 * 	1) Get time, Hours
 * 	2) MOD 12, save result (23 % 12 = 11)
 * 	3) Result Mod 2 for even/odd (11%2 = 1)
 * 	4) Result MOD 6 for location (11%6 = 5)
 */

void TLC5929_TimeToDisplay(TLC5929_HandleTypeDef * x, uint8_t Hour, uint8_t Min)
{
	/* Function to get RTC H/M, store them*/
	/* one of two functions, initial on, immed. display, and N.Op. */
	//Array to store
	Min += 3; // Add 2 for offset, and 1 for next min

	uint8_t posm = Min/10; // SR that has minute value
	uint8_t posh = Hour % 12;

	for (uint8_t t = 0; t<6; t++)
	{
		//*HM_Array[t] = 0; // clear previous contents
//		use tlc5929[t].data
		x[t].data = 0; // clear contents

		if (posm == t)
		{
			switch (Min % 10) {
			case 0:
				x[t].data |= MIN_0;
				break;
			case 1:
				x[t].data |= MIN_1;
				break;
			case 2:
				x[t].data |= MIN_2;
				break;
			case 3:
				x[t].data |= MIN_3;
				break;
			case 4:
				x[t].data |= MIN_5;
				break;
			case 5:
				x[t].data |= MIN_5;
				break;
			case 6:
				x[t].data |= MIN_6;
				break;
			case 7:
				x[t].data |= MIN_7;
				break;
			case 8:
				x[t].data |= MIN_8;
				break;
			case 9:
				x[t].data |= MIN_9;
				break;
			default:
				break;

			}
		}

		if ((posh/2) % 6 == t )
		{
			switch ( posh % 2) {
			case 0:
				x[t].data |= HOUR_EVEN;
				break;
			case 1:
				x[t].data |= HOUR_ODD;
				break;
			default:
				break;
			}
		}
	}

}

//Time supervisor
