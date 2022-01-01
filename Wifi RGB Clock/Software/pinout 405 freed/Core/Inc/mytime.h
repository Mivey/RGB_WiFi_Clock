/*
 * mytime.h
 *
 *  Created on: May 27, 2021
 *      Author: marka
 */

#ifndef INC_MYTIME_H_
#define INC_MYTIME_H_

#include "stm32f4xx_hal.h"
#define RECENTEPOCH 0x5FEE6600U /* Friday, Jan 1, 2021. 12:00:00 AM */

#endif /* INC_MYTIME_H_ */

typedef struct {

	uint8_t second;
	uint8_t minute;
	uint8_t hour;
	uint8_t day;
	uint8_t month;
	uint8_t year;
	uint8_t weekday;
} tm;

tm breaktime(uint32_t epoch);
void displayData(uint16_t arr[6], uint32_t mflags[2], uint16_t hflags);
