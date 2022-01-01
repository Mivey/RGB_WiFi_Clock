/*
 * mytime.c
 *
 *  Created on: May 27, 2021
 *      Author: marka
 */



#include "mytime.h"

uint8_t months[]  = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
tm breaktime(uint32_t epoch)
{
	tm downtime;
	int yearcnt = 0;
	int monthcnt = 0;
	int daycnt = 0;

	epoch = epoch - RECENTEPOCH; // base epoch on Jan 1, 2021
	downtime.second = epoch % 60;
	epoch /= 60; // divide by seconds
	downtime.minute = epoch % 60;
	epoch /= 60; // divide by minutes
	int z = (8 + (epoch % 12))%12;
	downtime.hour = ( z == 0 ? 12 : z);
	epoch /= 24; // divide by hours
	z = ((epoch % 7) + 5)% 7;
	downtime.weekday =  (z == 0 ? 7 : z) ; // friday offset (5) + number of days

	while (epoch > 0x016D ) // check if it's been more than a year.
	{
		epoch -= 0x16D;
		if ((yearcnt % 4) == 3)
			epoch--;
		yearcnt++;
	}
	 downtime.year = 21 + yearcnt;
	 while (epoch > 31) // max number of day in month
	 {
		 epoch = epoch - months[monthcnt];
		 monthcnt++;
		 if ((monthcnt == 2) && ((yearcnt % 4) == 3))
			 epoch--;
	 }
	 if (epoch > months[monthcnt])
		 epoch -= months[monthcnt];
	 downtime.month = monthcnt + 1;
	 downtime.day = epoch;

	 return downtime;

}

void displayData(uint16_t arr[6], uint32_t mflags[2], uint16_t hflags)
{
	//properly align data, move 58, 59 to LSB
	uint32_t tflag[2];
	tflag[0] = (mflags[0] << 2) | ((mflags[1] >> 28) & 0x03);
	tflag[1] = (mflags[1] << 2) | ((mflags[0] >> 28) & 0x03);

	uint8_t data[12];
	int t = 0;
	for (int k = 0; k < 2; k++)
	{
		for (int i = 0; i < 6; i++)
		{
			//move last 5 bits to temp
			uint8_t temp = (tflag[k] & 0x1F);
			tflag[k] >>=5;
			//clear data
			data[t] = 0;
			if ((hflags & (0x01<<t)) != 0)
				data[t] |= 0x08;
			for(int j = 0; j < 5; j++)
			{
				uint8_t x = temp & (0x01 << j);
				switch (x)
				{
					case (0x01):
							data[t] |= 0x40;
							break;

					case (0x02):
							data[t] |= 0x20;
							break;

					case (0x04):
							data[t] |= 0x10;
							break;

					case (0x08):
							data[t] |= 0x04;
							break;

					case (0x10):
							data[t] |= 0x02;
							break;

					default:
						break;
				}
			}
			t++;
		}
	}
	// put it all together
	for (int i = 0; i < 6; i++)
		arr[i] = (data[i*2] << 8) | data[i*2+1];
}
