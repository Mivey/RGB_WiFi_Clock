#ifndef TLC5929_LED_H
#define TLC5929_LED_H

#include "stm32f4xx_hal.h"

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

typedef enum {
	MIN_0		= 0x4000,
	MIN_1		= 0x2000,
	MIN_2		= 0x1000,
	MIN_3		= 0x0400,
	MIN_4		= 0x0200,
	MIN_5		= 0x0040,
	MIN_6		= 0x0020,
	MIN_7		= 0x0010,
	MIN_8		= 0x0004,
	MIN_9		= 0x0002,

	HOUR_ODD	= 0x0008,
	HOUR_EVEN	= 0x0800,

	LOW_PWR		= 0x8000,

	TLC_REG		=0xE67F //Global brightness at 100%, [6:0] bits
	//TLC_BC		= 0xFF7FU
} TLC5929_TIME;

typedef enum { /* Position of the TLC5929 chip */
	POS0 		= 0x01,
	POS1 		= 0x02,
	POS2		= 0x04,
	POS3 		= 0x08,
	POS4 		= 0x10,
	POS5 		= 0x20
} TLC5929_POS;

typedef struct {
	//Timer for Clock line
	TIM_HandleTypeDef 				*PWM_Channel;
	uint32_t				 		Clock; // should be uint16_t?
	// GPIO Bank location (GPIOA, GPIOB...)
	GPIO_TypeDef					*LatchBank;
	GPIO_TypeDef					*BlankBank;
	GPIO_TypeDef					*Data_inBank; //Add 'uint16_t xxPin' for these 3
	// GPIO Pin Location
	uint16_t						LatchPin;
	uint16_t						BlankPin;
	uint16_t						Data_inPin;
	// TLC5929 Chip location (POS0, POS1, etc)
	uint8_t							Shift_Reg_Loc;
	// Bri. Ctrl for Config reg.
	uint16_t						TLC_BC;
	uint8_t							TLC_Reg;
	/*
	 * *********** Ctrl*********
	 * Bit 0		Hi = set blank
	 * Bit 1		Hi = Data sent ...?
	 * Bit 2		Hi = Set low pwr pin
	 * Bit 3		Hi = SR tlc5929 control, low is SR led
	 * Bit 4		Hi = Initialization Flag ... ?
	 * Bit 5
	 * Bit 6
	 * Bit 7
	 *
	 */
	uint16_t data;


} TLC5929_HandleTypeDef;


/*
 *
 * INITIATILIZATION
 *
 */

void TLC5929_Init(TLC5929_HandleTypeDef * shift_reg,
											TIM_HandleTypeDef *PWM_Channel, uint32_t Clock,
											GPIO_TypeDef *LatchBank, uint16_t LatchPin,
											GPIO_TypeDef *BlankBank, uint16_t BlankPin,
											GPIO_TypeDef *Data_inBank, uint16_t Data_inPin,
											uint8_t Shift_Reg_Loc);


/*
 * Add init function that puts all leds at default brightness, sends 0?
 */

void TLC5929_TimeToDisplay( TLC5929_HandleTypeDef * x, uint8_t Hour, uint8_t Min);

void TLC5929_ArrayShiftHour( TLC5929_HandleTypeDef * x, uint8_t flag);

//void TLC5929_ArrayShiftMin(TLC5929_HandleTypeDef * x , uint8_t flag);

void TLC5929_ArrayShiftMin(uint16_t *x , uint8_t flag);

#endif

/*
 * how I think the TLC5929 ISR will work
 * set timer to 1200 ticks/sec
 * task 2:
 * 		Declare global pointer, or put pointer in queue
 * 		enable PWM
 * 		index i = 0, put in queue
 * 		then 'tasknotifytake' blocking statement
 * 		i++
 * 		'if(i == 17)'
 * 			disable pwm,
 * 			set flags,
 * 			notify task 1 timer is ready
 * 		else
 * 			update queue index value
 * 	PWM callback:
 * 		pass pointer of TLC5929 to callback via queue
 * 		pass index value via queue
 * 		tasknotify for task 2
 * 		sethigherpriority task 2
 * 		if (data && (1 << i) == 0)
 * 			hal_gpio_pin(gpiox, gpion, reset)
 * 		else
 * 			hal_gpio_pin(gpiox,gpion, set)
 * 		Tasknotifyfromisr
 * 		portyieldfromISR
 * 	Task 1
 * 		initial setup data calculations for TLC5929 chip n,
 * 		if(data !=0)
 * 			pwm config
 * 			tasknotify?
 * 		else
 * 			set flags
 */
