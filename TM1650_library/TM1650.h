/*
 * TM1650.h
 *
 *  Created on: Sep 3, 2025
 *      Author: Marcin Mikula
 */
#ifndef INC_TM1650_H_
#define INC_TM1650_H_

// Change the library according to your uC
#include "stm32l4xx_hal.h"

#include "math.h"


///// Data command settings
#define SYS_COMMAND 0x48
#define READ_COMMAND 0x49

///////  Brightness settings

#define BRIGHTNESS_DEFAULT 0x00 /// default (level 8)
#define BRIGHTNESS_1 0x10 /// default (level 1)
#define BRIGHTNESS_2 0x20 /// default (level 2)
#define BRIGHTNESS_3 0x30 /// default (level 3)
#define BRIGHTNESS_4 0x40 /// default (level 4)
#define BRIGHTNESS_5 0x50 /// default (level 5)
#define BRIGHTNESS_6 0x60 /// default (level 6)
#define BRIGHTNESS_7 0x70 /// default (level 7)


///Segment mode
#define SEGM_MODE_7 0x00
#define SEGM_MODE_8 0x08

//////// Display mode

#define DISP_MODE_OFF 0x00 //Off screen display
#define DISP_MODE_ON 0x01 ///Open screen display

//////// Working mode (normal/standby)

#define WORKING_MODE_NORMAL 0x00
#define WORKING_MODE_STANBY 0x04

// Init return values

/// RANGE_0, - positive integers in the range(0-9999)
/// RANGE_1 - integers in the range(-999,999)
/// RANGE_2, - positive and negative float numbers  (-99.9 : 999.9)
/// RANGE_3, - positive and negative float numbers  (-9.99:9.99)


typedef enum {RANGE_0,RANGE_1,RANGE_2, RANGE_3} range_mode;

typedef enum {ON, OFF} disp_mode;
typedef enum {BLANK, ZEROS} preceding_mode;




/////////////// TM1650 handle structure
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t br_lvl;
    range_mode range;
    disp_mode status;
    preceding_mode preceding;
} TM1650_HandleTypeDef;


/// Init
HAL_StatusTypeDef TM1650_Init(TM1650_HandleTypeDef *htm, I2C_HandleTypeDef *TM1650_hi2c, uint8_t br_lvl, uint8_t range, preceding_mode preceding);


///// Brightness
void TM1650_SetBrightness(TM1650_HandleTypeDef *htm, uint8_t brightness_lvl);

//// Range////////////

/// RANGE_0, - positive integers in the range(0-9999)
/// RANGE_1 - integers in the range(-999,999)
/// RANGE_2, - positive and negative float numbers  (-99.9 : 999.9)
/// RANGE_3, - positive and negative float numbers  (-9.99:9.99)

void TM1650_SetRange(TM1650_HandleTypeDef *htm, uint8_t range);

void TM1650_show_next_digit(TM1650_HandleTypeDef *htm);

void TM1650_prepare_number(TM1650_HandleTypeDef *htm, float number);
///// I2C sent flag check
void TM1650_current_digit_update(void);


uint8_t TM1650_check_i2c_flag(void);



#endif /* INC_TM1650_H_ */
