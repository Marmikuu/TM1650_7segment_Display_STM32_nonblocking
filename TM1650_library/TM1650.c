/*
 * TM1650.c
 *
 *  Created on: Sep 3, 2025
 *      Author: Marcin Mikula
 */


////////////////////////////////////////  I do not care for encapsulation
#include "TM1650.h"

#include <stdlib.h>

//////////////////////////////// PRIVATE  VARIABLES //////////////

static const uint8_t numbers_09[10] = //// addresses that correspond to particular digit
{0x3F,/// 0
0x06, /// 1
0x5B, /// 2
0x4F, /// 3
0x66, /// 4
0x6D, /// 5
0x7D, /// 6
0x07, /// 7
0x7F  /// 8
,0x6F}; //9

static uint8_t buffer[4];
static uint8_t current_digit;
static uint8_t i2c_sent_flag; /// FLAG
static uint8_t sign;


static const uint8_t num_pos[4] =				///// the addresses of digits positions on the display (from right to left)
{0x68, /// 0 (right)
0x6A, // 1
0x6C, // 2
0x6E}; /// 3 (left)

//////////////////// FUNCTIONS

void TM1650_SetBrightness(TM1650_HandleTypeDef *htm, uint8_t brightness_lvl)  ///// set brightness level
{
	uint8_t br_lvl_address = 0x00; /// default value

	switch (brightness_lvl) /// the greater value ->> the brighter the screen is
	{
	case 1:
		br_lvl_address = BRIGHTNESS_1;
		break;
	case 2:
		br_lvl_address = BRIGHTNESS_2;
		break;
	case 3:
		br_lvl_address = BRIGHTNESS_3;
		break;
	case 4:
		br_lvl_address = BRIGHTNESS_4;
		break;
	case 5:
		br_lvl_address = BRIGHTNESS_5;
		break;
	case 6:
		br_lvl_address = BRIGHTNESS_6;
		break;
	case 7:
		br_lvl_address = BRIGHTNESS_7;
		break;
	default:

		br_lvl_address = BRIGHTNESS_DEFAULT;
		break;
	}

	uint8_t command;
	command = br_lvl_address | DISP_MODE_ON;
	HAL_I2C_Master_Transmit(htm->hi2c, SYS_COMMAND, &command , 1, 10);
	i2c_sent_flag = 1;
}


void TM1650_SetRange(TM1650_HandleTypeDef *htm, uint8_t range_new)  ///// set display range
{
	switch(range_new)
	{
	case 0: ////
		(htm->range) = RANGE_0;
		break;
	case 1:
		(htm->range) = RANGE_1;
		break;
	case 2:
		(htm->range) = RANGE_2;
		break;
	case 3:
		(htm->range) = RANGE_3;
		break;
	default:
		(htm->range) = RANGE_0;
	}
}



/////// Initialize a TM165O LED display

//// htm - tm1650 handle
//// br_lvl  1:8 (8- default and the brightest)
/// range -

HAL_StatusTypeDef TM1650_Init(TM1650_HandleTypeDef *htm, I2C_HandleTypeDef *TM1650_hi2c, uint8_t br_lvl, uint8_t range, preceding_mode preceding)
{
	i2c_sent_flag = 0;
	htm->hi2c = TM1650_hi2c;
	htm-> br_lvl = 8;
	htm->range = RANGE_0;
	htm->status = ON;
	htm->preceding = ZEROS; /// a default value, it will be configurable in the upcoming releases

	TM1650_SetRange(htm, range);
	TM1650_SetBrightness(htm, br_lvl);
	return HAL_OK;
}

///// function to compute exponential of 100 -> math.h should be included in 'main'
int32_t pow100(uint8_t exp) {
    int32_t result = 1;
    while (exp--) result *= 10;
    return result;
}

//void sign_number(int16_t number_int) //// adds a sign to the displayed number
//{
//	if (number_int <0)
//	{	buffer[3] = 0x40;} /// "-" sign if the number is negative
//	else
//	{
//		buffer[3] = 0x00;
//	}
//
//}

void out_of_range_show(void)
{
	for (int i=0;i<=3;i++)
	{
		buffer[i] = 0x40;
	}
}


uint8_t is_number_out_of_range(range_mode range, int16_t int_number) //// in case the number is out of range, just show '----'
{
	if (range == RANGE_0)
	{
		if ((int_number > 9999) || (int_number < 0)) /// check if the number is out of range
		{
			out_of_range_show();
			return 1;
		}
		else
		{
			return 0;
		}
	}
	else
	{
		if ((int_number > 999) || (int_number<-999))
		{
			out_of_range_show();
			return 1;
		}
		else
		{
			return 0;
		}
	}


}



///// left digit - pos 4 / right pos - 1
uint8_t convert_to_digit_09(int16_t number, uint8_t digit_pos, range_mode range) //// 4-digit number
{
//	number = abs(number);
//	if (((number > -1000) && (number < 1000) && ((digit_pos >=1) && (digit_pos <= 4))))
//	{
//	int16_t digit;
//	digit = ( (number/ pow100(digit_pos-1) ) % 10);
//	return (uint8_t)(abs(digit));
//	}
//	else
//	{
//		return 0;
//	}

	if (range == RANGE_0)
	{
		if ( ( ((number >= 0) && (number < 10000)) && ((digit_pos >=1) && (digit_pos <= 4)) ) )
		{
		uint16_t digit;
		digit = abs(( (number/ pow100(digit_pos-1) ) % 10));
		return digit;
		}
		else
		{
			return 0;
		}

	}
	else
	{
		if ( ( ((number > -1000) && (number < 1000)) && ((digit_pos >=1) && (digit_pos <= 4)) ) )
		{
		uint16_t digit;
		digit = abs(( (number/ pow100(digit_pos-1) ) % 10));
		return digit;
		}
		else
		{
			return 0;
		}
	}

}

/////////////////////////////////////// konwersja liczby 0-9 na kod I2C dla wyświetlacza
uint8_t convert_09_number_to_I2C(uint8_t digit, uint8_t point_pos, uint8_t counter,range_mode range)
{
	if ( range != RANGE_0)  ///// if its not the RANGE_0 ->> doesnt require a point, then:
	{
		if (counter == 3) //// if its a sign position
		{
				//// display sign
			    if (sign == 1)
			    {
			        return 0x40; //// "-"
			    }
			    else
			    {
		        return 0x00; /// blank
			    }
		}

		else //// if it's not a sign position, just fill the buffer with digit address (add a point if needed)
		{
			if ((point_pos >0) && (counter == point_pos))
			{
				return (numbers_09[digit] | 0x80);
			}
			else
			{
				if (digit >= 0 && digit <10)
				{

						return numbers_09[digit];
				}
				else
				{
				        return numbers_09[0]; /// blank
				}
			}
		}


	}
	else
	{
		if (digit >= 0 && digit <10)
		{

				return numbers_09[digit];
		}
		else
		{
		        return numbers_09[0]; /// blank
		}

	}
}


void TM1650_show_next_digit(TM1650_HandleTypeDef *htm)
{
	i2c_sent_flag = 1;
	if(current_digit >= 0 && current_digit < 4)
	{
		uint8_t temp_digit = buffer[current_digit];
		if (i2c_sent_flag == 1)
		{
			HAL_I2C_Master_Transmit_IT(htm->hi2c, num_pos[3-current_digit],&temp_digit , 1);
			i2c_sent_flag = 0;
		}
	}
}



///// ISR
void TM1650_current_digit_update()
{
	current_digit++;
	if (current_digit >3)
	{
		current_digit = 0;}
	i2c_sent_flag = 1; /// ustawienie flagi po wyslaniu na 1
}

void TM1650_update_buffer(TM1650_HandleTypeDef *htm,int16_t int_number,uint8_t point_pos)  //// fills the buffer table
{
		for(int i = 0; i<=3; i++)
		{
			uint8_t temp_buffer = convert_to_digit_09(int_number,i+1,htm->range);
			buffer[i] = convert_09_number_to_I2C(temp_buffer,point_pos,i,htm->range);
		}
}


//////////// Prepare number to display on the screen
void TM1650_prepare_number(TM1650_HandleTypeDef *htm, float number)
{
	uint8_t point_pos = 0;
	int16_t int_number = (int16_t)(number*1000);

	if ((int_number < 0))
	{
		sign = 1;
		number = fabs(number);
	}
	else
	{
		sign = 0;
	}

	{
		switch(htm->range)
		{

		case RANGE_0:  /// (0:9999)
		case RANGE_1: /// (-999:999)

			int_number = (int16_t)number; /////// conversion to int16_t
			break;

		case RANGE_2: //// (-99.9 : 99.9)
			point_pos = 1;
			int_number = (int16_t)(number * 100 + (number >= 0 ? 0.5f : -0.5f));
			break;

		case RANGE_3: /// (-9.99 : 9.99)
			point_pos = 2;

			int_number = (int16_t)(number * 1000 + (number >= 0 ? 0.5f : -0.5f));
			break;
		}

		if (is_number_out_of_range(htm->range, int_number) == 1) /// if the number is OUT OF range
		{}
		else
		{
			TM1650_update_buffer(htm,int_number,point_pos);
		}
	}
}

void TM1650_set_i2c_flag(uint8_t status)
{
	if (status == 1 )
	{
		i2c_sent_flag = 1;
	}
	else
		i2c_sent_flag = 0;
}


uint8_t TM1650_check_i2c_flag(void) ///// checks if a byte was sent trough I2C in interrupt mode
{
	if (i2c_sent_flag == 1) {
		return 1;
	}
	else{
		return 0;
	}

}



