/*
 * i2c_bitbang.h
 *
 *  Created on: Oct 31, 2024
 *      Author: dell
 */

#ifndef I2C_BITBANG_H_
#define I2C_BITBANG_H_

#include "main.h"
#include <stdbool.h>

#include "stdbool.h"
#include "stm32f4xx.h"
/*Define SDA and SCL below after configure GPIO in ioc*/

#define I2C_GPIO_PORT GPIOB
#define I2C_SCL_PIN LL_GPIO_PIN_6
#define I2C_SDA_PIN LL_GPIO_PIN_7

#define ACK 0
#define NACK 1

#define TIME_DELAY 5

#define I2C_DELAY() DWT_Delay_us(TIME_DELAY);
#define NUMBER_OF_SLAVES 1



void I2C_Bitbang_Init(void);
void DWT_Clock_Enable(void);
void DWT_Delay_us(volatile uint32_t microseconds);
void I2C_Event_Take(); // for interrupt
void check_start_condition();
#endif                 /* I2C_BITBANG_H_ */
