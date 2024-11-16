/*
 * myuart.h
 *
 *  Created on: Nov 16, 2024
 *      Author: dell
 */

#ifndef MYUART_H_
#define MYUART_H_

#include "main.h"
#include "string.h"
#include "stdarg.h"
#include "stdio.h"


void uart_print(const char *str);
void uart_printf(const char *format, ...);


#endif /* MYUART_H_ */
