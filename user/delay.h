/**
@file		delay.h
@brief		定义与延时有关的函数接口
@author		Zev
*/
/**
@addtogroup delay
@brief		延时模块
@{
*/
#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f4xx.h"

void delay_us(int32_t us);
void delay_ms(int32_t ms);
void delay_ms_without_block(int32_t ms);
int32_t delay_get_timing_delay(void);
void timing_delay_decrement(void);
#endif
/**
@}
*/
