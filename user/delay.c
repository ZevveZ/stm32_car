/**
@file		delay.c
@brief		实现与延时有关的函数
@author		Zev
*/

#include "delay.h"

#define PRESCALER_US	1e6	///<微秒分频因子
#define PRESCALER_MS	1e3	///<毫秒分频因子

int32_t timing_delay;		///<记录延时时间

static void delay_config(int32_t prescaler);

/**
@brief		微秒级延时
@param		us 设定延时的微秒数
@retval		None
@note		这是一个阻塞函数
*/
void delay_us(int32_t us){
	delay_config(PRESCALER_US);
	
	timing_delay=us;
	while(timing_delay);
}
/**
@brief		毫秒级延时
@param		ms 设定延时的毫秒数
@retval		None
@note		这是一个阻塞函数
*/
void delay_ms(int32_t ms){
	delay_config(PRESCALER_MS);
	
	timing_delay=ms;
	while(timing_delay);
}
/**
@brief		毫秒级延时
@param		ms 设定延时的毫秒数
@retval		None
@note		这是一个非阻塞函数，需要结合delay_get_timing_delay函数使用
*/
void delay_ms_without_block(int32_t ms){
	delay_config(PRESCALER_MS);
	
	timing_delay=ms;
}
/**
@brief		获取当前剩余的延时时间
@param		None
@retval		当前剩余的延时时间
@note		需要结合delay_ms_without_block函数使用
*/
int32_t delay_get_timing_delay(void){
	return timing_delay;
}
/**
@brief		中断处理函数SysTick_Handler每隔一段时间将调用此函数递减延时时间，实现延时
@param		None
@retval		None
@note		此函数仅仅是提供给SysTick_Handler调用
*/
void timing_delay_decrement(void){
	if(timing_delay)
		--timing_delay;
}
/**
@brief		配置为微秒延时或者毫秒延时
@param		prescaler 可选PRESCALER_US，PRESCALER_MS
@retval		None
*/
static void delay_config(int32_t prescaler){
	while(SysTick_Config(SystemCoreClock/prescaler)){
		printf("In function delay_config:SysTick_Config failed\n");
	}
}
