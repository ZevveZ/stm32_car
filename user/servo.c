/**
@file		servo.c
@brief		实现舵机控制相关的函数
@author		Zev
*/

#include "servo.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "delay.h"

#define SERVO_DELAY_TIME 1000	///<等待舵机转动到位的时间，单位为毫秒

static void servo_rcc_config(void);
static void servo_gpio_config(void);
static void servo_tim_config(void);

/**
@brief		配置舵机控制所需的时钟，GPIO口和定时器,并将舵机转动到正前方
@param		None
@retval 	None
*/
void servo_config(void){
	servo_rcc_config();
	servo_gpio_config();
	servo_tim_config();
		
	servo_turn_to(SERVO_DIRECTION_FRONT);	//将舵机转动到正前方
}
/**
@brief		初始化舵机控制所需的时钟
@param		None
@retval 	None
@note		这是一个私有函数
*/
static void servo_rcc_config(void){
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
}
/**
@brief		初始化舵机控制所需的GPIO口
@param		None
@retval 	None
@note		这是一个私有函数
*/
static void servo_gpio_config(void){
	GPIO_InitTypeDef gpio_init;
	
	gpio_init.GPIO_Mode=GPIO_Mode_AF;
	gpio_init.GPIO_OType=GPIO_OType_PP;
	gpio_init.GPIO_Pin=GPIO_Pin_1;
	gpio_init.GPIO_PuPd=GPIO_PuPd_NOPULL;
	gpio_init.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOB,&gpio_init);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource1,GPIO_AF_TIM3);
}
/**
@brief		初始化舵机控制的定时器
@param		None
@retval 	None
@note		这是一个私有函数
*/
static void servo_tim_config(void){
	TIM_TimeBaseInitTypeDef tim_base_init;
	TIM_OCInitTypeDef tim_oc_init;
	
	//舵机控制需要20ms的时基脉冲
	TIM_DeInit(TIM3);
	tim_base_init.TIM_ClockDivision=0;
	tim_base_init.TIM_CounterMode=TIM_CounterMode_Up;
	tim_base_init.TIM_Period=200-1;
	tim_base_init.TIM_Prescaler=8400-1;	//默认时钟频率为84MHz
	tim_base_init.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM3,&tim_base_init);
	TIM_OCStructInit(&tim_oc_init);
	tim_oc_init.TIM_OCMode=TIM_OCMode_PWM1;
	tim_oc_init.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OC4Init(TIM3,&tim_oc_init);
	TIM_Cmd(TIM3,ENABLE);	//持续的PWM信号可以维持舵机的状态不受外力改变
}
/**
@brief		控制舵机的转动方向
@param		dir 可选SERVO_DIRECTION_LEFT，SERVO_DIRECTION_FRONT，SERVO_DIRECTION_RIGHT
@retval 	None
@note		延时一段时间使舵机转动到位
*/
void servo_turn_to(SERVO_DIRECTION dir){
	TIM3->CCR4=dir;
	delay_ms(SERVO_DELAY_TIME);
}
