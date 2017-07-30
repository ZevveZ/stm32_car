/**
@file		motor.c
@brief		实现与电机控制有关的函数
@author		Zev
*/

#include "motor.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "pid.h"
#include "main.h"

#define ENCODER_PERIOD 334										///<未经过减速器的电机转动一圈产生的脉冲数
#define REDUCTION_RATIO 43										///<电机的减速比
#define PULSE_PER_CIRCLE 21400									///<理论上PULSE_PER_CIRCLE==ENCODER_PERIOD*REDUCTION_RATIO，但是因为没有电机的参考资料，所以没用ENCODER_PERIOD*REDUCTION_RATIO，PULSE_PER_CIRCLE是实际测量的
#define SPEED_TO_CIRCLE(v) (v / (6.4f * PI) * PULSE_PER_CIRCLE) ///<将速度转换为PID采样周期100ms要达到的脉冲数，速度单位为dm/s，车轮的直径为6.4cm

static void motor_rcc_config(void);
static void motor_gpio_config(void);
static void motor_tim_config(void);
static void motor_nvic_config(void);

/**
@brief		配置电机控制所需的时钟，GPIO口，定时器，中断
@param		None
@retval		None
*/
void motor_config(void)
{
	motor_rcc_config();
	motor_gpio_config();
	motor_tim_config();
	motor_nvic_config();
}
/**
@brief		设定左右电机的速度
@param		speed0 设定右电机的速度
			speed1 设定左电机的速度
@retval		None
@note		速度单位为dm/s，速度的正负可以表示方向
*/
void motor_set_speed(float speed0, float speed1)
{
	if (speed0 < 0)
	{
		motor_set_direction(MOTOR_DIRECTION_BACKWARD, MOTOR_0);
		speed0 = -speed0;
	}
	else
	{
		motor_set_direction(MOTOR_DIRECTION_FORWARD, MOTOR_0);
	}
	if (speed1 < 0)
	{
		motor_set_direction(MOTOR_DIRECTION_BACKWARD, MOTOR_1);
		speed1 = -speed1;
	}
	else
	{
		motor_set_direction(MOTOR_DIRECTION_FORWARD, MOTOR_1);
	}

	pid_set_point(SPEED_TO_CIRCLE(speed0), SPEED_TO_CIRCLE(speed1)); //PID控制接受的参数是采样周期100ms内所要达到的脉冲数，因此需要对速度进行转换
}
/**
@brief		设定电机的转动方向
@param		dir 可选MOTOR_DIRECTION_FORWARD,MOTOR_DIRECTION_STOP,MOTOR_DIRECTION_BACKWARD,MOTOR_DIRECTION_KEEP
			motor 可选MOTOR_0，MOTOR_1或者两者的组合MOTOR_0|MOTOR_1
@retval		None
*/
void motor_set_direction(MOTOR_DIRECTION dir, uint16_t motor)
{
	uint16_t pin0 = 0, pin1 = 0;

	if (motor & MOTOR_0)
	{
		pin0 = GPIO_Pin_6;
		pin1 = GPIO_Pin_7;
	}
	if (motor & MOTOR_1)
	{
		pin0 |= GPIO_Pin_4;
		pin1 |= GPIO_Pin_5;
	}

	switch (dir)
	{
	case MOTOR_DIRECTION_STOP:
		GPIO_ResetBits(GPIOA, pin0 | pin1);
		break;
	case MOTOR_DIRECTION_FORWARD:
		GPIO_ResetBits(GPIOA, pin0);
		GPIO_SetBits(GPIOA, pin1);
		break;
	case MOTOR_DIRECTION_BACKWARD:
		GPIO_ResetBits(GPIOA, pin1);
		GPIO_SetBits(GPIOA, pin0);
		break;
	default:
		break;
	}
}
/**
@brief		获得单个电机的转动方向
@param		motor 可选MOTOR_0，MOTOR_1
@retval		返回电机motor的转动方向
*/
MOTOR_DIRECTION motor_get_direction(uint16_t motor)
{
	uint16_t pin0 = 0, pin1 = 0;

	if (motor & MOTOR_0)
	{
		pin0 = GPIO_Pin_6;
		pin1 = GPIO_Pin_7;
	}
	else if (motor & MOTOR_1)
	{
		pin0 = GPIO_Pin_4;
		pin1 = GPIO_Pin_5;
	}

	if (GPIO_ReadOutputDataBit(GPIOA, pin0) == RESET && GPIO_ReadOutputDataBit(GPIOA, pin1) == SET)
		return MOTOR_DIRECTION_FORWARD;
	else if (GPIO_ReadOutputDataBit(GPIOA, pin0) == SET && GPIO_ReadOutputDataBit(GPIOA, pin1) == RESET)
		return MOTOR_DIRECTION_BACKWARD;
	else
		return MOTOR_DIRECTION_STOP;
}
/**
@brief		配置电机控制所需的时钟
@param		None
@retval		None
*/
static void motor_rcc_config(void)
{
	//GPIO
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE);
	//TIM
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM8, ENABLE);
}
/**
@brief		配置电机控制所需的GPIO口
@param		None
@retval		None
@note		
*/
static void motor_gpio_config(void)
{
	GPIO_InitTypeDef gpio_init;

	GPIO_StructInit(&gpio_init);

	//PA0--MSP1;PA1--MSP2
	gpio_init.GPIO_Mode = GPIO_Mode_AF;
	gpio_init.GPIO_OType = GPIO_OType_PP;
	gpio_init.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; //PA0和PA1
	gpio_init.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &gpio_init);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5); //必须开启复用功能
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);

	//PA6--MT11,PA7--MT12;PA4--MT21,PA5--MT22
	gpio_init.GPIO_Mode = GPIO_Mode_OUT;
	gpio_init.GPIO_OType = GPIO_OType_PP;
	gpio_init.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_4 | GPIO_Pin_5; //PA6,PA7;PA4,PA5
	GPIO_Init(GPIOA, &gpio_init);

	//PA8--MP11,PA9--MP12;PC6--MP21,PC7--MP22
	gpio_init.GPIO_Mode = GPIO_Mode_AF;
	gpio_init.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9; //PA8,PA9
	gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &gpio_init);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1);

	gpio_init.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; //PC6,PC7
	GPIO_Init(GPIOC, &gpio_init);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8);
}
/**
@brief		配置电机控制所需的定时器
@param		None
@retval		None
@note		
*/
static void motor_tim_config(void)
{
	TIM_TimeBaseInitTypeDef tim_base_init;
	TIM_OCInitTypeDef tim_oc_init;
	TIM_ICInitTypeDef tim_ic_init;

	//TIM5_OC1--PA0--MSP1;TIM5_OC2--PA1--MSP2
	TIM_DeInit(TIM5);
	tim_base_init.TIM_ClockDivision = 0;
	tim_base_init.TIM_CounterMode = TIM_CounterMode_Up;
	tim_base_init.TIM_Period = TIM5_PERIOD;
	tim_base_init.TIM_Prescaler = 0;
	tim_base_init.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM5, &tim_base_init);

	TIM_OCStructInit(&tim_oc_init);
	tim_oc_init.TIM_OCMode = TIM_OCMode_PWM1;
	tim_oc_init.TIM_OutputState = TIM_OutputState_Enable;
	tim_oc_init.TIM_Pulse = 0; //0~TIM5_PERIOD+1
	TIM_OC1Init(TIM5, &tim_oc_init);
	TIM_OC2Init(TIM5, &tim_oc_init);
	TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Disable); //取消预加载，为了在motor_set_action函数中对CCR1的修改能够立即生效
	TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Disable);
	TIM_ARRPreloadConfig(TIM5, ENABLE); //使能预加载，防止溢出

	TIM_Cmd(TIM5, ENABLE);
	//TIM1_TI1--PA8--MP11,TIM1_TI2--PA9--MP12;TIM8_TI1--PC6--MP21,TIM8_TI2--PC7--MP22
	TIM_DeInit(TIM1);
	TIM_DeInit(TIM8);
	tim_base_init.TIM_Period = TIM1_PERIOD;
	TIM_TimeBaseInit(TIM1, &tim_base_init);
	TIM_TimeBaseInit(TIM8, &tim_base_init);

	TIM_ICStructInit(&tim_ic_init);
	TIM_ICInit(TIM1, &tim_ic_init);
	TIM_ICInit(TIM8, &tim_ic_init);
	TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI1, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_EncoderInterfaceConfig(TIM8, TIM_EncoderMode_TI1, TIM_ICPolarity_Rising, TIM_ICPolarity_Falling); //将TIM8_TI2修改为TIM_ICPolarity_Falling为了使TIM1和TIM8计数方向相同

	tim_ic_init.TIM_Channel = TIM_Channel_3;
	tim_ic_init.TIM_ICSelection = TIM_ICSelection_TRC;
	TIM_ICInit(TIM1, &tim_ic_init);
	TIM_ICInit(TIM8, &tim_ic_init);
	TIM_SelectInputTrigger(TIM1, TIM_TS_ITR1);
	TIM_SelectInputTrigger(TIM8, TIM_TS_ITR1);

	TIM_Cmd(TIM1, ENABLE);
	TIM_Cmd(TIM8, ENABLE);
	TIM_ClearFlag(TIM1, TIM_FLAG_Update | TIM_FLAG_CC3);
	TIM_ITConfig(TIM1, TIM_IT_Update | TIM_IT_CC3, ENABLE);
	TIM_ClearFlag(TIM8, TIM_FLAG_Update);
	TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE);
}
/**
@brief		配置电机控制所需的中断
@param		None
@retval		None
@note		
*/
static void motor_nvic_config(void)
{
	NVIC_InitTypeDef nvic_init;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

	nvic_init.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
	nvic_init.NVIC_IRQChannelCmd = ENABLE;
	nvic_init.NVIC_IRQChannelPreemptionPriority = 1;
	nvic_init.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&nvic_init);
	nvic_init.NVIC_IRQChannel = TIM8_UP_TIM13_IRQn;
	NVIC_Init(&nvic_init);

	nvic_init.NVIC_IRQChannel = TIM1_CC_IRQn;
	nvic_init.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&nvic_init);
}
