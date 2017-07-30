/**
@file		usart.c
@brief		实现串口通信相关的函数
@author		Zev
*/

#include "usart.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"

static void usart_rcc_config(void);
static void usart_gpio_config(void);
static void usart_nvic_config(void);

/**
@brief		配置串口USART2的时钟和GPIO口，并初始化USART2
@param		None
@retval 	None
@note		调试时才需要调用此函数
*/
void usart_config(void)
{
	USART_InitTypeDef usart_init;

	usart_rcc_config();
	usart_gpio_config();

	usart_init.USART_BaudRate = 115200;
	usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usart_init.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	usart_init.USART_Parity = USART_Parity_No;
	usart_init.USART_StopBits = USART_StopBits_1;
	usart_init.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART2, &usart_init);

	USART_ClearFlag(USART2, USART_FLAG_RXNE);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	usart_nvic_config();

	USART_Cmd(USART2, ENABLE);
}
/**
@brief		配置串口通信需要的时钟
@param		None
@retval 	None
@note		这是一个私有函数
*/
static void usart_rcc_config(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
}
/**
@brief		配置串口USART2的GPIO口
@param		None
@retval 	None
@note		这是一个私有函数
*/
static void usart_gpio_config(void)
{
	GPIO_InitTypeDef gpio_init;

	//USART2_TX--PA2,USART2_RX--PA3
	gpio_init.GPIO_Mode = GPIO_Mode_AF;
	gpio_init.GPIO_OType = GPIO_OType_PP;
	gpio_init.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	gpio_init.GPIO_PuPd = GPIO_PuPd_UP;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio_init);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
}
/**
@brief		配置串口USART2的中断
@param		None
@retval 	None
@note		这是一个私有函数；注意函数将串口中断配置为最高优先级，以便及时接收上位机发送的命令
*/
static void usart_nvic_config(void)
{
	NVIC_InitTypeDef nvic_init;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

	nvic_init.NVIC_IRQChannel = USART2_IRQn;
	nvic_init.NVIC_IRQChannelCmd = ENABLE;
	nvic_init.NVIC_IRQChannelPreemptionPriority = 0;
	nvic_init.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&nvic_init);
}
/**
@brief		通过串口发送字符串
@param		msg 指向待发送字符串的指针
@retval 	None
*/
void usart_send_str(const char *msg)
{
	if (!msg)
		return;

	while (*msg != '\0')
	{
		usart_send_char(*msg++);
	}
}
/**
@brief		通过串口发送一个32位整型数据
@param		data 待发送的32位整型数据
@retval 	None
@note		从高位到低位发送数据，上位机接收时需要注意大小端问题
*/
void usart_send_int32(const int32_t data)
{
	while (!USART_GetFlagStatus(USART2, USART_FLAG_TXE))
		; //等待发送缓冲区空闲
	USART_SendData(USART2, (data & 0xff000000) >> 24);
	while (!USART_GetFlagStatus(USART2, USART_FLAG_TXE))
		;
	USART_SendData(USART2, (data & 0xff0000) >> 16);
	while (!USART_GetFlagStatus(USART2, USART_FLAG_TXE))
		;
	USART_SendData(USART2, (data & 0xff00) >> 8);
	while (!USART_GetFlagStatus(USART2, USART_FLAG_TXE))
		;
	USART_SendData(USART2, data & 0xff);
	while (!USART_GetFlagStatus(USART2, USART_FLAG_TXE))
		;
}
/**
@brief		通过串口发送一个字符型数据
@param		c 待发送字符型数据
@retval 	None
*/
void usart_send_char(const char c)
{
	while (!USART_GetFlagStatus(USART2, USART_FLAG_TXE))
		; //等待发送缓冲区空闲
	USART_SendData(USART2, c);
	while (!USART_GetFlagStatus(USART2, USART_FLAG_TXE))
		; //等待发送结束
}
/**
@brief		通过串口发送一个浮点数据
@param		data 待发送的浮点数据
@retval 	None
@note		上位机接收时需要注意大小端问题
*/
void usart_send_float(const float data)
{
	union {
		float f;
		char s[sizeof(float)];
	} f2s;

	f2s.f = data;
	for (int i = 0; i < sizeof(float); ++i)
	{
		usart_send_char(f2s.s[i]);
	}
}
