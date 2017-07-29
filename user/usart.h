/**
@file		usart.h
@brief		定义串口通信相关的接口函数
@author		Zev
*/
/**
@addtogroup usart
@brief 		串口模块
@{
*/
#ifndef __USART_H
#define __USART_H

#include "stm32f4xx.h"

/**
@brief		配置串口USART2的时钟和GPIO口，并初始化USART2
@param		None
@retval 	None
@note		调试时才需要调用此函数
*/
void usart_config(void);
/**
@brief		通过串口发送字符串
@param		msg 指向待发送字符串的指针
@retval 	None
*/
void usart_send_str(const char *msg);
/**
@brief		通过串口发送一个字符型数据
@param		c 待发送字符型数据
@retval 	None
*/
void usart_send_char(const char c);
/**
@brief		通过串口发送一个32位整型数据
@param		data 待发送的32位整型数据
@retval 	None
@note		从高位到低位发送数据，上位机接收时需要注意大小端问题
*/
void usart_send_int32(const int32_t data);
/**
@brief		通过串口发送一个浮点数据
@param		data 待发送的浮点数据
@retval 	None
@note		上位机接收时需要注意大小端问题
*/
void usart_send_float(const float data);
#endif
/**
@}
*/
