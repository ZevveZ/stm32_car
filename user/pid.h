/**
@file		pid.h
@brief		定义与PID控制有关的接口函数
@author		Zev
*/
/**
@addtogroup pid
@brief 		pid控制模块
@{
*/

#ifndef __PID_H
#define __PID_H

#include "stm32f4xx.h"
#include "motor.h"

/**
@brief		配置PID控制所需的时钟和定时器，并初始化与PID控制有关的结构体
@param		None
@retval 	None
@note		在第一次使用PID控制前必须调用此函数进行时钟和定时器的配置，以后不需要调用此函数
*/
void pid_config(void);

/**
@brief		通过当前时刻的两个电机的光电码盘脉冲数计算并设置下一时刻两个电机的输出脉宽
@param		step0 右边电机在采样时间段T_SAMPLING的光电码盘脉冲数
@param		step1 左边电机在采样时间段T_SAMPLING的光电码盘脉冲数
@retval		None
*/
void pid_control(int32_t step0, int32_t step1);

/**
@brief		设置两个电机在一秒要达到的脉冲数
@param		point0 右边电机在一秒要达到的脉冲数
@param		point1 左边电机在一秒要达到的脉冲数
@retval		None
*/
void pid_set_point(int32_t point0, int32_t point1);

#if defined(DEBUG_PID)
/**
@brief		根据函数的形参调整PID的参数和电机的转动方向，用于整定PID控制的参数
@param		point 在采样时间段T_SAMPLING电机光电码盘脉冲数要达到的目标值
@param		pd 比例带，其倒数即为比例系数
@param		ti 积分时间，单位为秒
@param		td 微分时间，单位为秒，本程序使用的是PI控制，因此不需要使用该参数
@param		dir 电机转动方向
@retval		None
@note		在调试PID的参数时才需要使用此函数
*/
void pid_debug_params(uint32_t pwm, float pd, float ti, float td, MOTOR_DIRECTION dir);
#endif
#endif
/**
@}
*/
