/**
@file		fuzzy_decision.h
@brief		定义与模糊逻辑有关的数据结构和函数接口
@author		Zev
*/
/**
@addtogroup fuzzy_decision
@brief 		模糊决策算法模块
@{
*/
#ifndef __FUZZY_DECISION_H
#define __FUZZY_DECISION_H

#define DISTANCE_NEAR 0.2f	///<障碍物近的阈值，单位为m
#define DISTANCE_FAR 0.8f	///<障碍物远的阈值，单位为m
#define DISTANCE_SAFE 0.4f	///<安全阈值，单位为m，如果小车发现前方障碍物的距离小于DISTANCE_SAFE，就会停下来进行模糊决策

/**
@brief		模糊决策的结果，包含左右电机的速度，单位为dm/s
@param		None
@retval		None
*/
typedef struct{
	float speed0;	///<为了避开障碍物，右边电机应达到的速度，单位为dm/s
	float speed1;	///<为了避开障碍物，左边电机应达到的速度，单位为dm/s
}FUZZY_ST;

FUZZY_ST fuzzy_decision(float dl,float df,float dr);
#endif
/**
@}
*/
