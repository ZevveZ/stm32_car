/**
@file		sonar.h
@brief		定义与超声波控制有关的接口函数
@author		Zev
*/

/**
@addtogroup sonar
@brief		超声波模块
@{
*/
#ifndef __SONAR_H
#define __SONAR_H

#define SONAR_INFINITE_DISTANCE -1.0f ///<如果障碍物的距离超过检测的范围就返回无限距离

/**
@brief		配置与超声波模块有关的时钟，GPIO，定时器
@param		None
@retval		None
*/
void sonar_config(void);
/**
@brief		调用此函数扫描当前方向的障碍物
@param		None
@retval		障碍物的距离信息，单位为m，如果障碍物的距离超过检测的范围将返回SONAR_INFINITE_DISTANCE
@note		调用此函数前需要先调用舵机模块对应的函数，调整好舵机的方向
*/
float sonar_scan_barrier(void);
#endif
/**
@}
*/
