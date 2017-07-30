/**
@file       sonar.c
@brief      实现与超声波控制有关的函数
@author     Zev
*/

#include "sonar.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "delay.h"

#define SONAR_SOUND_SPEED 340    ///<声音的传播速度，单位为m/s
#define SONAR_MAX_WAIT_TIME 60  ///<超声波最大等待时间为60ms，因为检测的距离不超过5米，来回所需的时间大约为30ms，60ms为其两倍的时间,可以根据实际缩短时间
#define SONAR_INFINITE_TIME -1.0f ///<如果障碍物的距离超过检测的范围就返回无限时间

static void sonar_rcc_config(void);
static void sonar_gpio_config(void);
static void sonar_tim_config(void);
static float sonar_trigger(void);

/**
@brief      配置与超声波模块有关的时钟，GPIO，定时器
@param      None
@retval     None
*/
void sonar_config(void)
{
    sonar_rcc_config();
    sonar_gpio_config();
    sonar_tim_config();
}
/**
@brief      调用此函数扫描当前方向的障碍物
@param      None
@retval     障碍物的距离信息，单位为m，如果障碍物的距离超过检测的范围将返回SONAR_INFINITE_DISTANCE
@note       调用此函数前需要先调用舵机模块对应的函数，调整好舵机的方向
*/
float sonar_scan_barrier(void)
{
    float time_s;

    time_s = sonar_trigger();
    return (time_s == SONAR_INFINITE_TIME ? SONAR_INFINITE_DISTANCE : time_s * SONAR_SOUND_SPEED / 2);
}
/**
@brief      配置超声波模块所需的时钟
@param      None
@retval     None
*/
void sonar_rcc_config(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
}
/**
@brief      配置超声波模块所需的GPIO
@param      None
@retval     None
*/
void sonar_gpio_config(void)
{
    GPIO_InitTypeDef gpio_init;
    //PB0--Trig
    gpio_init.GPIO_Mode = GPIO_Mode_OUT;
    gpio_init.GPIO_OType = GPIO_OType_PP;
    gpio_init.GPIO_Pin = GPIO_Pin_0;
    gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio_init.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &gpio_init);
    /*
    //TIM3_CH4--PC9--Echo
    gpio_init.GPIO_Mode=GPIO_Mode_AF;
    gpio_init.GPIO_Pin=GPIO_Pin_9;
    GPIO_Init(GPIOC,&gpio_init);
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource9,GPIO_AF_TIM3);
    */
    gpio_init.GPIO_Mode = GPIO_Mode_IN;
    gpio_init.GPIO_Pin = GPIO_Pin_9;
    gpio_init.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOC, &gpio_init);
}
/**
@brief      配置超声波模块所需的定时器
@param      None
@retval     None
*/
void sonar_tim_config(void)
{
    TIM_TimeBaseInitTypeDef tim_base_init;

    //设置为1MHz
    tim_base_init.TIM_ClockDivision = 0;
    tim_base_init.TIM_CounterMode = TIM_CounterMode_Up;
    tim_base_init.TIM_Period = 0xffff;
    tim_base_init.TIM_Prescaler = 84 - 1;
    tim_base_init.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM6, &tim_base_init);
}
/**
@brief      触发超声波传感器探测障碍物
@param      None
@retval     超声波往返时间，单位为s
@note       这是一个私有函数；这里使用轮询进行脉宽检测
*/
static float sonar_trigger(void)
{
    float time_s = SONAR_INFINITE_TIME;

    //触发一个10us的高电平启动超声波
    GPIO_SetBits(GPIOB, GPIO_Pin_0);
    delay_us(10);
    GPIO_ResetBits(GPIOB, GPIO_Pin_0);

    delay_ms_without_block(SONAR_MAX_WAIT_TIME);
    while (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_9) == RESET && delay_get_timing_delay())
        ; //在SONAR_MAX_WAIT_TIME时间内等待是否有回声

    if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_9))
    { //接受到回声
        TIM6->CNT = 0;
        TIM_Cmd(TIM6, ENABLE); //开始计时
        while (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_9) == SET)
            ;                      //轮询
        TIM_Cmd(TIM6, DISABLE); //结束计时
        time_s = TIM6->CNT * 1e-6; //将单位由us转换为s
    }
    return time_s;
}
