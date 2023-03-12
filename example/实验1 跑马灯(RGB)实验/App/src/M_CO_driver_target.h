/*
 * @Author: magyu
 * @Date: 2023-03-08 17:28:50
 * @LastEditors: magyu
 * @LastEditTime: 2023-03-12 12:38:39
 * @Description: 请填写简介
 */

#ifndef M_CO_DRIVER_TARGET_H
#define M_CO_DRIVER_TARGET_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include "M_CO_LEDs_System.h"
#include "debugLEDs.h"
#include "M_CO_LEDs.h"
#include "M_CO_driver.h"
#include "led.h"
#include "delay.h"


/**
 * @brief 
 *  led 指示器初始化
 *  ! DO NOT EDIT THE FUNCTION!
 */
void debug_LEDs_System_Init(void);

/**
 * @brief 
 * 
 *  led 指示器运行
 *  该函数要放在循环中
 *  ! DO NOT EDIT THE FUNCTION!
 */
void debug_LEDs_System_Run(void);


void aLedsRun (void);
uint32_t M_CO_get_Time_Difference_us(void);
M_CO_SMT_internalState_t M_CO_get_SMT_internalState_t(void);
bool_t M_CO_get_SYSconfig(void);
bool_t M_CO_get_ErrPriority1(void);
bool_t M_CO_get_ErrPriority5(void);
bool_t M_CO_get_ErrPriority4(void);
bool_t M_CO_get_ErrPriority3(void);
bool_t M_CO_get_ErrPriority6(void);
bool_t M_CO_get_ErrOther(void);
bool_t M_CO_get_SYSotherState(void);


extern uint32_t tOld ;
extern uint32_t tNow ;
extern uint32_t tDiff ;
extern M_CO_SMT_internalState_t mSMT_internalState ;
extern bool_t sysConfig ;
extern bool_t ErrPriority1 ;
extern bool_t ErrPriority3 ;
extern bool_t ErrPriority4 ;
extern bool_t ErrPriority5 ;
extern bool_t ErrPriority6 ;
extern bool_t ErrOther ;
extern bool_t SYSotherState ;



#endif

