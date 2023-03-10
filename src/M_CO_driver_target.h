/*
 * @Author: magyu
 * @Date: 2023-03-08 17:28:50
 * @LastEditors: magyu
 * @LastEditTime: 2023-03-10 20:56:07
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



/* NULL is defined in stddef.h */
/* true and false are defined in stdbool.h */
/* int8_t to uint64_t are defined in stdint.h */
typedef uint_fast8_t            bool_t;
typedef float                   float32_t;
typedef double                  float64_t;

/**
 * @brief 
 *  led 指示器初始化
 */
void debugLEDsInit(void);

/**
 * @brief 
 * 
 *  led 指示器运行
 *  该函数要放在循环中
 */
void debug_LEDs_System_Run(void);

/**
 * @brief 
 * 
 */
extern void aLedsRun (void) __attribute__((weak));


extern uint32_t M_CO_get_Time_Difference_us(void) __attribute__((weak));
extern M_CO_SMT_internalState_t M_CO_get_SMT_internalState_t(void) __attribute__((weak));
extern uint8_t __attribute__((weak)) M_CO_get_SYSconfig(void);
extern uint8_t __attribute__((weak)) M_CO_get_ErrPriority1(void);
extern uint8_t __attribute__((weak)) M_CO_get_ErrPriority5(void);
extern uint8_t __attribute__((weak)) M_CO_get_ErrPriority4(void);
extern uint8_t __attribute__((weak)) M_CO_get_ErrPriority3(void);
extern uint8_t __attribute__((weak)) M_CO_get_ErrPriority6(void);
extern uint8_t __attribute__((weak)) M_CO_get_ErrOther(void);
extern uint8_t __attribute__((weak)) M_CO_get_SYSotherState(void);


#endif

