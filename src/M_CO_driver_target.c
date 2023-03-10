/*
 * @Author: magyu
 * @Date: 2023-03-09 23:35:27
 * @LastEditors: magyu
 * @LastEditTime: 2023-03-10 21:02:44
 * @Description: 请填写简介
 */

#include "M_CO_driver_target.h"


SEM_EVENT_TYPE mEventNo;
M_CO_LEDs_t *LEDs;


/******************************************************************************/

void debug_LEDs_System_Init(void)
{

    M_CO_LEDs_init(LEDs);
    debugLEDsVSInitAll();
    mEventNo = eDebugLedsSetupDone;
    debugLEDsVSDeduct(mEventNo);

}


/******************************************************************************/

void debug_LEDs_System_Run(void)
{
    M_CO_LEDs_process(LEDs,
                     M_CO_get_Time_Difference_us(),
                     M_CO_get_SMT_internalState_t(),
                     M_CO_get_SYSconfig(),
                     M_CO_get_ErrPriority1,
                     M_CO_get_ErrPriority6,
                     M_CO_get_ErrPriority3,
                     M_CO_get_ErrPriority4,
                     M_CO_get_ErrPriority5,
                     M_CO_get_ErrOther,
                     M_CO_get_SYSotherState
                     );
    mEventNo = eLedsProcessDone;
    debugLEDsVSDeduct(mEventNo);
}

/******************************************************************************/

/*led运行输出函数*/
extern void aLedsRun (void) __attribute__((weak));
//在应用中实现具体内容


/******************************************************************************/

/*获取运行时间，单位是us*/
extern uint32_t M_CO_get_Time_Difference_us(void) __attribute__((weak));
//在应用中实现具体内容


/******************************************************************************/

/*获取系统运行内部的状态*/
extern M_CO_SMT_internalState_t M_CO_get_SMT_internalState_t(void) __attribute__((weak));
//如果需要在应用中实现具体内容


/******************************************************************************/

/*获取系统配置的状态*/
extern uint8_t __attribute__((weak)) M_CO_get_SYSconfig(void)
{
    return 0;
}
//如果需要在应用中实现具体内容


/******************************************************************************/

/*获取系统错误1的的状态*/
extern uint8_t __attribute__((weak)) M_CO_get_ErrPriority1(void)
{
    return 0;
}
//如果需要在应用中实现具体内容


/******************************************************************************/

/*获取系统错误5的的状态*/
extern uint8_t __attribute__((weak)) M_CO_get_ErrPriority5(void)
{
    return 0;
}
//如果需要在应用中实现具体内容


/******************************************************************************/

/*获取系统错误4的的状态*/
extern uint8_t __attribute__((weak)) M_CO_get_ErrPriority4(void)
{
    return 0;
}
//如果需要在应用中实现具体内容


/******************************************************************************/

/*获取系统错误6的的状态*/
extern uint8_t __attribute__((weak)) M_CO_get_ErrPriority6(void)
{
    return 0;
}
//如果需要在应用中实现具体内容


/******************************************************************************/

/*获取系统错误3的的状态*/
extern uint8_t __attribute__((weak)) M_CO_get_ErrPriority3(void)
{
    return 0;
}
//如果需要在应用中实现具体内容


/******************************************************************************/

/*获取系统其他错误的状态*/
extern uint8_t __attribute__((weak)) M_CO_get_ErrOther(void)
{
    return 0;
}
//如果需要在应用中实现具体内容


/******************************************************************************/

/*获取系统运行的其他状态*/
extern uint8_t __attribute__((weak)) M_CO_get_SYSotherState(void)
{
    return 0;
}
//如果需要在应用中实现具体内容


