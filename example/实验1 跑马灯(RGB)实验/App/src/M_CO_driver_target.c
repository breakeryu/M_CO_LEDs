/*
 * @Author: magyu
 * @Date: 2023-03-09 23:35:27
 * @LastEditors: magyu
 * @LastEditTime: 2023-03-12 12:44:39
 * @Description: 请填写简介
 */

#include "M_CO_driver_target.h"


SEM_EVENT_TYPE mEventNo;

uint32_t tOld = 0;
uint32_t tNow = 0;
uint32_t tDiff = 0;

M_CO_SMT_internalState_t mSMT_internalState = CO_SMT_OPERATIONAL;
bool_t sysConfig = 0;
bool_t ErrPriority1 = 0;
bool_t ErrPriority3 = 0;
bool_t ErrPriority4 = 0;
bool_t ErrPriority5 = 0;
bool_t ErrPriority6 = 0;
bool_t ErrOther = 0;
bool_t SYSotherState = 1;

/******************************************************************************/

void debug_LEDs_System_Init(void)
{

	M_CO_LEDs_init(&mLEDs);
    debugLEDsVSInitAll();
	mEventNo = SE_RESET;
	debugLEDsVSDeduct(mEventNo);
    mEventNo = eDebugLedsSetupDone;
    debugLEDsVSDeduct(mEventNo);

}


/******************************************************************************/

void debug_LEDs_System_Run(void)
{
	M_CO_LEDs_process(&mLEDs,
                     M_CO_get_Time_Difference_us(),
                     M_CO_get_SMT_internalState_t(),
                     M_CO_get_SYSconfig(),
                     M_CO_get_ErrPriority1(),
                     M_CO_get_ErrPriority6(),
                     M_CO_get_ErrPriority3(),
                     M_CO_get_ErrPriority4(),
                     M_CO_get_ErrPriority5(),
                     M_CO_get_ErrOther(),
                     M_CO_get_SYSotherState()
                     );
    mEventNo = eLedsProcessDone;
    debugLEDsVSDeduct(mEventNo);
}


/******************************************************************************/

/**
 * @brief 
 * 
 * led运行输出函数 
 * (用户自定义函数)
 */
void aLedsRun (void)
{
	LED_R(M_CO_LED_RED(&mLEDs, CO_LED_CANopen));
	LED_G(M_CO_LED_GREEN(&mLEDs, CO_LED_CANopen));
}



/******************************************************************************/

/**
 * @brief 
 * 
 * 获取两次循环运行的时间差，单位是us
 * (用户自定义函数)
 * @return uint32_t 
 * 返回和上次循环的时间差，单位是us
 */
uint32_t M_CO_get_Time_Difference_us(void)
{
	u32 reload = SysTick->LOAD;				
	tNow = SysTick->VAL;
	tDiff = 0;
	if(tNow <= tOld) tDiff = (tOld - tNow) / 80;	
	else tDiff = (reload - tNow + tOld) / 80;
	tOld = tNow;
	return tDiff;
}


/******************************************************************************/

/**
 * @brief 
 * 获取系统运行内部的状态
 * (用户自定义函数)
 * @return M_CO_SMT_internalState_t 
 */
M_CO_SMT_internalState_t M_CO_get_SMT_internalState_t(void)
{
	return mSMT_internalState;
}


/******************************************************************************/


/**
 * @brief 
 * 获取系统配置的状态
 * (用户自定义函数)
 * @return bool_t 
 */
bool_t M_CO_get_SYSconfig(void) 
{
	return sysConfig;
}


/******************************************************************************/

/**
 * @brief 
 * 获取系统错误1的的状态
 * (用户自定义函数)
 * @return bool_t 
 */
bool_t M_CO_get_ErrPriority1(void) 
{
	return ErrPriority1;
}


/******************************************************************************/

/**
 * @brief 
 * 获取系统错误5的的状态
 * (用户自定义函数)
 * @return bool_t 
 */
bool_t M_CO_get_ErrPriority5(void) 
{
	return ErrPriority5;
}


/******************************************************************************/

/**
 * @brief 
 * 获取系统错误4的的状态
 * (用户自定义函数)
 * @return bool_t 
 */
bool_t M_CO_get_ErrPriority4(void) 
{
	return ErrPriority4;
}


/******************************************************************************/

/**
 * @brief 
 * 获取系统错误3的的状态
 * (用户自定义函数)
 * @return bool_t 
 */
bool_t M_CO_get_ErrPriority3(void) 
{
	return ErrPriority3;
}


/******************************************************************************/

/**
 * @brief 
 * 获取系统错误6的的状态
 * (用户自定义函数)
 * @return bool_t 
 */
bool_t M_CO_get_ErrPriority6(void) 
{
	return ErrPriority6;
}
/******************************************************************************/

/**
 * @brief 
 * 获取系统其他错误的状态
 * (用户自定义函数)
 * @return bool_t 
 */
bool_t M_CO_get_ErrOther(void) 
{
	return ErrOther;
}


/******************************************************************************/

/**
 * @brief 
 * 获取系统运行的其他状态
 * (用户自定义函数)
 * @return bool_t 
 */
bool_t M_CO_get_SYSotherState(void) 
{
	return SYSotherState;
}

