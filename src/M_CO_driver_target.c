/*
 * @Author: magyu
 * @Date: 2023-03-09 23:35:27
 * @LastEditors: magyu
 * @LastEditTime: 2023-03-09 23:50:20
 * @Description: 请填写简介
 */

#include "M_CO_driver_target.h"


SEM_EVENT_TYPE mEventNo;
M_CO_LEDs_t *LEDs;




void debug_LEDs_System_Init(void)
{

    M_CO_LEDs_init(LEDs);
    debugLEDsVSInitAll();
    mEventNo = eDebugLedsSetupDone;
    debugLEDsVSDeduct(mEventNo);

}



void debug_LEDs_System_Run(void)
{
    M_CO_LEDs_process(LEDs,);//todo 完善该函数，需要定义各个全局声明变量
    mEventNo = eLedsProcessDone;
    debugLEDsVSDeduct(mEventNo);
}



//todo 指向函数
void aLedsRun (void)
{
    
}


