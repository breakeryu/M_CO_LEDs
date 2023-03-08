/*
 * @Author: magyu
 * @Date: 2023-03-08 20:06:26
 * @LastEditors: magyu
 * @LastEditTime: 2023-03-08 22:17:25
 * @Description: 请填写简介
 */

M_CO_LEDs_t LEDs;

uint8_t ErrFirstEm;
uint32_t time_old;

void changeErrFirstEm(uint8_t value)
{
    ErrFirstEm = value;
}



void app_process() {
    /* loop for normal program execution ******************************************/
    /* get time difference since last function call */
    uint8_t outStatusLEDRed;
    uint8_t outStatusLEDGreen;

    uint32_t time_current = app_GetTick();
    
    if ((time_current - time_old) > 1000) { // Make sure more than 1ms elapsed
     
        uint32_t timeDifference_us = time_current - time_old;
        time_old = time_current;

        M_CO_LEDs_process(LEDs,
                     timeDifference_us,
                     M_CO_SMT_internalState_t SMTstate,
                     bool_t LSSconfig,
                     bool_t ErrCANbusOff,
                     bool_t ErrCANbusWarn,
                     bool_t ErrRpdo,
                     bool_t ErrSync,
                     bool_t ErrHbCons,
                     bool_t ErrOther,
                     bool_t firmwareDownload,
                     ErrFirstEm,
                     uint32_t *timerNext_us);

        outStatusLEDRed = CO_LED_RED(CO->LEDs, CO_LED_CANopen);
        outStatusLEDGreen = CO_LED_GREEN(CO->LEDs, CO_LED_CANopen);

    }

    

}

