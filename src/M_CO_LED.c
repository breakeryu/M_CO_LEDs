/*
 * @Author: magyu
 * @Date: 2023-03-08 16:23:18
 * @LastEditors: magyu
 * @LastEditTime: 2023-03-09 23:45:09
 * @Description: M_CO_LED 使用CANOPEN协议中LED指示器部分代码
 * @Version: 0.1 2023/3/8 初步构建 
 */

#include "M_CO_LEDs.h"




/******************************************************************************/
M_CO_ReturnError_t M_CO_LEDs_init(M_CO_LEDs_t *LEDs) 
{
    M_CO_ReturnError_t ret = CO_ERROR_NO;

    /* verify arguments */
    if (LEDs == NULL) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    /* clear the object */
    memset(LEDs, 0, sizeof(M_CO_LEDs_t));

    return ret;
}



/******************************************************************************/
void M_CO_LEDs_process(M_CO_LEDs_t *LEDs,
                     uint32_t timeDifference_us,
                     M_CO_SMT_internalState_t SMTstate,
                     bool_t LSSconfig,
                     bool_t ErrCANbusOff,
                     bool_t ErrCANbusWarn,
                     bool_t ErrRpdo,
                     bool_t ErrSync,
                     bool_t ErrHbCons,
                     bool_t ErrOther,
                     bool_t firmwareDownload,
                     bool_t ErrFirstEm,
                     uint32_t *timerNext_us)
{
    (void)timerNext_us; /* may be unused */

    uint8_t rd = 0;
    uint8_t gr = 0;
    bool_t tick = false;

    LEDs->LEDtmr50ms += timeDifference_us;
    while (LEDs->LEDtmr50ms >= 50000) {
        bool_t rdFlickerNext = (LEDs->LEDred & CO_LED_flicker) == 0;

        tick = true;
        LEDs->LEDtmr50ms -= 50000;

        if (++LEDs->LEDtmr200ms > 3) {
            /* calculate 2,5Hz blinking and flashing */
            LEDs->LEDtmr200ms = 0;
            rd = gr = 0;

            if ((LEDs->LEDred & CO_LED_blink) == 0) rd |= CO_LED_blink;
            else                                    gr |= CO_LED_blink;

            switch (++LEDs->LEDtmrflash_1) {
                case 1: rd |= CO_LED_flash_1; break;
                case 2: gr |= CO_LED_flash_1; break;
                case 6: LEDs->LEDtmrflash_1 = 0; break;
                default: break;
            }
            switch (++LEDs->LEDtmrflash_2) {
                case 1: case 3: rd |= CO_LED_flash_2; break;
                case 2: case 4: gr |= CO_LED_flash_2; break;
                case 8: LEDs->LEDtmrflash_2 = 0; break;
                default: break;
            }
            switch (++LEDs->LEDtmrflash_3) {
                case 1: case 3: case 5: rd |= CO_LED_flash_3; break;
                case 2: case 4: case 6: gr |= CO_LED_flash_3; break;
                case 10: LEDs->LEDtmrflash_3 = 0; break;
                default: break;
            }
            switch (++LEDs->LEDtmrflash_4) {
                case 1: case 3: case 5: case 7: rd |= CO_LED_flash_4; break;
                case 2: case 4: case 6: case 8: gr |= CO_LED_flash_4; break;
                case 12: LEDs->LEDtmrflash_4 = 0; break;
                default: break;
            }
        }
        else {
            /* clear flicker and CANopen bits, keep others */
            rd = LEDs->LEDred & (0xFF ^ (CO_LED_flicker | CO_LED_CANopen));
            gr = LEDs->LEDgreen & (0xFF ^ (CO_LED_flicker | CO_LED_CANopen));
        }

        /* calculate 10Hz flickering */
        if (rdFlickerNext) rd |= CO_LED_flicker;
        else               gr |= CO_LED_flicker;

    } /* while (LEDs->LEDtmr50ms >= 50000) */

    if (tick) {
        uint8_t rd_co, gr_co;

        /* CANopen red ERROR LED */
        if      (ErrCANbusOff)                      rd_co = 1;
        else if (SMTstate == CO_SMT_INITIALIZING)   rd_co = rd & CO_LED_flicker;
        else if (ErrRpdo)                           rd_co = rd & CO_LED_flash_4;
        else if (ErrSync)                           rd_co = rd & CO_LED_flash_3;
        else if (ErrHbCons)                         rd_co = rd & CO_LED_flash_2;
        else if (ErrCANbusWarn)                     rd_co = rd & CO_LED_flash_1;
        else if (ErrOther)                          rd_co = rd & CO_LED_blink;
        else                                        rd_co = 0;

        /* CANopen green RUN LED */
        if      (LSSconfig)                         gr_co = gr & CO_LED_flicker;
        else if (firmwareDownload)                  gr_co = gr & CO_LED_flash_3;
        else if (SMTstate == CO_SMT_STOPPED)        gr_co = gr & CO_LED_flash_1;
        else if (SMTstate == CO_SMT_PRE_OPERATIONAL)gr_co = gr & CO_LED_blink;
        else if (SMTstate == CO_SMT_OPERATIONAL)    gr_co = 1;
        else                                        gr_co = 0;

        if (rd_co != 0) rd |= CO_LED_CANopen;
        if (gr_co != 0) gr |= CO_LED_CANopen;
        LEDs->LEDred = rd;
        LEDs->LEDgreen = gr;
    } /* if (tick) */

#if (CO_CONFIG_LEDS) & CO_CONFIG_FLAG_TIMERNEXT
    if (timerNext_us != NULL) {
        uint32_t diff = 50000 - LEDs->LEDtmr50ms;
        if (*timerNext_us > diff) {
            *timerNext_us = diff;
        }
    }
#endif
}