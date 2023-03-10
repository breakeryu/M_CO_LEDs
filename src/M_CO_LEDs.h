/*
 * @Author: magyu
 * @Date: 2023-03-08 16:23:10
 * @LastEditors: magyu
 * @LastEditTime: 2023-03-10 20:30:36
 * @Description: 请填写简介
 */

#ifndef M_CO_LEDS_H
#define M_CO_LEDS_H

#include "M_CO_driver.h"
#include "M_CO_driver_target.h"




/** 
 * 无效的代码注释是不需要的! 
 * todo 这一部分需要修改！
 * @defgroup CO_LEDs LED indicators
 * Specified in standard CiA 303-3.
 *
 * @ingroup CO_CANopen_303
 * @{
 *
 * CIA 303-3 standard specifies indicator LED diodes, which reflects state of
 * the CANopen device. Green and red leds or bi-color led can be used.
 *
 * CANopen green led - run led:
 * - flickering: LSS configuration state is active
 * - blinking: device is in NMT pre-operational state
 * - single flash: device is in NMT stopped state
 * - triple flash: a software download is running in the device
 * - on: device is in NMT operational state
 *
 * CANopen red led - error led:
 * - off: no error
 * - flickering: LSS node id is not configured, CANopen is not initialized
 * - blinking: invalid configuration, general error
 * - single flash: CAN warning limit reached
 * - double flash: heartbeat consumer - error in remote monitored node
 * - triple flash: sync message reception timeout
 * - quadruple flash: PDO has not been received before the event timer elapsed
 * - on: CAN bus off
 *
 * To apply on/off state to led diode, use #CO_LED_RED and #CO_LED_GREEN macros.
 * For CANopen leds use CO_LED_BITFIELD_t CO_LED_CANopen. Other bitfields are
 * available for implementing custom leds.
 */

/** Bitfield for combining with red or green led */
typedef enum {
    CO_LED_flicker = 0x01,  /**< LED flickering 10Hz */
    CO_LED_blink   = 0x02,  /**< LED blinking 2,5Hz */
    CO_LED_flash_1 = 0x04,  /**< LED single flash */
    CO_LED_flash_2 = 0x08,  /**< LED double flash */
    CO_LED_flash_3 = 0x10,  /**< LED triple flash */
    CO_LED_flash_4 = 0x20,  /**< LED quadruple flash */
    CO_LED_CANopen = 0x80   /**< LED CANopen according to CiA 303-3 */
} M_CO_LED_BITFIELD_t;

/** Get on/off state for green led for specified bitfield */
#define M_CO_LED_RED(LEDs, BITFIELD) (((LEDs)->LEDred & BITFIELD) ? 1 : 0)
/** Get on/off state for green led for specified bitfield */
#define M_CO_LED_GREEN(LEDs, BITFIELD) (((LEDs)->LEDgreen & BITFIELD) ? 1 : 0)


/**
 * LEDs object, initialized by M_CO_LEDs_init()
 */
typedef struct{
    uint32_t            LEDtmr50ms;     /**< 50ms led timer */
    uint8_t             LEDtmr200ms;    /**< 200ms led timer */
    uint8_t             LEDtmrflash_1;  /**< single flash led timer */
    uint8_t             LEDtmrflash_2;  /**< double flash led timer */
    uint8_t             LEDtmrflash_3;  /**< triple flash led timer */
    uint8_t             LEDtmrflash_4;  /**< quadruple flash led timer */
    uint8_t             LEDred;         /**< red led #M_CO_LED_BITFIELD_t */
    uint8_t             LEDgreen;       /**< green led #M_CO_LED_BITFIELD_t */
} M_CO_LEDs_t;

/**
 * @brief 
 * 
 * Initialize LEDs object.
 * Function must be called in the system reset section.
 * 
 * @param LEDs This object will be initialized.
 * @return #M_CO_ReturnError_t#  CO_ERROR_NO or CO_ERROR_ILLEGAL_ARGUMENT.
 */
M_CO_ReturnError_t M_CO_LEDs_init(M_CO_LEDs_t *LEDs);





/**
 * @brief 
 * 
 * Process indicator states
 * Function must be called cyclically.
 * 
 * @param LEDs This object.
 * @param timeDifference_us Time difference from previous function call in [microseconds].
 * @param SMTstate SMT operating state.
 * @param SYSconfig System is in configuration state indication.
 * @param ErrPriority1 System Err indication (highest priority).
 * @param ErrPriority6 System Err indication (priority 6 ).
 * @param ErrPriority3 System Err indication (priority 3 ).
 * @param ErrPriority4 System Err indication (priority 4 ).
 * @param ErrPriority5 System Err indication (priority 5 ).
 * @param ErrOther System Err indication (lowest priority).
 * @param SYSotherState 
 */

void M_CO_LEDs_process(M_CO_LEDs_t *LEDs,
                     uint32_t timeDifference_us,
                     M_CO_SMT_internalState_t SMTstate,
                     bool_t SYSconfig,
                     bool_t ErrPriority1,
                     bool_t ErrPriority6,
                     bool_t ErrPriority3,
                     bool_t ErrPriority4,
                     bool_t ErrPriority5,
                     bool_t ErrOther,
                     bool_t SYSotherState
                     );

                     

#endif // M_CO_LEDS_H
