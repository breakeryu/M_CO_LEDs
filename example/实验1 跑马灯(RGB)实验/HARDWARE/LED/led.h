/*
 * @Author: magyu
 * @Date: 2023-03-11 19:01:07
 * @LastEditors: magyu
 * @LastEditTime: 2023-03-12 12:34:54
 * @Description: ����д���
 */
#ifndef _LED_H
#define _LED_H
#include "sys.h"

#include "M_CO_driver_target.h"
#include "M_CO_LEDs.h"
#include "M_CO_LEDs_System.h"
#include "debugLEDs.h"
#include "M_CO_driver.h"
#include "delay.h"
/*********************************************************************************
			  ___   _     _____  _____  _   _  _____  _____  _   __
			 / _ \ | |   |_   _||  ___|| \ | ||_   _||  ___|| | / /
			/ /_\ \| |     | |  | |__  |  \| |  | |  | |__  | |/ /
			|  _  || |     | |  |  __| | . ` |  | |  |  __| |    \
			| | | || |_____| |_ | |___ | |\  |  | |  | |___ | |\  \
			\_| |_/\_____/\___/ \____/ \_| \_/  \_/  \____/ \_| \_/

 *	******************************************************************************
 *	������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 *	ALIENTEK Pandora STM32L475 IOT������
 *	LED��������
 *	����ԭ��@ALIENTEK
 *	������̳:www.openedv.com
 *	��������:2018/10/27
 *	�汾��V1.0
 *	��Ȩ���У�����ؾ���
 *	Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 *	All rights reserved
 *	******************************************************************************
 *	��ʼ�汾
 *	******************************************************************************/

//RGB�ӿڶ���
#define LED_R(n)			(n?HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_RESET):HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_SET))
#define LED_R_TogglePin		HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_7)	//LED_R��ƽ��ת

#define LED_G(n)			(n?HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_RESET):HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_SET))
#define LED_G_TogglePin     HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_8)	//LED_G��ƽ��ת

#define LED_B(n)			(n?HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_RESET))
#define LED_B_TogglePin     HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_9)	//LED_B��ƽ��ת

void LED_Init(void);
void m_led_Init_All(void);
void m_led_Process(void);




#endif




