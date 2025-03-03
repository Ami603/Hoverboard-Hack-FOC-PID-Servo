/*
* This file is part of the hoverboard-firmware-hack project.
*
* Copyright (C) 2017-2018 Rene Hopf <renehopf@mac.com>
* Copyright (C) 2017-2018 Nico Stute <crinq@crinq.de>
* Copyright (C) 2017-2018 Niklas Fauth <niklas.fauth@kit.fail>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// Define to prevent recursive inclusion
#ifndef DEFINES_H
#define DEFINES_H

#include "stm32f1xx_hal.h"
#include "config.h"

#define LEFT_HALL_U_PIN GPIO_PIN_5
#define LEFT_HALL_V_PIN GPIO_PIN_6
#define LEFT_HALL_W_PIN GPIO_PIN_7

#define LEFT_HALL_U_PORT GPIOB
#define LEFT_HALL_V_PORT GPIOB
#define LEFT_HALL_W_PORT GPIOB

#define RIGHT_HALL_U_PIN GPIO_PIN_10
#define RIGHT_HALL_V_PIN GPIO_PIN_11
#define RIGHT_HALL_W_PIN GPIO_PIN_12

#define RIGHT_HALL_U_PORT GPIOC
#define RIGHT_HALL_V_PORT GPIOC
#define RIGHT_HALL_W_PORT GPIOC

#define LEFT_TIM TIM8
#define LEFT_TIM_U CCR1
#define LEFT_TIM_UH_PIN GPIO_PIN_6
#define LEFT_TIM_UH_PORT GPIOC
#define LEFT_TIM_UL_PIN GPIO_PIN_7
#define LEFT_TIM_UL_PORT GPIOA
#define LEFT_TIM_V CCR2
#define LEFT_TIM_VH_PIN GPIO_PIN_7
#define LEFT_TIM_VH_PORT GPIOC
#define LEFT_TIM_VL_PIN GPIO_PIN_0
#define LEFT_TIM_VL_PORT GPIOB
#define LEFT_TIM_W CCR3
#define LEFT_TIM_WH_PIN GPIO_PIN_8
#define LEFT_TIM_WH_PORT GPIOC
#define LEFT_TIM_WL_PIN GPIO_PIN_1
#define LEFT_TIM_WL_PORT GPIOB

#define RIGHT_TIM TIM1
#define RIGHT_TIM_U CCR1
#define RIGHT_TIM_UH_PIN GPIO_PIN_8
#define RIGHT_TIM_UH_PORT GPIOA
#define RIGHT_TIM_UL_PIN GPIO_PIN_13
#define RIGHT_TIM_UL_PORT GPIOB
#define RIGHT_TIM_V CCR2
#define RIGHT_TIM_VH_PIN GPIO_PIN_9
#define RIGHT_TIM_VH_PORT GPIOA
#define RIGHT_TIM_VL_PIN GPIO_PIN_14
#define RIGHT_TIM_VL_PORT GPIOB
#define RIGHT_TIM_W CCR3
#define RIGHT_TIM_WH_PIN GPIO_PIN_10
#define RIGHT_TIM_WH_PORT GPIOA
#define RIGHT_TIM_WL_PIN GPIO_PIN_15
#define RIGHT_TIM_WL_PORT GPIOB

// #define LEFT_DC_CUR_ADC ADC1
// #define LEFT_U_CUR_ADC ADC1
// #define LEFT_V_CUR_ADC ADC1

#define LEFT_DC_CUR_PIN GPIO_PIN_0
#define LEFT_U_CUR_PIN GPIO_PIN_0
#define LEFT_V_CUR_PIN GPIO_PIN_3

#define LEFT_DC_CUR_PORT GPIOC
#define LEFT_U_CUR_PORT GPIOA
#define LEFT_V_CUR_PORT GPIOC

// #define RIGHT_DC_CUR_ADC ADC2
// #define RIGHT_U_CUR_ADC ADC2
// #define RIGHT_V_CUR_ADC ADC2

#define RIGHT_DC_CUR_PIN GPIO_PIN_1
#define RIGHT_U_CUR_PIN GPIO_PIN_4
#define RIGHT_V_CUR_PIN GPIO_PIN_5

#define RIGHT_DC_CUR_PORT GPIOC
#define RIGHT_U_CUR_PORT GPIOC
#define RIGHT_V_CUR_PORT GPIOC

// #define DCLINK_ADC ADC3
// #define DCLINK_CHANNEL
//#define DCLINK_PIN GPIO_PIN_2    //this i changed pin 1
//#define DCLINK_PORT GPIOC        //this is changed to A

#define DCLINK_PIN GPIO_PIN_1    //v_batt_meas   was on C2
#define DCLINK_PORT GPIOA



// #define DCLINK_PULLUP 30000
// #define DCLINK_PULLDOWN 1000

#define LED_PIN GPIO_PIN_2
#define LED_PORT GPIOB

#define BUZZER_PIN GPIO_PIN_4
#define BUZZER_PORT GPIOA

//#define BUZZER_PIN GPIO_PIN_13  //was A4
//#define BUZZER_PORT GPIOC


#define SWITCH_PIN GPIO_PIN_1
#define SWITCH_PORT GPIOA
//#define SWITCH_PIN GPIO_PIN_9  // was A1 also called AIN_V_BATT
//#define SWITCH_PORT GPIOB

#define OFF_PIN GPIO_PIN_5
#define OFF_PORT GPIOA
//#define OFF_PIN GPIO_PIN_15   //was A5 aksi called SUPPLY_EN_MCU on diagram
//#define OFF_PORT GPIOC

#define BUTTON_PIN GPIO_PIN_1
#define BUTTON_PORT GPIOA
//#define BUTTON_PIN GPIO_PIN_9  // thiss is the same as the switch pin now
//#define BUTTON_PORT GPIOB

#define CHARGER_PIN GPIO_PIN_12
#define CHARGER_PORT GPIOA
//CHARGER PIN IS SET TO PULLUP LATER IN THIS FILE
//#define CHARGER_PIN GPIO_PIN_11// was pin A12  
//#define CHARGER_PORT GPIOA


#if defined(SUPPORT_BUTTONS_LEFT)
#define BUTTON1_PIN         GPIO_PIN_2
#define BUTTON1_PORT        GPIOA
#define BUTTON2_PIN         GPIO_PIN_3
#define BUTTON2_PORT        GPIOA
#elif defined(SUPPORT_BUTTONS_RIGHT)
#define BUTTON1_PIN         GPIO_PIN_10
#define BUTTON1_PORT        GPIOB
#define BUTTON2_PIN         GPIO_PIN_11
#define BUTTON2_PORT        GPIOB
#endif

#define DELAY_TIM_FREQUENCY_US 1000000

#define MILLI_R (R * 1000)
#define MILLI_PSI (PSI * 1000)
#define MILLI_V (V * 1000)

#define NO 0
#define YES 1
#define ABS(a) (((a) < 0) ? -(a) : (a))
#define LIMIT(x, lowhigh) (((x) > (lowhigh)) ? (lowhigh) : (((x) < (-lowhigh)) ? (-lowhigh) : (x)))
#define SAT(x, lowhigh) (((x) > (lowhigh)) ? (1.0f) : (((x) < (-lowhigh)) ? (-1.0f) : (0.0f)))
#define SAT2(x, low, high) (((x) > (high)) ? (1.0f) : (((x) < (low)) ? (-1.0f) : (0.0f)))
#define STEP(from, to, step) (((from) < (to)) ? (MIN((from) + (step), (to))) : (MAX((from) - (step), (to))))
#define DEG(a) ((a)*M_PI / 180.0f)
#define RAD(a) ((a)*180.0f / M_PI)
#define SIGN(a) (((a) < 0) ? (-1) : (((a) > 0) ? (1) : (0)))
#define CLAMP(x, low, high) (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))
#define IN_RANGE(x, low, high) (((x) >= (low)) && ((x) <= (high)))
#define SCALE(value, high, max) MIN(MAX(((max) - (value)) / ((max) - (high)), 0.0f), 1.0f)
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN3(a, b, c) MIN(a, MIN(b, c))
#define MAX3(a, b, c) MAX(a, MAX(b, c))
#define ARRAY_LEN(x) (uint32_t)(sizeof(x) / sizeof(*(x)))
#define MAP(x, in_min, in_max, out_min, out_max) (((((x) - (in_min)) * ((out_max) - (out_min))) / ((in_max) - (in_min))) + (out_min))

#if defined(PRINTF_FLOAT_SUPPORT) && (defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)) && defined(__GNUC__)
    asm(".global _printf_float");     // this is the magic trick for printf to support float. Warning: It will increase code considerably! Better to avoid!
#endif


typedef struct {
  uint16_t dcr; 
  uint16_t dcl; 
  uint16_t rlA;
  uint16_t rlB;
  uint16_t rrB;
  uint16_t rrC;
  uint16_t batt1;
  uint16_t l_tx2;
  uint16_t temp;
  uint16_t l_rx2;
} adc_buf_t;

// Sideboard definitions
#define LED1_SET            (0x01)
#define LED2_SET            (0x02)
#define LED3_SET            (0x04)
#define LED4_SET            (0x08)
#define LED5_SET            (0x10)
#define SENSOR1_SET         (0x01)
#define SENSOR2_SET         (0x02)
#define SENSOR_MPU          (0x04)

// RC iBUS switch definitions. Flysky FS-i6S has [SW1, SW2, SW3, SW4] = [2, 3, 3, 2] positions switch
#define SW1_SET             (0x0100)   //  0000 0001 0000 0000
#define SW2_SET             (0x0600)   //  0000 0110 0000 0000
#define SW3_SET             (0x1800)   //  0001 1000 0000 0000
#define SW4_SET             (0x2000)   //  0010 0000 0000 0000


#endif // DEFINES_H

