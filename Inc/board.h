#ifndef BOARD_H
#define BOARD_H

#include "stm32f4xx.h"

/* This is a specific board constants, you need to change these
 *  for each individual board. In this case we use the STM Nucleo_F401RE board
 */

#define HSI_VALUE 16000000U
#define HSE_VALUE 8000000U

typedef enum {
    PIN_LED      = 5,  // on-board led pin
    USR_PUSH_BTN = 13, // user push button pin (blue btn)
    PIN_0        = 0,
    PIN_1        = 1,
    PIN_2        = 2,
    PIN_3        = 3,
    PIN_4        = 4,
    PIN_5        = 5,
    PIN_6        = 6,
    PIN_7        = 7,
    PIN_8        = 8,
    PIN_9        = 9,
    PIN_10       = 10,
    PIN_11       = 11,
    PIN_12       = 12,
    PIN_13       = 13,
    PIN_14       = 14,
    PIN_15       = 15,
} PIN_NUMBERS;

typedef enum {
    GPIO_MODE_INPUT  = 0U,
    GPIO_MODE_OUTPUT = 1U,
    GPIO_MODE_AF     = 2U,
    GPIO_MODE_ANALOG = 3U,
} GPIO_MODES;

/**
 * @brief Alternate function selection values for GPIO AFR registers.
 * Each value corresponds to the 4-bit field written into AFRL/AFRH.
 */
typedef enum {
    GPIO_AF0  = 0x0, /**< AF0: System functions (RTC, MCO, etc.) */
    GPIO_AF1  = 0x1, /**< AF1 */
    GPIO_AF2  = 0x2, /**< AF2 */
    GPIO_AF3  = 0x3, /**< AF3 */
    GPIO_AF4  = 0x4, /**< AF4: Typically I2C */
    GPIO_AF5  = 0x5, /**< AF5: Typically SPI */
    GPIO_AF6  = 0x6, /**< AF6 */
    GPIO_AF7  = 0x7, /**< AF7: Typically USART1/2/3 */
    GPIO_AF8  = 0x8, /**< AF8: Typically USART4/5/6 */
    GPIO_AF9  = 0x9, /**< AF9 */
    GPIO_AF10 = 0xA, /**< AF10: OTG_FS / SDIO */
    GPIO_AF11 = 0xB, /**< AF11: Ethernet */
    GPIO_AF12 = 0xC, /**< AF12: FSMC/SDIO */
    GPIO_AF13 = 0xD, /**< AF13: DCMI */
    GPIO_AF14 = 0xE, /**< AF14 */
    GPIO_AF15 = 0xF  /**< AF15: EVENTOUT */
} GPIO_AltFunc_t;

typedef enum {
    GPIO_OUTPUT_PUSHPULL  = 0U, // default, reset state
    GPIO_OUTPUT_OPENDRAIN = 1U
} GPIO_OutputType_t;

typedef enum { GPIO_NO_PULL = 0x0U, GPIO_PULLUP = 0x1U, GPIO_PULLDOWN = 0x2U } GPIO_PullMode_t;

#endif // BOARD_H
