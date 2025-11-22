#ifndef I2C_H
#define I2C_H

#include "board.h"
#include "gpio.h"
#include "status.h"
#include "stm32f4xx.h"
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

#define I2C_BUS_FREE_TIMEOUT 10000U
#define I2C_MIN_STD_PERIPH_CLK_HZ 2000000U
#define I2C_MIN_FAST_PERIPH_CLK_HZ 4000000U
#define HZ_TO_MHZ(hz_value) ((hz_value) / 1000000U)
#define I2C_MIN_CCR_VALUE 4U
#define I2C_STD_MAX_RISE_TIME_NS 1000U // 1000ns = 1us
#define I2C_FAST_MAX_RISE_TIME_NS 300U // 300ns

typedef struct {
    GPIO_TypeDef* scl_port;
    uint32_t      scl_pin;
    GPIO_TypeDef* sda_port;
    uint32_t      sda_pin;
} I2C_PinMap_t;

typedef enum {
    I2C1_PIN_MAP_IDX = 0U,
    I2C2_PIN_MAP_IDX = 1U,
    I2C3_PIN_MAP_IDX = 2U
} I2CX_PIN_MAP_IDX;

typedef enum {
    I2C_SPEED_STD  = 100000U,
    I2C_SPEED_FAST = 400000U,
} I2C_SPEED_MODE_HZ;

typedef enum {
    I2C_DUTY_CYCLE_STD  = 0U,
    I2C_DUTY_CYCLE_FAST = 1U,
} I2C_DUTY_CYCLE;

typedef enum {
    I2C_WRITE = 0U,
    I2C_READ  = 1U,
} I2C_direction_t;

typedef enum { FLAG_CLEAR = 0U, FLAG_SET = 1U } FlagStatus_t;

typedef enum {
    I2C_SR1 = 0U,
    I2C_SR2 = 1U,
} StatusRegister_t;
/* Currently supports only master mode 7 bit address */
typedef struct {
    uint32_t speed_mode; // I2C speed in Hz (100000 = standard, 400000 = fast)
    uint32_t duty_cycle; // 0 = standard (2:1), 1 = fast mode (16:9)
} I2C_config_t;

typedef enum {
    I2C_INIT_OK            = 0x00,
    I2C_INIT_RCC_FAIL      = 0x01,
    I2C_INIT_GPIO_FAIL     = 0x02,
    I2C_INIT_RESET_FAIL    = 0x03,
    I2C_INIT_CLOCK_FAIL    = 0x04
} I2C_InitError_t;

/*-------------------------------- Functions ----------------------------*/


#ifdef __cplusplus
}
#endif

#endif // I2C_H
