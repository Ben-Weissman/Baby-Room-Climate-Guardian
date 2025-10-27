#ifndef I2C_H
#define I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "board.h"
#include "gpio.h"
#include "status.h"
#include "stm32f4xx.h"
#include <stdio.h>

#define I2C_BUS_FREE_TIMEOUT 10000U
#define I2C_MIN_STD_PERIPH_CLK_HZ 2000000U
#define I2C_MIN_FAST_PERIPH_CLK_HZ 4000000U
#define HZ_TO_MHZ(hz_value) ((hz_value) / 1000000U)
#define I2C_MIN_CCR_VALUE 4U

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

typedef struct {
    uint32_t clock_speed;     // I2C speed in Hz (100000 = standard, 400000 = fast)
    uint32_t duty_cycle;      // 0 = standard (Tlow/Thigh = 2), 1 = fast mode (16/9)
    uint8_t  own_address;     // For slave mode (0 if unused)
    uint8_t  addressing_mode; // 0 = 7-bit, 1 = 10-bit
    uint8_t  general_call;    // Enable/disable general call address
    uint8_t  clock_stretch;   // Allow slaves to hold SCL low
    uint8_t  dual_address;    // Enable second own address
    uint8_t  second_address;  // Optional second address
} I2C_config_t;

#ifdef __cplusplus
}
#endif

#endif // I2C_H
