#include "i2c.h"

static const I2C_PinMap_t I2C_PinMap[] = {
    {GPIOB, PIN_8, GPIOB, PIN_9},   // I2C1
    {GPIOB, PIN_10, GPIOB, PIN_11}, // I2C2
    {GPIOA, PIN_8, GPIOC, PIN_9}    // I2C3
};

Status_t I2C_enable(I2C_TypeDef* I2Cx) {
    if (I2Cx == I2C1) {
        RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    } else if (I2Cx == I2C2) {
        RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;

    } else if (I2Cx == I2C3) {
        RCC->APB1ENR |= RCC_APB1ENR_I2C3EN;

    } else {
        return STATUS_INVALID;
    }
    return STATUS_OK;
}

Status_t I2C_gpio_config(I2C_TypeDef* I2Cx) {
    uint8_t map_index;

    if (I2Cx == I2C1) {
        map_index = I2C1_PIN_MAP_IDX;
    } else if (I2Cx == I2C2) {
        map_index = I2C2_PIN_MAP_IDX;
    } else if (I2Cx == I2C3) {
        map_index = I2C3_PIN_MAP_IDX;
    } else {
        return STATUS_INVALID;
    }

    const I2C_PinMap_t* map = &I2C_PinMap[map_index];

    if (GPIO_enable(map->scl_port) != STATUS_OK || GPIO_enable(map->sda_port) != STATUS_OK) {
        return STATUS_INVALID;
    }

    // configure scl line
    if (GPIO_mode_configure(map->scl_port, GPIO_MODE_AF, map->scl_pin) != STATUS_OK) {
        return STATUS_INVALID;
    }
    if (GPIO_config_alternate(map->scl_port, map->scl_pin, GPIO_AF4) != STATUS_OK) {
        return STATUS_INVALID;
    }

    // configure sda line
    if (GPIO_mode_configure(map->sda_port, GPIO_MODE_AF, map->sda_pin) != STATUS_OK) {
        return STATUS_INVALID;
    }
    if (GPIO_config_alternate(map->sda_port, map->sda_pin, GPIO_AF4) != STATUS_OK) {
        return STATUS_INVALID;
    }
    return STATUS_OK;
}

/* TO DO: ADD SYSTICK TIMER FOR BETTER TIMEOUT COUNTING*/
Status_t I2C_software_reset(I2C_TypeDef* I2Cx) {
    // enable software reset
    I2Cx->CR1 |= I2C_CR1_SWRST_Msk;

    PinState_t scl_state, sda_state;
    Status_t   status;
    uint32_t   timeout = I2C_BUS_FREE_TIMEOUT;
    uint8_t    map_index;

    if (I2Cx == I2C1) {
        map_index = I2C1_PIN_MAP_IDX;
    } else if (I2Cx == I2C2) {
        map_index = I2C2_PIN_MAP_IDX;
    } else if (I2Cx == I2C3) {
        map_index = I2C3_PIN_MAP_IDX;
    } else {
        return STATUS_INVALID;
    }

    const I2C_PinMap_t* map = &I2C_PinMap[map_index];
    // make sure bus is clear before disabling software reset
    while (timeout > 0U) {
        status = STATUS_OK;
        status |= GPIO_pin_read(map->scl_port, map->scl_pin, &scl_state);
        status |= GPIO_pin_read(map->sda_port, map->sda_pin, &sda_state);
        if (status != STATUS_OK) {
            return STATUS_ERROR;
        } else if (scl_state == PIN_SET && sda_state == PIN_SET) {
            // bus is clear, reset SWRST bit
            I2Cx->CR1 &= ~I2C_CR1_SWRST_Msk;
            return STATUS_OK;
        }
        timeout--;
    }
    return STATUS_TIMEOUT;
}

Status_t I2C_init(I2C_TypeDef* I2Cx, I2C_config_t* cfg) {
    /*  1. Reset I2C
     *  2.  Enable peripheral clock in RCC.
     *  3. Set peripheral input clock frequency
     *  4. Configure speed mode & timing
     *  5. Set addressing mode
     *  6. Configure ACK
     *  7. Enable I2C module
     */

    // Software reset I2C
    if (I2C_software_reset(I2Cx) != STATUS_OK) {
        return STATUS_ERROR;
    }

    // Enable peripheral
    if (I2C_enable(I2Cx) != STATUS_OK) {
        return STATUS_ERROR;
    }

    // Set peripheral input clock frequency
    uint32_t periph_clk;
    if (RCC_get_periph_clk(I2Cx, &periph_clk) != STATUS_OK) {
        return STATUS_ERROR;
    }
    // recieved periph_clk is in hz so convert to mhz, minmum allowed is 2 Mhz
    periph_clk /= 1000000;
    I2Cx->CR2 |= (periph_clk >= 2U) ? periph_clk : 2U;
    return STATUS_OK;
}

static Status_t I2C_calc_ccr(uint32_t pclk, uint32_t speed_mode, uint32_t duty, uint32_t* ccr_val) {
    /* Standard mode - time for each part (low:high) is equal in one full cycle, therefor
     * T_scl = T_high + T_low = CCR * T_pclk + CCR * T_pclk -> CCR = T_scl / (2 * T_pclk)
     * -> CCR = T_scl / (2 * T_pclk) -> CCR = F_pclk / (2 * F_scl)
     *
     * Fast mode, DUTY = 0 - low time is twice the high time (ratio 2:1), therefore
     * T_scl = T_high + 2 * T_low = CCR * T_pclk + 2 * CCR * T_pclk = 3 * CCR * T_pclk
     * -> CCR = T_scl / (3 * T_pclk) -> CCR = F_pclk / (3 * F_scl)
     *
     Fm mode, DUTY = 1 - time for each part is 9:16 for T_high and T_low respectively, therefore
     * T_scl = 9 * T_high + 16 * T_low = 9 * CCR * T_pclk + 16 * CCR * T_pclk = 25 * CCR * T_pclk
     * -> CCR = T_scl / (25 * T_pclk) -> CCR = F_pclk / (25 * F_scl)
     *
     * Note: T_pclk = 1 / F_pclk , T_scl = 1 / F_scl = 1 / speed_mode_value
     */
    if (ccr_val == NULL) {
        return STATUS_INVALID;
    }
    if (speed_mode == I2C_SPEED_STD) {
        *ccr_val = pclk / (2U * speed_mode);
    } else if (speed_mode == I2C_SPEED_FAST && pclk >= I2C_MIN_FAST_PERIPH_CLK_HZ) {
        if (duty == I2C_DUTY_CYCLE_STD) {
            *ccr_val = pclk / (3U * speed_mode);
        } else if (duty == I2C_DUTY_CYCLE_FAST) {
            *ccr_val = pclk / (25U * speed_mode);
        } else {
            return STATUS_INVALID;
        }
    } else { // speed mode unkown or PCLK not sufficent
        return STATUS_INVALID;
    }

    // Enforce hardware minimum CCR value
    if (*ccr_val < I2C_MIN_CCR_VALUE) {
        *ccr_val = I2C_MIN_CCR_VALUE;
    }
    return STATUS_OK;
}

Status_t I2C_config_clock(I2C_TypeDef* I2Cx, uint32_t speed_mode, uint32_t duty_cycle) {
    /* Standard mode - time for each part (low:high) is equal in one full cycle, therefor
     * T_scl = T_high + T_low = CCR * T_pclk + CCR * T_pclk -> CCR = T_scl / (2 * T_pclk)
     * -> CCR = T_scl / (2 * T_pclk) -> CCR = F_pclk / (2 * F_scl)
     *
     * Fast mode, DUTY = 0 - low time is twice the high time (ratio 2:1), therefore
     * T_scl = T_high + 2 * T_low = CCR * T_pclk + 2 * CCR * T_pclk = 3 * CCR * T_pclk
     * -> CCR = T_scl / (3 * T_pclk) -> CCR = F_pclk / (3 * F_scl)
     *
     Fm mode, DUTY = 1 - time for each part is 9:16 for T_high and T_low respectively, therefore
     * T_scl = 9 * T_high + 16 * T_low = 9 * CCR * T_pclk + 16 * CCR * T_pclk = 25 * CCR * T_pclk
     * -> CCR = T_scl / (25 * T_pclk) -> CCR = F_pclk / (25 * F_scl)
     *
     * Note: T_pclk = 1 / F_pclk , T_scl = 1 / F_scl = 1 / speed_mode_value
     */

    // disable I2C
    I2Cx->CR1 &= ~I2C_CR1_PE_Msk;

    uint32_t periph_clk;
    if (RCC_get_periph_clk(I2Cx, &periph_clk) != STATUS_OK) {
        return STATUS_ERROR;
    }
    if (periph_clk < I2C_MIN_STD_PERIPH_CLK_HZ) { // min speed for I2C
        return STATUS_ERROR;
    }

    I2Cx->CR2 &= ~I2C_CR2_FREQ_Msk;
    I2Cx->CR2 |= HZ_TO_MHZ(periph_clk) & I2C_CR2_FREQ_Msk; // set FREQ[5:0]

    uint32_t ccr_val;
    if (I2C_calc_ccr(periph_clk, speed_mode, duty_cycle, &ccr_val) != STATUS_OK) {
        return STATUS_ERROR;
    }
    return STATUS_OK;
}

// static uint32_t I2C_calc_trise(uint32_t pclk, uint32_t speed) {}
