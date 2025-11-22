#include "i2c.h"
#include "rcc.h"
#include <stdio.h>

// *-------------------------------- I2C Pin Mapping ----------------------------*/
static const I2C_PinMap_t I2C_PinMap[] = {
    {GPIOB, PIN_8, GPIOB, PIN_9},   // I2C1
    {GPIOB, PIN_10, GPIOB, PIN_11}, // I2C2
    {GPIOA, PIN_8, GPIOC, PIN_9}    // I2C3
};

// ---------------------------- I2C Internal Function Prototypes ----------------------------*/
static Status_t I2C_enable_rcc(I2C_TypeDef* I2Cx);
static Status_t I2C_gpio_config(I2C_TypeDef* I2Cx);
static Status_t I2C_software_reset(I2C_TypeDef* I2Cx);
static Status_t I2C_config_clock(I2C_TypeDef* I2Cx, uint32_t speed_mode, uint32_t duty_cycle);

static Status_t    I2C_wait_flag(I2C_TypeDef* I2Cx, StatusRegister_t reg_sel, uint32_t flag_mask,
                                 FlagStatus_t desired_state);
static Status_t    I2C_wait_flag_recover(I2C_TypeDef* I2Cx, StatusRegister_t reg_sel,
                                         uint32_t flag_mask, FlagStatus_t desired_state);
static Status_t    I2C_recover(I2C_TypeDef* I2Cx);
static void        I2C_clear_addr_flag(I2C_TypeDef* I2Cx);
static inline void I2C_enable_ack(I2C_TypeDef* I2Cx);
static inline void I2C_disable_ack(I2C_TypeDef* I2Cx);
static inline void I2C_generate_stop(I2C_TypeDef* I2Cx);

static Status_t I2C_calc_ccr(uint32_t pclk, uint32_t speed_mode, uint32_t duty, uint32_t* ccr_val);
static Status_t I2C_calc_trise(uint32_t pclk, uint32_t speed_mode, uint32_t* trise_val);
static Status_t I2C_start(I2C_TypeDef* I2Cx);
static Status_t I2C_send_address(I2C_TypeDef* I2Cx, uint8_t address, I2C_direction_t direction);
static inline void I2C_enable(I2C_TypeDef* I2Cx);
static inline void I2C_disable(I2C_TypeDef* I2Cx);

// *---------------------------- I2C Public Functions ----------------------------*/

I2C_InitError_t I2C_init(I2C_TypeDef* I2Cx, I2C_config_t* cfg) {
    if (cfg == NULL) {
        return I2C_INIT_CFG_FAIL;
    }

    // Enable I2C clock via RCC
    if (I2C_enable_rcc(I2Cx) != STATUS_OK) {
        return I2C_INIT_RCC_FAIL;
    }

    // Configure GPIO
    if (I2C_gpio_config(I2Cx) != STATUS_OK) {
        return I2C_INIT_GPIO_FAIL;
    }

    // Perform I2C reset
    if (I2C_software_reset(I2Cx) != STATUS_OK) {
        return I2C_INIT_RESET_FAIL;
    }

    // Configure timing
    if (I2C_config_clock(I2Cx, cfg->speed_mode, cfg->duty_cycle) != STATUS_OK) {
        return I2C_INIT_CLOCK_FAIL;
    }
    return I2C_INIT_OK;
}

Status_t I2C_master_write_byte(I2C_TypeDef* I2Cx, uint8_t data) {
    // Wait until data register is empty (TXE=1) or previous byte finished (BTF=1)
    Status_t status = I2C_wait_flag_recover(I2Cx, I2C_SR1, I2C_SR1_TXE | I2C_SR1_BTF, FLAG_SET);
    if (status != STATUS_OK) {
        return status;
    }

    // Write new data to data register
    I2Cx->DR = data;

    return STATUS_OK;
}

Status_t I2C_master_write(I2C_TypeDef* I2Cx, uint8_t slave_addr, uint8_t mem_addr,
                          const uint8_t* data, uint32_t length) {
    if (data == NULL || length == 0) {
        return STATUS_INVALID;
    }

    /* --- Step 1 ---
     * Send START condition and the slave address with write indication, then send the
     * memory address inside the slave to read from.
     */
    if (I2C_start(I2Cx) != STATUS_OK) {
        return STATUS_ERROR;
    }

    if (I2C_send_address(I2Cx, slave_addr, I2C_WRITE) != STATUS_OK) {
        return STATUS_ERROR;
    }

    if (I2C_master_write_byte(I2Cx, mem_addr) != STATUS_OK) {
        return STATUS_ERROR;
    }

    /* --- Step 2 ---
     * Write N bytes from the data buffer to the slave device.
     */
    for (uint32_t i = 0; i < length; i++) {
        if (I2C_master_write_byte(I2Cx, data[i]) != STATUS_OK) {
            return STATUS_ERROR;
        }
    }

    // STOP condition
    I2C_generate_stop(I2Cx);

    return STATUS_OK;
}

Status_t I2C_master_read(I2C_TypeDef* I2Cx, uint8_t slave_addr, uint8_t mem_addr, uint8_t* buffer,
                         uint32_t length) {
    if (buffer == NULL || length == 0) {
        return STATUS_INVALID;
    }

    /* --- Step 1 ---
     * Send START condition and the slave address with write indication, then send the
     * memory address inside the slave to read from.
     */
    if (I2C_start(I2Cx) != STATUS_OK) {
        return STATUS_ERROR;
    }

    if (I2C_send_address(I2Cx, slave_addr, I2C_WRITE) != STATUS_OK) {
        return STATUS_ERROR;
    }

    if (I2C_master_write_byte(I2Cx, mem_addr) != STATUS_OK) {
        return STATUS_ERROR;
    }

    /* --- Step 2 ---
     * Repeat START condition in order to read from the slave, this time with read indication.
     */
    if (I2C_start(I2Cx) != STATUS_OK) {
        return STATUS_ERROR;
    }

    if (I2C_send_address(I2Cx, slave_addr, I2C_READ) != STATUS_OK) {
        return STATUS_ERROR;
    }

    /* --- Step 3 ---
     * Enable ACK and read N bytes from the slave, store the data into the provided buffer.
     */
    I2C_enable_ack(I2Cx);

    for (uint32_t i = 0; i < length; i++) {
        // If last byte then NACK and generate STOP
        if (i == (length - 1)) {
            I2C_disable_ack(I2Cx); // Acts as NACK for last byte
            I2C_generate_stop(I2Cx);
        }

        // Wait for RXNE flag
        if (I2C_wait_flag_recover(I2Cx, I2C_SR1, I2C_SR1_RXNE, FLAG_SET) != STATUS_OK) {
            return STATUS_ERROR;
        }
        // Insert received byte into buffer
        buffer[i] = (uint8_t) I2Cx->DR;
    }

    return STATUS_OK;
}

// *---------------------------- I2C Internal Functions ----------------------------*/
/**
 * @brief  Clear the ADDR flag after an address acknowledge.
 *
 * Reads SR1 followed by SR2 to release the I2C peripheral from the
 * address phase, as required by the STM32 hardware design.
 *
 * @param[in] I2Cx  Pointer to the I2C peripheral instance.
 *
 * @note This operation is mandatory before any data phase begins.
 */

static void I2C_clear_addr_flag(I2C_TypeDef* I2Cx) {
    // Clear ADDR flag by reading SR1 and SR2 (required by hardware)
    (void) I2Cx->SR1;
    (void) I2Cx->SR2;
}

/**
 * @brief  Enable I2C acknowledge.
 *
 * Sets the ACK bit, allowing the peripheral to acknowledge received bytes.
 *
 * @param[in] I2Cx  Pointer to the I2C peripheral instance.
 */

static inline void I2C_enable_ack(I2C_TypeDef* I2Cx) {
    I2Cx->CR1 |= I2C_CR1_ACK;
}

/**
 * @brief  Disable I2C acknowledge.
 *
 * Clears the ACK bit, causing the next received byte to be NACKed.
 * Required when reading the final byte in a multi-byte sequence.
 *
 * @param[in] I2Cx  Pointer to the I2C peripheral instance.
 */

static inline void I2C_disable_ack(I2C_TypeDef* I2Cx) {
    I2Cx->CR1 &= ~I2C_CR1_ACK;
}

/**
 * @brief  Generate a STOP condition on the I2C bus.
 *
 * Sets the STOP bit in CR1 to release the bus after the current
 * communication finishes.
 *
 * @param[in] I2Cx  Pointer to the I2C peripheral instance.
 */

static inline void I2C_generate_stop(I2C_TypeDef* I2Cx) {
    I2Cx->CR1 |= I2C_CR1_STOP;
}

/**
 * @brief  Wait for an I2C status flag with automatic recovery on timeout.
 *
 * Calls @ref I2C_wait_flag() to poll a flag in either SR1 or SR2.
 * If a timeout occurs, the function executes the @ref I2C_recover() procedure.
 *
 *
 * @param[in] I2Cx           Pointer to the I2C peripheral instance.
 * @param[in] flag_mask      Mask of the flag bit to evaluate.
 * @param[in] reg_sel        Status register to check (I2C_SR1 or I2C_SR2).
 * @param[in] desired_state  Expected state of the flag (FLAG_SET or FLAG_CLEAR).
 *
 * @retval STATUS_OK         Flag reached desired state, or timeout occurred but
 *                           recovery completed successfully.
 * @retval STATUS_INVALID    Invalid desired_state parameter.
 *
 * @note This wrapper provides robustness for higher-level functions such as START,
 *       address sending, or multi-byte reads, without complicating their logic.
 */
static Status_t I2C_wait_flag_recover(I2C_TypeDef* I2Cx, StatusRegister_t reg_sel,
                                      uint32_t flag_mask, FlagStatus_t desired_state) {
    Status_t status = I2C_wait_flag(I2Cx, flag_mask, reg_sel, desired_state);
    if (status == STATUS_TIMEOUT) {
        status = I2C_recover(I2Cx);
        return (status == STATUS_OK) ? STATUS_OK : status;
    }
    return status;
}

/**
 * @brief  Wait for an I2C status flag in SR1 or SR2 to reach the desired state.
 *
 * Polls either SR1 or SR2 (selected via @p reg_sel) until the specified
 * flag reaches the expected state (set or cleared), or until a timeout occurs.
 *
 * @param[in] I2Cx           Pointer to the I2C peripheral instance.
 * @param[in] reg_sel        Selects the status register to check:
 *                           - I2C_SR1 → check SR1
 *                           - I2C_SR2 → check SR2
 * @param[in] flag_mask      Mask of the flag bit to evaluate in the chosen register.
 * @param[in] desired_state  Expected state of the flag (FLAG_SET or FLAG_CLEAR).
 *
 * @retval STATUS_OK         Flag reached the expected state within timeout.
 * @retval STATUS_TIMEOUT    Timeout expired before flag reached desired state.
 * @retval STATUS_INVALID    Invalid desired_state parameter.
 *
 * @note This function performs no recovery. It is a low-level building block
 *       used by higher-level operations such as START, address sending,
 *       or byte reception.
 */
static Status_t I2C_wait_flag(I2C_TypeDef* I2Cx, StatusRegister_t reg_sel, uint32_t flag_mask,
                              FlagStatus_t desired_state) {
    uint32_t           timeout = I2C_BUS_FREE_TIMEOUT;
    volatile uint32_t* reg     = NULL;

    if (desired_state != FLAG_SET && desired_state != FLAG_CLEAR) {
        return STATUS_INVALID;
    }
    // select the correct status register
    reg = (reg_sel == I2C_SR2) ? &I2Cx->SR2 : &I2Cx->SR1;

    while (timeout > 0U) {
        const uint32_t current = *reg;

        if ((desired_state == FLAG_SET && (current & flag_mask)) ||
            (desired_state == FLAG_CLEAR && !(current & flag_mask))) {
            return STATUS_OK;
        }
        timeout--;
    }
    return STATUS_TIMEOUT;
}

/**
 * @brief  Enable the peripheral clock for the specified I2C instance.
 *
 * Sets the appropriate enable bit in the APB1ENR register to provide
 * a clock signal to the selected I2C peripheral.
 *
 * @param[in] I2Cx  Pointer to the I2C peripheral instance (I2C1, I2C2, or I2C3).
 *
 * @retval STATUS_OK       Clock successfully enabled.
 * @retval STATUS_INVALID  Invalid I2C peripheral pointer.
 *
 * @note  This function is intended for internal use only and should
 *        not be called directly by user code. Use @ref I2C_init() instead.
 */
static inline Status_t I2C_enable_rcc(I2C_TypeDef* I2Cx) {
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

/**
 * @brief  Configure a single GPIO pin for I2C operation.
 *
 * Sets the given pin to Alternate Function mode (AF4), Open-Drain output,
 * and disables internal pull resistors (assuming external pull-ups are used).
 *
 * @param[in] port  GPIO port of the pin (e.g., GPIOB).
 * @param[in] pin   Pin number to configure (e.g., PIN_8).
 *
 * @retval STATUS_OK     Pin configured successfully.
 * @retval STATUS_ERROR  Pin configuration failed.
 *
 * @note  This function is intended for internal use within the I2C driver.
 */

static inline Status_t I2C_configure_pin(GPIO_TypeDef* port, PIN_NUMBERS pin) {
    // Note: I2C requires Open-Drain & Alternate Function AF4 for STM32F4, assuming external
    // resistors are present for pull-ups
    if (GPIO_mode_configure(port, GPIO_MODE_AF, pin) != STATUS_OK ||
        GPIO_config_alternate(port, pin, GPIO_AF4) != STATUS_OK ||
        GPIO_pin_output_type(port, GPIO_OUTPUT_OPENDRAIN, pin) != STATUS_OK ||
        GPIO_pin_pullup_pulldown(port, GPIO_NO_PULL, pin) != STATUS_OK) {
        return STATUS_ERROR;
    }
    return STATUS_OK;
}
/**
 * @brief  Configure GPIO pins (SCL and SDA) for the specified I2C peripheral.
 *
 * Selects the correct pin mapping based on the given I2C instance, enables
 * the corresponding GPIO ports, and configures both pins for I2C operation
 * using Alternate Function 4 (AF4), Open-Drain output type, and no internal
 * pull resistors (assuming external pull-ups are present).
 *
 * @param[in] I2Cx  Pointer to the I2C peripheral instance (I2C1, I2C2, or I2C3).
 *
 * @retval STATUS_OK       GPIO configuration succeeded.
 * @retval STATUS_ERROR    One or more pin configurations failed.
 * @retval STATUS_INVALID  Invalid I2C instance or GPIO port enable failure.
 *
 * @note  This function is automatically called during @ref I2C_init().
 *        Users should not call it directly.
 */
static Status_t I2C_gpio_config(I2C_TypeDef* I2Cx) {
    uint8_t map_index;

    // Select map according to I2C instance
    if (I2Cx == I2C1) {
        map_index = I2C1_PIN_MAP_IDX;
    } else if (I2Cx == I2C2) {
        map_index = I2C2_PIN_MAP_IDX;
    } else if (I2Cx == I2C3) {
        map_index = I2C3_PIN_MAP_IDX;
    } else {
        return STATUS_INVALID;
    }

    const I2C_PinMap_t* map    = &I2C_PinMap[map_index];
    Status_t            status = STATUS_OK;
    // Enable GPIO ports for SCL and SDA
    status = GPIO_enable(map->scl_port);
    status |= GPIO_enable(map->sda_port);
    if (status != STATUS_OK) {
        return STATUS_ERROR;
    }

    // Configure SCL line pin
    status = I2C_configure_pin(map->scl_port, map->scl_pin);

    // Configure sda line pin
    status |= I2C_configure_pin(map->sda_port, map->sda_pin);
    if (status != STATUS_OK) {
        // pin configuration failed
        return STATUS_ERROR;
    }

    return STATUS_OK;
}

/**
 * @brief  Attempt to recover the I2C peripheral after a communication timeout.
 *
 * Generates a STOP condition, checks if the bus is still busy, and performs
 * a software reset if necessary. The peripheral is then re-enabled.
 *
 * @param[in] I2Cx  Pointer to the I2C peripheral instance.
 *
 * @retval STATUS_OK  Recovery procedure completed.
 *
 * @note This function assumes recovery must always succeed and does
 *       not propagate further error status.
 */
static Status_t I2C_recover(I2C_TypeDef* I2Cx) {
    // Try generating stop condition
    I2Cx->CR1 |= I2C_CR1_STOP;

    // Short delay to propagate stop condition
    for (volatile uint32_t i = 0; i < 1000; i++)
        ;

    if (I2Cx->SR2 & I2C_SR2_BUSY) {
        // If line is till busy, perform software reset
        I2C_software_reset(I2Cx);
    }

    /* TO DO: if still busy try bus recovery */

    // Re-enable I2C peripheral and short delay
    I2Cx->CR1 |= I2C_CR1_PE;
    for (volatile uint32_t i = 0; i < 1000; i++)
        ;

    return STATUS_OK; // Recovery must be successful
}

/**
 * @brief  Retrieve the GPIO pin mapping for the specified I2C peripheral.
 *
 * Returns a pointer to the corresponding entry in the static I2C pin map table,
 * which defines the SCL and SDA pins for each I2C instance.
 *
 * @param[in] I2Cx  Pointer to the I2C peripheral instance (I2C1, I2C2, or I2C3).
 *
 * @return Pointer to the corresponding I2C_PinMap_t structure, or NULL if the
 *         specified instance is invalid.
 *
 * @note  This function is for internal use within the I2C driver.
 */
static inline const I2C_PinMap_t* I2C_get_pin_map(I2C_TypeDef* I2Cx) {
    if (I2Cx == I2C1) {
        return &I2C_PinMap[I2C1_PIN_MAP_IDX];
    } else if (I2Cx == I2C2) {
        return &I2C_PinMap[I2C2_PIN_MAP_IDX];
    } else if (I2Cx == I2C3) {
        return &I2C_PinMap[I2C3_PIN_MAP_IDX];
    } else {
        return NULL;
    }
}
/**
 * @brief  Perform a software reset of the specified I2C peripheral.
 *
 * Disables and re-enables the I2C peripheral to clear internal logic,
 * ensuring the peripheral is reset to a known good state. The function
 * also verifies that the I2C bus lines (SCL and SDA) are free before
 * returning.
 *
 * @param[in] I2Cx  Pointer to the I2C peripheral instance (I2C1, I2C2, or I2C3).
 *
 * @retval STATUS_OK       Reset completed successfully and bus is free.
 * @retval STATUS_ERROR    GPIO read failed during bus check.
 * @retval STATUS_TIMEOUT  Bus remains busy after timeout period.
 * @retval STATUS_INVALID  Invalid I2C instance pointer.
 *
 * @note  This function is intended for internal use by the I2C driver.
 *        It does not attempt full bus recovery if the lines remain busy.
 */
static Status_t I2C_software_reset(I2C_TypeDef* I2Cx) {
    // ADD SYSTICK DELAY IN THE FUTURE

    // Disable I2C peripheral and perform software reset pulse
    I2C_disable(I2Cx);
    I2Cx->CR1 |= I2C_CR1_SWRST;
    I2Cx->CR1 &= ~I2C_CR1_SWRST;
    I2C_enable(I2Cx);

    // Select map according to I2C instance
    const I2C_PinMap_t* map = I2C_get_pin_map(I2Cx);
    if (map == NULL) {
        return STATUS_INVALID;
    }

    PinState_t scl_state, sda_state;
    uint32_t   timeout = I2C_BUS_FREE_TIMEOUT;
    // Make sure bus is free
    while (timeout > 0U) {
        Status_t status = STATUS_OK;
        status |= GPIO_pin_read(map->scl_port, map->scl_pin, &scl_state);
        status |= GPIO_pin_read(map->sda_port, map->sda_pin, &sda_state);
        if (status != STATUS_OK) {
            return STATUS_ERROR;
        } else if (scl_state == PIN_SET && sda_state == PIN_SET) {
            // Bus is clear
            return STATUS_OK;
        }
        timeout--;
    }
    // Timeout expired, bus still busy
    return STATUS_TIMEOUT;
}

/**
 * @brief  Compute the Clock Control Register (CCR) value for I2C timing configuration.
 *
 * Calculates the CCR value based on the peripheral clock frequency (PCLK),
 * desired I2C speed mode (Standard or Fast), and duty cycle setting.
 *
 * **Computation formulas:**
 * - Standard mode (100 kHz):  CCR = F_PCLK / (2 × F_SCL)
 * - Fast mode, DUTY = 0 (2:1): CCR = F_PCLK / (3 × F_SCL)
 * - Fast mode, DUTY = 1 (16:9): CCR = F_PCLK / (25 × F_SCL)
 *
 * @param[in]  pclk        Peripheral clock frequency in Hz.
 * @param[in]  speed_mode  Desired I2C speed (e.g., 100000 for Standard, 400000 for Fast).
 * @param[in]  duty        Duty cycle configuration (0 = 2:1, 1 = 16:9).
 * @param[out] ccr_val     Pointer to store the calculated CCR register value.
 *
 * @retval STATUS_OK        CCR computed successfully.
 * @retval STATUS_INVALID   Invalid input parameter or unsupported mode.
 *
 * @note  The CCR value is automatically clamped to the minimum allowed hardware limit.
 * @note  For Fast Mode operation, the peripheral clock must be ≥ 4 MHz.
 */

static Status_t I2C_calc_ccr(uint32_t pclk, uint32_t speed_mode, uint32_t duty, uint32_t* ccr_val) {
    // Validate output pointer
    if (ccr_val == NULL) {
        return STATUS_INVALID;
    }
    // Compute CCR
    if (speed_mode == I2C_SPEED_STD) { // Standard mode
        *ccr_val = pclk / (2U * speed_mode);
    } else { // Fast mode
        if (speed_mode == I2C_SPEED_FAST && pclk >= I2C_MIN_FAST_PERIPH_CLK_HZ) {
            if (duty == I2C_DUTY_CYCLE_STD) { // 2:1
                *ccr_val = pclk / (3U * speed_mode);
            } else if (duty == I2C_DUTY_CYCLE_FAST) { // 16:9
                *ccr_val = pclk / (25U * speed_mode);
            } else { // invalid duty cycle
                return STATUS_INVALID;
            }
        } else { // speed mode unkown or PCLK not sufficent
            return STATUS_INVALID;
        }
    }

    // Enforce hardware minimum CCR value
    if (*ccr_val < I2C_MIN_CCR_VALUE) {
        *ccr_val = I2C_MIN_CCR_VALUE;
    }
    return STATUS_OK;
}

/**
 * @brief  Configure the I2C peripheral timing registers.
 *
 * Programs the I2C clock control (CCR) and rise time (TRISE) registers
 * based on the peripheral clock frequency, desired I2C speed, and duty cycle.
 * Configuration sequence:
 *
 * 1. Disable the I2C peripheral before modifying timing registers.
 * 2. Program the peripheral input clock frequency (FREQ) in MHz.
 * 3. Compute and set the CCR value (using @ref I2C_calc_ccr()).
 * 4. Compute and set the TRISE value (using @ref I2C_calc_trise()).
 * 5. Re-enable the I2C peripheral.
 *
 * @param[in] I2Cx         Pointer to the I2C peripheral instance (I2C1, I2C2, or I2C3).
 * @param[in] speed_mode   Desired I2C speed in Hz (e.g., 100000 for Standard, 400000 for Fast).
 * @param[in] duty_cycle   Duty cycle mode (0 = 2:1, 1 = 16:9 for Fast Mode).
 *
 * @retval STATUS_OK        Configuration succeeded.
 * @retval STATUS_ERROR     Clock computation or register write failed.
 * @retval STATUS_INVALID   Invalid parameters or insufficient peripheral clock frequency.
 *
 * @note The peripheral clock frequency must be at least 2 MHz for Standard mode
 *       and at least 4 MHz for Fast mode. The function automatically enforces
 *       the hardware minimum CCR value.
 */
static Status_t I2C_config_clock(I2C_TypeDef* I2Cx, uint32_t speed_mode, uint32_t duty_cycle) {
    // Disable I2C before timing configuration
    I2C_disable(I2Cx);

    // Get peripheral input clock frequency (Hz)
    uint32_t periph_clk;
    if (RCC_get_periph_clk(I2Cx, &periph_clk) != STATUS_OK) {
        return STATUS_ERROR;
    }

    // Validate minimum required peripheral clock (spec requires >= 2 MHz)
    if (periph_clk < I2C_MIN_STD_PERIPH_CLK_HZ) {
        return STATUS_INVALID;
    }

    // Write peripheral clock frequency in MHz to CR2 register
    I2Cx->CR2 &= ~I2C_CR2_FREQ_Msk;
    uint32_t freq_mhz = HZ_TO_MHZ(periph_clk);
    I2Cx->CR2 |= (freq_mhz & I2C_CR2_FREQ_Msk);

    // Compute CCR value
    uint32_t ccr_val;
    if (I2C_calc_ccr(periph_clk, speed_mode, duty_cycle, &ccr_val) != STATUS_OK) {
        return STATUS_ERROR;
    }

    // Program CCR register
    I2Cx->CCR = 0;
    if (speed_mode == I2C_SPEED_STD) {
        // Standard mode (100 kHz)
        I2Cx->CCR = ccr_val;
        I2Cx->CCR &= ~I2C_CCR_FS; // Reset FS bit for standard mode
    } else if (speed_mode == I2C_SPEED_FAST) {
        // Fast mode (400 kHz)
        I2Cx->CCR = ccr_val | I2C_CCR_FS; // Enable FS bit for fast mode
        if (duty_cycle == I2C_DUTY_CYCLE_FAST)
            I2Cx->CCR |= I2C_CCR_DUTY; // 16:9 ratio
        else
            I2Cx->CCR &= ~I2C_CCR_DUTY; // 2:1 ratio
    } else {
        return STATUS_INVALID;
    }

    // Compute TRISE value
    uint32_t trise_val;
    if (I2C_calc_trise(periph_clk, speed_mode, &trise_val) != STATUS_OK) {
        return STATUS_ERROR;
    }

    // 7. Write TRISE register
    I2Cx->TRISE = trise_val & I2C_TRISE_TRISE_Msk;

    // All done, re-enable I2C peripheral
    I2C_enable(I2Cx);
    return STATUS_OK;
}
/**
 * @brief  Compute the TRISE (maximum rise time) value for the I2C peripheral.
 *
 * Calculates the TRISE register value based on the I2C speed mode and
 * the peripheral input clock frequency, according to STM32 specifications:
 *
 * - **Standard mode (100 kHz):**  TRISE = F_CLK(MHz) + 1
 * - **Fast mode (400 kHz):**      TRISE = (F_CLK(MHz) × 300 ns) + 1
 *
 * @param[in]  pclk         Peripheral clock frequency in Hz.
 * @param[in]  speed_mode   Desired I2C speed (e.g., 100000 or 400000).
 * @param[out] trise_val    Pointer to store the computed TRISE value.
 *
 * @retval STATUS_OK        TRISE successfully computed.
 * @retval STATUS_INVALID   Invalid parameter or unsupported mode.
 *
 * @note  The TRISE value ensures compliance with I2C timing requirements.
 *        It must be set before enabling the I2C peripheral for communication.
 */

static Status_t I2C_calc_trise(uint32_t pclk, uint32_t speed_mode, uint32_t* trise_val) {
    /*  In standard mode (100 khz), the maximum rise time is 1000ns:
     *  TRISE = F_CLK1(MHz) + 1, where F_CLK1 = peripheral clock frequency in MHz
     *  In fast mode (400 khz), the maximum rise time is 300ns:
     *  TRISE = [F_CLK1(MHz) * 300ns] + 1
     */
    if (!trise_val) {
        return STATUS_INVALID;
    }

    uint32_t freq_mhz = HZ_TO_MHZ(pclk);
    if (freq_mhz == 0U) {
        return STATUS_INVALID;
    }
    if (speed_mode == I2C_SPEED_STD) {
        *trise_val = freq_mhz + 1U;
    } else if (speed_mode == I2C_SPEED_FAST) {
        // Multiply then divide to avoid truncation
        *trise_val = ((freq_mhz * I2C_FAST_MAX_RISE_TIME_NS) / 1000U) + 1U;
    } else {
        return STATUS_INVALID;
    }
    return STATUS_OK;
}

/**
 * @brief  Enable the specified I2C peripheral.
 *
 * Sets the PE (Peripheral Enable) bit in the I2C_CR1 register to activate
 * the selected I2C interface. A short delay is applied to allow the
 * peripheral to stabilize before further configuration or communication.
 *
 * @param[in] I2Cx  Pointer to the I2C peripheral instance (I2C1, I2C2, or I2C3).
 *
 * @note  This function should be called only after configuring timing registers.
 *        The delay ensures proper internal synchronization of the peripheral.
 */

static inline void I2C_enable(I2C_TypeDef* I2Cx) {
    I2Cx->CR1 |= I2C_CR1_PE;
    for (volatile int i = 0; i < 200; i++)
        ; // short delay to allow peripheral to stabilize
}

/**
 * @brief  Disable the specified I2C peripheral.
 *
 * Clears the PE (Peripheral Enable) bit in the I2C_CR1 register to
 * deactivate the selected I2C interface.
 *
 * @param[in] I2Cx  Pointer to the I2C peripheral instance (I2C1, I2C2, or I2C3).
 *
 * @note  This function should be called before reconfiguring CCR, TRISE,
 *        or any other timing-related registers.
 */

static inline void I2C_disable(I2C_TypeDef* I2Cx) {
    I2Cx->CR1 &= ~I2C_CR1_PE;
}

/**
 * @brief  Generate a START condition on the I2C bus.
 *
 * Waits until the bus is free, issues a START condition, and checks for
 * the SB (Start Bit) flag using @ref I2C_wait_flag_recover().
 *
 * @param[in] I2Cx  Pointer to the I2C peripheral instance.
 *
 * @retval STATUS_OK       START generated successfully, or timeout occurred
 *                         but recovery succeeded.
 * @retval STATUS_INVALID  Invalid desired_state parameter passed internally.
 *
 * @note In the current implementation, timeouts are automatically handled
 *       by the recovery mechanism, which may mask underlying bus issues.
 */
static Status_t I2C_start(I2C_TypeDef* I2Cx) {
    // Wait until bus is free (BUSY flag cleared)
    Status_t status = STATUS_OK;
    status          = I2C_wait_flag_recover(I2Cx, I2C_SR2, I2C_SR2_BUSY, FLAG_CLEAR);
    if (status != STATUS_OK) {
        return status;
    }

    // Generate start condition
    I2Cx->CR1 |= I2C_CR1_START;

    // Wait until start generated (SB flag set)
    status = I2C_wait_flag_recover(I2Cx, I2C_SR1, I2C_SR1_SB, FLAG_SET);
    if (status != STATUS_OK) {
        return status;
    }
    return STATUS_OK;
}

/**
 * @brief  Send a 7-bit slave address with direction bit on the I2C bus.
 *
 * Writes the 7-bit slave address (left-shifted by 1) combined with the R/W bit
 * into the data register. The function then waits for the ADDR flag to be set,
 * using @ref I2C_wait_flag_recover(), and clears the ADDR flag by reading SR1
 * followed by SR2.
 *
 * @param[in] I2Cx       Pointer to the I2C peripheral instance.
 * @param[in] address    7-bit slave address.
 * @param[in] direction  Transfer direction: @ref I2C_WRITE or @ref I2C_READ.
 *
 * @retval STATUS_OK       Address sent and acknowledged by the slave, or
 *                         timeout occurred but recovery succeeded.
 * @retval STATUS_INVALID  Invalid direction value or invalid desired_state
 *                         detected internally.
 *
 * @note This function clears the ADDR flag internally as required by STM32 I2C hardware.
 */
static Status_t I2C_send_address(I2C_TypeDef* I2Cx, uint8_t address, I2C_direction_t direction) {
    // Check direction valid
    if (direction != I2C_WRITE && direction != I2C_READ) {
        return STATUS_INVALID;
    }

    // Send address + direction
    I2Cx->DR = (address << 1U) | direction;

    // Wait until address is sent and ackowledged (ADDR flag set)
    Status_t status = I2C_wait_flag_recover(I2Cx, I2C_SR1, I2C_SR1_ADDR, FLAG_SET);
    if (status != STATUS_OK) {
        return status;
    }
    // Clear ADDR flag
    I2C_clear_addr_flag(I2Cx);

    return STATUS_OK;
}
