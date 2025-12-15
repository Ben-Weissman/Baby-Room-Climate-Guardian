#ifndef I2C_H
#define I2C_H

#include "board.h"
#include "gpio.h"
#include "status.h"
#include "stm32f4xx.h"

#ifdef __cplusplus
extern "C" {
#endif

#define I2C_BUS_FREE_TIMEOUT 1000000U
#define I2C_MIN_STD_PERIPH_CLK_HZ 2000000U
#define I2C_MIN_FAST_PERIPH_CLK_HZ 4000000U
#define HZ_TO_MHZ(hz_value) ((hz_value) / 1000000U)
#define I2C_MIN_CCR_VALUE 4U
#define I2C_STD_MAX_RISE_TIME_NS 1000U // 1000ns = 1us
#define I2C_FAST_MAX_RISE_TIME_NS 300U // 300ns

/* Pin Map to support the three I2C modules on the board */
typedef struct {
    GPIO_TypeDef* scl_port;
    uint32_t      scl_pin;
    GPIO_TypeDef* sda_port;
    uint32_t      sda_pin;
} I2C_PinMap_t;

/* Choose the correct I2C module according to it's index */
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

/* Choose wheter we want to write or read in the I2C */
typedef enum {
    I2C_WRITE = 0U,
    I2C_READ  = 1U,
} I2C_direction_t;

/* State of flag bit, either clear or set */
typedef enum { FLAG_CLEAR = 0U, FLAG_SET = 1U } FlagStatus_t;

/* Status register (SR1/SR2) chooser*/
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
    I2C_INIT_OK         = 0U,
    I2C_INIT_RCC_FAIL   = 1U,
    I2C_INIT_GPIO_FAIL  = 2U,
    I2C_INIT_RESET_FAIL = 3U,
    I2C_INIT_CLOCK_FAIL = 4U,
    I2C_INIT_CFG_FAIL   = 5U
} I2C_InitError_t;

/*-------------------------------- Functions ----------------------------*/
/**
 * @brief  Initialize the specified I2C peripheral.
 *
 * Performs the full initialization sequence required to bring the I2C
 * peripheral into a functional state. The sequence includes enabling the
 * peripheral clock, configuring the GPIO pins, issuing a software reset,
 * and programming timing registers (CCR/TRISE).
 *
 * Each initialization step returns a specific error code allowing the caller
 * to determine exactly which phase failed.
 *
 * @param[in] I2Cx  Pointer to the I2C peripheral instance (I2C1, I2C2, or I2C3).
 * @param[in] cfg   Pointer to configuration structure defining speed and duty cycle.
 *
 * @retval I2C_INIT_OK          Initialization completed successfully.
 * @retval I2C_INIT_RCC_FAIL    Failed to enable RCC clock for the I2C peripheral.
 * @retval I2C_INIT_GPIO_FAIL   Failed to configure GPIO pins for SCL/SDA.
 * @retval I2C_INIT_RESET_FAIL  Software reset did not complete successfully.
 * @retval I2C_INIT_CLOCK_FAIL  Timing configuration failed (invalid parameters or clock).
 * @retval I2C_INIT_CFG_FAIL    Configuration is null or has invalid parameters.
 * @note The user must ensure that the peripheral input clock is configured
 *       correctly and that GPIO ports have their RCC clocks enabled.
 */
I2C_InitError_t I2C_init(I2C_TypeDef* I2Cx, I2C_config_t* cfg);

/**
 * @brief  Write a single byte to the I2C data register.
 *
 * Waits until the transmit buffer is empty (TXE=1) or the previous byte has
 * finished transmission (BTF=1), then writes the given byte into the DR register.
 *
 * @param[in] I2Cx  Pointer to the I2C peripheral instance.
 * @param[in] data  Byte to transmit.
 *
 * @retval STATUS_OK       Byte written successfully, or recovery succeeded.
 * @retval STATUS_INVALID  Invalid desired_state parameter detected internally.
 *
 * @note Timeout conditions are handled internally by @ref I2C_wait_flag_recover().
 *       If recovery fails in future revisions of the driver, this function will
 *       propagate a corresponding error code instead of STATUS_OK.
 */

Status_t I2C_master_write_byte(I2C_TypeDef* I2Cx, uint8_t data);

/**
 * @brief  Write a sequence of bytes to a slave device register.
 *
 * Performs a START condition, sends the slave address in write mode, writes
 * the register address, then transmits all bytes from the provided buffer.
 * A STOP condition is generated at the end of the transfer.
 *
 * @param[in] I2Cx        Pointer to the I2C peripheral instance.
 * @param[in] slave_addr  7-bit slave address.
 * @param[in] mem_addr    Register/memory address inside the slave.
 * @param[in] data        Pointer to buffer containing bytes to write.
 * @param[in] length      Number of bytes to transmit.
 *
 * @retval STATUS_OK       Write completed successfully.
 * @retval STATUS_INVALID  Null pointer or zero length.
 * @retval STATUS_ERROR    Failed at any I2C step.
 */
Status_t I2C_master_write(I2C_TypeDef* I2Cx, uint8_t slave_addr, uint8_t mem_addr,
                          const uint8_t* data, uint32_t length);

/**
 * @brief  Read a sequence of bytes from a slave device register.
 *
 * Performs a START condition, sends the slave address in write mode,
 * transmits the register address, issues a repeated START, sends the
 * address in read mode, and receives multiple bytes. ACK is sent for
 * all bytes except the last, which is NACKed before generating STOP.
 *
 * @param[in]  I2Cx        Pointer to the I2C peripheral instance.
 * @param[in]  slave_addr  7-bit slave address.
 * @param[in]  mem_addr    Register/memory address to read from.
 * @param[out] buffer      Buffer to store received bytes.
 * @param[in]  length      Number of bytes to read.
 *
 * @retval STATUS_OK       Read completed successfully.
 * @retval STATUS_INVALID  Null pointer or zero length.
 * @retval STATUS_ERROR    Failed at any I2C step.
 */

Status_t I2C_master_read(I2C_TypeDef* I2Cx, uint8_t slave_addr, uint8_t mem_addr, uint8_t* buffer,
                         uint32_t length);
#ifdef __cplusplus
}
#endif

#endif // I2C_H
