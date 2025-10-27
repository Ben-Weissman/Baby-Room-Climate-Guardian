#ifndef GPIO_H
#define GPIO_H

#include "board.h"
#include "status.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AFR_FIELD_WIDTH 4U

/**
 * @brief  Configure mode of a GPIO pin.
 * @param  GPIOx       Pointer to GPIO port (e.g., GPIOA, GPIOB).
 * @param  mode        Pin mode (GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, etc.).
 * @param  pin_number  Pin number [0..15].
 * @retval STATUS_OK if successful, STATUS_INVALID if parameters are wrong.
 *
 * Example:
 *   GPIO_mode_configure(GPIOA, GPIO_MODE_OUTPUT, 5); // PA5 as output
 */
Status_t GPIO_mode_configure(GPIO_TypeDef* GPIOx, uint32_t mode, uint32_t pin_number);

/**
 * @brief  Enable the peripheral clock for a GPIO port.
 * @note   This function supports only GPIO ports A–E and H.
 * @param  GPIOx  Pointer to GPIO port base address (e.g., GPIOA, GPIOB, ...).
 * @retval STATUS_OK      Clock successfully enabled for the given GPIO port.
 * @retval STATUS_INVALID The given GPIOx is not supported by this function.
 *
 */
Status_t GPIO_enable(GPIO_TypeDef* GPIOx);

/**
 * @brief  Set or reset a GPIO pin using the BSRR register.
 *
 * This function writes directly to the GPIOx->BSRR register to atomically
 * set or reset a single output pin.
 *
 * - Writing a '1' to bits [15:0] sets the corresponding pin.
 * - Writing a '1' to bits [31:16] resets the corresponding pin.
 *
 * @param[in]  GPIOx       Pointer to the GPIO peripheral (e.g. GPIOA, GPIOB).
 * @param[in]  value       Desired pin action:
 *                        - SET (1)   : set the pin high
 *                        - RESET (0) : reset the pin low
 * @param[in]  pin_number  Pin number to configure (0–15).
 *
 * @retval STATUS_OK       Operation successful.
 * @retval STATUS_INVALID  Invalid pin number or invalid action value.
 */
Status_t GPIO_port_set_reset(GPIO_TypeDef* GPIOx, uint32_t value, uint32_t pin_number);

/**
 * @brief  Read the logic level of a GPIO pin.
 *
 * This function checks the input data register (IDR) of the specified GPIO port
 * and stores the pin state (high or low) in the provided output parameter.
 *
 * @param[in]   GPIOx       Pointer to the GPIO peripheral (e.g., GPIOA, GPIOB).
 * @param[in]   pin_number  Pin number to read (0–15).
 * @param[out]  state       Pointer to a variable where the pin state will be
 * stored:
 *                          - PIN_SET   : pin is high (logic '1')
 *                          - PIN_RESET : pin is low (logic '0')
 *
 * @retval STATUS_OK       Operation successful, @p state contains valid data.
 * @retval STATUS_INVALID  Invalid pin number (greater than 15).
 *
 * @note   This function does not configure the pin mode; it assumes the pin
 *        mode is already set as an input.
 *
 * Example:
 * @code
 *   PinState_t pin_state;
 *   if (GPIO_pin_read(GPIOA, 0, &pin_state) == STATUS_OK) {
 *       if (pin_state == PIN_SET) {
 *           // PA0 is high
 *       }
 *   }
 * @endcode
 */
Status_t GPIO_pin_read(GPIO_TypeDef* GPIOx, uint32_t pin_number, PinState_t* state);

/**
 * @brief Configure a GPIO pin for an alternate function.
 *
 * @param[in] GPIOx      Pointer to GPIO port (e.g. GPIOA)
 * @param[in] pin_number Pin number (0–15)
 * @param[in] af_value   Alternate function value (0–15, e.g. GPIO_AF7)
 *
 * @retval STATUS_OK      Configuration succeeded
 * @retval STATUS_INVALID Invalid AF value or pin number
 */
Status_t GPIO_config_alternate(GPIO_TypeDef* GPIOx, uint32_t pin_number, uint32_t af_value);

#ifdef __cplusplus
}
#endif

#endif /* GPIO_H */
