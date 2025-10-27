#ifndef RCC_H
#define RCC_H

#include "board.h"  // For HSI_VALUE / HSE_VALUE and peripheral base addresses
#include "status.h" // For Status_t return type

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file    rcc.h
 * @brief   RCC (Reset and Clock Control) driver interface.
 *
 * Provides functions to query the current system clock (SYSCLK)
 * and the clock frequency of specific peripherals (e.g., USART).
 *
 */

/**
 * @brief  Get the current system clock (SYSCLK) frequency.
 *
 * Reads the active clock source (HSI, HSE, or PLL) and calculates
 * the resulting SYSCLK frequency in Hz.
 *
 * @param[out] sys_clk
 *   Pointer to a variable that will receive the calculated SYSCLK frequency in Hz.
 *
 * @retval STATUS_OK
 *   Calculation successful.
 * @retval STATUS_INVALID
 *   The SYSCLK source is unsupported or cannot be determined.
 */
Status_t get_sys_clk(uint32_t* sys_clk);

/**
 * @brief  Get the clock frequency for a given peripheral.
 *
 * Calculates HCLK, PCLK1, and PCLK2 from the current RCC configuration
 * and returns the clock frequency that drives the requested peripheral.
 *
 * @param[in]  periph
 *   Pointer to the peripheral base address (e.g., USART2).
 *
 * @param[out] periph_clk
 *   Pointer to a variable that will receive the peripheral clock frequency in Hz.
 *
 * @retval STATUS_OK
 *   Calculation successful.
 * @retval STATUS_INVALID
 *   The peripheral is not supported or configuration is invalid.
 */
Status_t RCC_get_periph_clk(void* periph, uint32_t* periph_clk);

#ifdef __cplusplus
}
#endif

#endif /* RCC_H */
