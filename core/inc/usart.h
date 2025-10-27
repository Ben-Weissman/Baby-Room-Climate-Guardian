/**
 * @file usart.h
 * @brief USART driver interface for STM32F4xx.
 *
 * Provides configuration and initialization functions for USART peripherals,
 * including baud rate, word length, stop bits, parity, mode, and oversampling.
 */

#ifndef USART_H
#define USART_H

#include "board.h"
#include "status.h"
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Default USART configuration macro.
 *
 * 115200 baud, 8 data bits, 1 stop bit, no parity, TX+RX enabled, 16x oversampling.
 */
#define USART_DEFAULT_CONFIG                                                                       \
    {.baudrate     = 115200,                                                                       \
     .word_len     = word_len_8b,                                                                  \
     .stop_bits    = one_stop_bit,                                                                 \
     .parity       = parity_none,                                                                  \
     .mode         = mode_tx_rx,                                                                   \
     .oversampling = oversample_16}

/** @brief Parity selection options. */
typedef enum {
    parity_none = 0U, /**< Parity check disabled */
    parity_even = 2U, /**< Enable parity check, even parity (PCE=1, PS=0) */
    parity_odd  = 3U  /**< Enable parity check, odd parity  (PCE=1, PS=1) */
} Parity_t;

/** @brief Word length options. */
typedef enum {
    word_len_8b = 0U, /**< 8-bit data */
    word_len_9b = 1U  /**< 9-bit data */
} Word_length_t;

/** @brief Stop bit options. */
typedef enum {
    one_stop_bit = 0U, /**< 1 stop bit */
    two_stop_bit = 2U  /**< 2 stop bits */
} Stop_bits_t;

/** @brief USART operating modes. */
typedef enum {
    mode_tx    = 0U, /**< Transmit only */
    mode_rx    = 1U, /**< Receive only */
    mode_tx_rx = 2U  /**< Transmit and receive */
} USART_mode_t;

/** @brief Oversampling options. */
typedef enum {
    oversample_16 = 0U, /**< Oversampling by 16 (default) */
    oversample_8  = 1U  /**< Oversampling by 8 */
} Oversampling_t;

/**
 * @brief USART configuration structure.
 */
typedef struct {
    uint32_t       baudrate;     /**< Communication speed in bits per second */
    Word_length_t  word_len;     /**< Number of data bits (8 or 9) */
    Stop_bits_t    stop_bits;    /**< Number of stop bits (1 or 2) */
    Parity_t       parity;       /**< Parity setting (none/even/odd) */
    USART_mode_t   mode;         /**< TX/RX selection */
    Oversampling_t oversampling; /**< Oversampling mode (8 or 16) */
} USART_config_t;

/* ------------------- Function Prototypes ------------------- */

/**
 * @brief Enable the clock for a USART peripheral.
 *
 * @param USARTx Pointer to USART instance (USART1, USART2, USART6).
 * @retval STATUS_OK on success.
 * @retval STATUS_INVALID if USARTx is not supported.
 */
Status_t USART_enable(USART_TypeDef* USARTx);

/**
 * @brief Configure GPIO pins for USART TX/RX.
 *
 * @param USARTx Pointer to USART instance.
 * @retval STATUS_OK on success.
 * @retval STATUS_INVALID if the USART peripheral is not supported.
 */
Status_t USART_gpio_config(USART_TypeDef* USARTx);

/**
 * @brief Initialize a USART peripheral with the provided configuration.
 *
 * @param USARTx Pointer to USART instance.
 * @param cfg    Pointer to a USART_config_t structure.
 * @retval STATUS_OK on success.
 * @retval STATUS_INVALID on configuration error.
 */
Status_t USART_init(USART_TypeDef* USARTx, const USART_config_t* cfg);

/**
 * @brief Set the baud rate for a USART peripheral.
 *
 * @param USARTx   Pointer to USART instance.
 * @param baudrate Desired baud rate in bits per second.
 * @retval STATUS_OK on success.
 * @retval STATUS_ERROR if the peripheral clock cannot be determined.
 */
Status_t USART_set_baudrate(USART_TypeDef* USARTx, uint32_t baudrate);

/**
 * @brief Set the data word length.
 *
 * @param USARTx   Pointer to USART instance.
 * @param word_len Desired word length (8b or 9b).
 * @retval STATUS_OK on success.
 * @retval STATUS_INVALID if word_len is unsupported.
 */
Status_t USART_set_word_len(USART_TypeDef* USARTx, Word_length_t word_len);

/**
 * @brief Set the number of stop bits.
 *
 * @param USARTx   Pointer to USART instance.
 * @param stop_bits Number of stop bits (1 or 2).
 * @retval STATUS_OK on success.
 * @retval STATUS_INVALID if stop_bits is unsupported.
 */
Status_t USART_set_stop_bits(USART_TypeDef* USARTx, Stop_bits_t stop_bits);

/**
 * @brief Set the parity mode.
 *
 * @param USARTx Pointer to USART instance.
 * @param parity Parity mode (none/even/odd).
 * @retval STATUS_OK on success.
 * @retval STATUS_INVALID if parity is unsupported.
 */
Status_t USART_set_parity(USART_TypeDef* USARTx, Parity_t parity);

/**
 * @brief Set the USART mode (TX, RX, or TX/RX).
 *
 * @param USARTx Pointer to USART instance.
 * @param mode   Mode to configure.
 * @retval STATUS_OK on success.
 * @retval STATUS_INVALID if mode is unsupported.
 */
Status_t USART_set_mode(USART_TypeDef* USARTx, USART_mode_t mode);

/**
 * @brief Set the oversampling mode.
 *
 * @param USARTx      Pointer to USART instance.
 * @param oversampling Desired oversampling mode (8 or 16).
 * @retval STATUS_OK on success.
 * @retval STATUS_INVALID if oversampling is unsupported.
 */
Status_t USART_set_oversampling(USART_TypeDef* USARTx, Oversampling_t oversampling);

/**
 * @brief Transmit one byte (blocking until TX ready).
 * @param USARTx USART instance.
 * @param data Byte to send.
 * @retval STATUS_OK on success.
 */
Status_t USART_write_byte(USART_TypeDef* USARTx, uint8_t data);

/**
 * @brief Transmit a data buffer.
 * @param USARTx USART instance.
 * @param data Pointer to buffer.
 * @param len Number of bytes.
 * @retval STATUS_OK, STATUS_INVALID, or STATUS_ERROR.
 */
Status_t USART_write_buffer(USART_TypeDef* USARTx, const uint8_t* data, uint32_t len);

/**
 * @brief Transmit a null-terminated string.
 * @param USARTx USART instance.
 * @param str Null-terminated string.
 * @retval STATUS_OK, STATUS_INVALID, or STATUS_ERROR.
 */
Status_t USART_write_string(USART_TypeDef* USARTx, const char* str);

/**
 * @brief Read one byte (blocking until RX ready).
 * @param USARTx USART instance.
 * @param data Pointer to store byte.
 * @retval STATUS_OK.
 */
Status_t USART_read_byte(USART_TypeDef* USARTx, uint8_t* data);

/**
 * @brief Read multiple bytes into buffer.
 * @param USARTx USART instance.
 * @param data Pointer to buffer.
 * @param len Number of bytes.
 * @retval STATUS_OK, STATUS_INVALID, or STATUS_ERROR.
 * @note Assumes caller allocates enough memory in data.
 */
Status_t USART_read_buffer(USART_TypeDef* USARTx, uint8_t* data, uint32_t len);

/**
 * @brief Read a text line until '\r' or '\n'.
 * @param USARTx USART instance.
 * @param str Buffer to store line.
 * @param max_len Buffer length.
 * @retval STATUS_OK, STATUS_ERROR, STATUS_TIMEOUT, or STATUS_INVALID.
 * @note Assumes caller allocates enough memory in str
 */
Status_t USART_read_line(USART_TypeDef* USARTx, char* str, uint32_t max_len);

#ifdef __cplusplus
}
#endif

#endif // USART_H
