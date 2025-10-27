
#include "usart.h"
#include "gpio.h"
#include "rcc.h"

// -------------------------------- Configuration ------------------------
Status_t USART_enable(USART_TypeDef* USARTx) {
    if (USARTx == USART2) {
        RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    } else if (USARTx == USART1) {
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    } else if (USARTx == USART6) {
        RCC->APB2ENR |= RCC_APB2ENR_USART6EN;

    } else {
        return STATUS_INVALID;
    }
    return STATUS_OK;
}

Status_t USART_gpio_config(USART_TypeDef* USARTx) {
    Status_t status = STATUS_OK;

    if (GPIO_enable(GPIOA) != STATUS_OK) {
        return STATUS_ERROR;
    }

    // Configure PA2/PA3 for USART2 (AF7)
    if (USARTx == USART2) {
        if (GPIO_mode_configure(GPIOA, GPIO_MODE_AF, PIN_2) != STATUS_OK) {
            return STATUS_INVALID;
        }
        if (GPIO_mode_configure(GPIOA, GPIO_MODE_AF, PIN_3) != STATUS_OK) {
            return STATUS_INVALID;
        }
        if (GPIO_config_alternate(GPIOA, PIN_2, GPIO_AF7) != STATUS_OK) { // TX
            return STATUS_INVALID;
        }
        if (GPIO_config_alternate(GPIOA, PIN_3, GPIO_AF7) != STATUS_OK) { // RX
            return STATUS_INVALID;
        }
    }
    // Configure PA9/PA10 for USART1 (AF7)
    else if (USARTx == USART1) {
        if (GPIO_mode_configure(GPIOA, GPIO_MODE_AF, PIN_9) != STATUS_OK) {
            return STATUS_INVALID;
        }
        if (GPIO_mode_configure(GPIOA, GPIO_MODE_AF, PIN_10) != STATUS_OK) {
            return STATUS_INVALID;
        }
        if (GPIO_config_alternate(GPIOA, PIN_9, GPIO_AF7) != STATUS_OK) { // TX
            return STATUS_INVALID;
        }
        if (GPIO_config_alternate(GPIOA, PIN_10, GPIO_AF7) != STATUS_OK) { // RX
            return STATUS_INVALID;
        }
    }
    // Configure PA11/PA12 for USART6 (AF8)
    else if (USARTx == USART6) {
        if (GPIO_mode_configure(GPIOA, GPIO_MODE_AF, PIN_11) != STATUS_OK) {
            return STATUS_INVALID;
        }
        if (GPIO_mode_configure(GPIOA, GPIO_MODE_AF, PIN_12) != STATUS_OK) {
            return STATUS_INVALID;
        }
        if (GPIO_config_alternate(GPIOA, PIN_11, GPIO_AF8) != STATUS_OK) { // TX
            return STATUS_INVALID;
        }
        if (GPIO_config_alternate(GPIOA, PIN_12, GPIO_AF8) != STATUS_OK) { // RX
            return STATUS_INVALID;
        }
    } else {
        return STATUS_INVALID;
    }
    return STATUS_OK;
}

// -------------------------------- Initialization ----------------------------
Status_t USART_init(USART_TypeDef* USARTx, const USART_config_t* cfg) {
    if (USART_set_baudrate(USARTx, cfg->baudrate) != STATUS_OK) {
        return STATUS_INVALID;
    }
    if (USART_set_word_len(USARTx, cfg->word_len) != STATUS_OK) {
        return STATUS_INVALID;
    }
    if (USART_set_stop_bits(USARTx, cfg->stop_bits) != STATUS_OK) {
        return STATUS_INVALID;
    }
    if (USART_set_parity(USARTx, cfg->parity) != STATUS_OK) {
        return STATUS_INVALID;
    }
    if (USART_set_mode(USARTx, cfg->mode) != STATUS_OK) {
        return STATUS_INVALID;
    }
    if (USART_set_oversampling(USARTx, cfg->oversampling) != STATUS_OK) {
        return STATUS_INVALID;
    }

    // after the config, enable the USART module
    USARTx->CR1 |= USART_CR1_UE_Msk;
    return STATUS_OK;
}

Status_t USART_set_baudrate(USART_TypeDef* USARTx, uint32_t baudrate) {
    uint32_t periph_clk;

    if (RCC_get_periph_clk(USARTx, &periph_clk) != STATUS_OK) {
        return STATUS_ERROR;
    }
    // Calculate USARTDIV
    uint32_t usartdiv = (periph_clk + (baudrate / 2U)) / baudrate; // integer rounding formula

    USARTx->BRR = usartdiv;

    return STATUS_OK;
}

Status_t USART_set_word_len(USART_TypeDef* USARTx, Word_length_t word_len) {
    if (word_len == word_len_8b) {
        USARTx->CR1 &= ~USART_CR1_M_Msk;
    } else if (word_len == word_len_9b) {
        USARTx->CR1 |= USART_CR1_M_Msk;
    } else {
        return STATUS_INVALID;
    }
    return STATUS_OK;
}

Status_t USART_set_stop_bits(USART_TypeDef* USARTx, Stop_bits_t stop_bits) {
    if (stop_bits != one_stop_bit && stop_bits != two_stop_bit) {
        return STATUS_INVALID;
    }
    USARTx->CR2 &= ~USART_CR2_STOP_Msk;
    USARTx->CR2 |= stop_bits << USART_CR2_STOP_Pos;
    return STATUS_OK;
}

Status_t USART_set_parity(USART_TypeDef* USARTx, Parity_t parity) {
    if (parity != parity_even && parity != parity_none && parity != parity_odd) {
        return STATUS_INVALID;
    }
    if (parity == parity_none) {
        USARTx->CR1 &= ~USART_CR1_PCE_Msk;
    } else {
        USARTx->CR1 |= USART_CR1_PCE_Msk;
    }

    if (parity == parity_even) {
        USARTx->CR1 &= ~USART_CR1_PS_Msk;
    } else {
        USARTx->CR1 |= USART_CR1_PS_Msk;
    }
    return STATUS_OK;
}

Status_t USART_set_mode(USART_TypeDef* USARTx, USART_mode_t mode) {
    USARTx->CR1 &= ~(USART_CR1_RE_Msk | USART_CR1_TE_Msk); // clear both first
    if (mode == mode_rx)
        USARTx->CR1 |= USART_CR1_RE_Msk;
    else if (mode == mode_tx)
        USARTx->CR1 |= USART_CR1_TE_Msk;
    else if (mode == mode_tx_rx)
        USARTx->CR1 |= USART_CR1_RE_Msk | USART_CR1_TE_Msk;
    else
        return STATUS_INVALID;
    return STATUS_OK;
}

Status_t USART_set_oversampling(USART_TypeDef* USARTx, Oversampling_t oversampling) {
    if (oversampling == oversample_16) {
        USARTx->CR1 &= ~USART_CR1_OVER8_Msk;
    } else if (oversampling == oversample_8) {
        USARTx->CR1 |= USART_CR1_OVER8_Msk;
    } else {
        return STATUS_INVALID;
    }
    return STATUS_OK;
}

// -------------------------------------- Runtime I/O ------------------------
Status_t USART_write_byte(USART_TypeDef* USARTx, uint8_t data) {
    while (!(USARTx->SR & USART_SR_TXE_Msk)) {
        // wait until TXE=1 (data register empty)
    }
    USARTx->DR = data & USART_DR_DR_Msk;
    return STATUS_OK;
}

Status_t USART_write_buffer(USART_TypeDef* USARTx, const uint8_t* data, uint32_t len) {
    if (data == NULL || len == 0U) {
        return STATUS_INVALID;
    }
    for (uint32_t i = 0; i < len; i++) {
        if (USART_write_byte(USARTx, data[i]) != STATUS_OK) {
            return STATUS_ERROR;
        }
    }
    return STATUS_OK;
}

Status_t USART_write_string(USART_TypeDef* USARTx, const char* str) {
    if (str == NULL) {
        return STATUS_INVALID;
    }

    while (*str) {
        if (USART_write_byte(USARTx, (uint8_t) *str++) != STATUS_OK) {
            return STATUS_ERROR;
        }
    }
    return STATUS_OK;
}

Status_t USART_read_byte(USART_TypeDef* USARTx, uint8_t* data) {
    while (!(USARTx->SR & USART_SR_RXNE_Msk)) {
        // busy wait whilst data register is empty
    }
    *data = USARTx->DR & USART_DR_DR_Msk;
    return STATUS_OK;
}

static inline uint8_t is_line_end(char ch) {
    return (ch == '\r' || ch == '\n');
}

Status_t USART_read_buffer(USART_TypeDef* USARTx, uint8_t* data, uint32_t len) {
    if (data == NULL || len == 0U) {
        return STATUS_INVALID;
    }
    for (uint32_t i = 0; i < len; i++) {
        if (USART_read_byte(USARTx, &data[i]) != STATUS_OK) {
            return STATUS_ERROR;
        }
    }
    return STATUS_OK;
}

Status_t USART_read_line(USART_TypeDef* USARTx, char* str, uint32_t max_len) {
    if (str == NULL || max_len == 0U) {
        return STATUS_INVALID;
    }
    for (uint32_t i = 0; i < max_len - 1; i++) { // leave space for null termniation
        uint8_t ch;
        if (USART_read_byte(USARTx, &ch) != STATUS_OK) {
            return STATUS_ERROR;
        }

        // if '\r' or '\n' -> null terminate the string
        if (is_line_end(ch)) {
            str[i] = '\0';
            return STATUS_OK;
        }

        str[i] = (char) ch;
    }

    // if reached here , end of buffer
    str[max_len - 1] = '\0';
    return STATUS_TIMEOUT;
}
