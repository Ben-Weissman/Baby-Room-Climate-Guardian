#include "board.h"
#include "gpio.h"
#include "rcc.h"
#include "status.h"
#include "stm32f4xx.h"
#include "usart.h"
#include <stdint.h>

int main(void) {
    GPIO_init(GPIOA);
    USART_enable(USART2);
    USART_gpio_config(USART2);

    USART_config_t cfg = USART_DEFAULT_CONFIG;
    cfg.baudrate       = 115200;
    USART_init(USART2, &cfg);

    GPIO_mode_configure(GPIOA, GPIO_MODE_OUTPUT, PIN_LED);
    GPIO_port_set_reset(GPIOA, SET, PIN_LED);

    USART_write_string(USART2, "\r\nUSART echo test ready!\r\n");
    USART_write_string(USART2, "Type something and press Enter:\r\n");

    char     buffer[128];
    uint32_t idx = 0;
    uint8_t  ch;

    while (1) {
        if (USART_read_byte(USART2, &ch) == STATUS_OK) {
            // Handle newline (end of message)
            if (ch == '\r' || ch == '\n') {
                buffer[idx] = '\0'; // terminate string
                USART_write_string(USART2, "\r\nYou typed: ");
                USART_write_string(USART2, buffer);
                USART_write_string(USART2, "\r\n> ");
                idx = 0; // reset for next line
            } else {
                // Echo character and store it
                USART_write_byte(USART2, ch);
                if (idx < sizeof(buffer) - 1)
                    buffer[idx++] = ch;
            }
        }
    }
    return 0;
}
