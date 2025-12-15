#include "board.h"
#include "gpio.h"
#include "i2c.h"
#include "rcc.h"
#include "status.h"
#include "stm32f4xx.h"
#include "usart.h"
#include <stdint.h>

#define BME280_ADDR 0x76 // If your board uses SDO pulled-up -> use 0x77
#define BME280_REG_ID 0xD0

static void print_hex_byte(uint8_t byte) {
    uint8_t hi = (byte >> 4) & 0xF;
    uint8_t lo = byte & 0xF;

    USART_write_byte(USART2, hi < 10 ? ('0' + hi) : ('A' + hi - 10));
    USART_write_byte(USART2, lo < 10 ? ('0' + lo) : ('A' + lo - 10));
}

int main(void) {
    // -----------------------------
    // USART2 INIT
    // -----------------------------
    USART_enable(USART2);
    USART_gpio_config(USART2);

    USART_config_t uart_cfg = USART_DEFAULT_CONFIG;
    USART_init(USART2, &uart_cfg);
    USART_write_string(USART2, "BME280 test start...\r\n");

    // -----------------------------
    // I2C1 INIT
    // -----------------------------
    I2C_config_t cfg;
    cfg.speed_mode = I2C_SPEED_STD;      // 100kHz
    cfg.duty_cycle = I2C_DUTY_CYCLE_STD; // Standard mode

    if (I2C_init(I2C1, &cfg) != I2C_INIT_OK) {
        USART_write_string(USART2, "I2C init FAILED\r\n");
        while (1)
            ;
    }
    USART_write_string(USART2, "I2C ready\r\n");

    // -----------------------------
    // READ CHIP ID
    // -----------------------------
    uint8_t  id  = 0;
    Status_t res = I2C_master_read(I2C1, BME280_ADDR, BME280_REG_ID, &id, 1);

    if (res != STATUS_OK) {
        USART_write_string(USART2, "BME280 READ ERROR\r\n");
    } else {
        USART_write_string(USART2, "BME280 ID = 0x");
        print_hex_byte(id);
        USART_write_string(USART2, "\r\n");
    }

    while (1) {
    }
}
