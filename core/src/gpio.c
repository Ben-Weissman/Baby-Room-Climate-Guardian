#include "gpio.h"

#define GPIO_OTYPER_OPENDRAIN_MASK(pin) (1U << (pin))
#define GPIO_OTYPER_PUSHPULL_MASK(pin) (1U << (pin))
#define GPIO_PUPD_NO_PULL_MASK(pin) (1U << (pin))

Status_t GPIO_enable(GPIO_TypeDef* GPIOx) {
    // supports only A–E, H
    if (GPIOx == GPIOA) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    } else if (GPIOx == GPIOB) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    } else if (GPIOx == GPIOC) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    } else if (GPIOx == GPIOD) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    } else if (GPIOx == GPIOE) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
    } else if (GPIOx == GPIOH) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
    } else {
        return STATUS_INVALID;
    }
    return STATUS_OK;
}

Status_t GPIO_mode_configure(GPIO_TypeDef* GPIOx, uint32_t mode, uint32_t pin_number) {
    if (mode > GPIO_MODE_ANALOG || pin_number > 15U) {
        return STATUS_INVALID;
    }
    uint32_t pos = pin_number * 2;
    GPIOx->MODER &= ~(3U << pos);  // reset previous pin mode
    GPIOx->MODER |= (mode << pos); // set new pin mode

    return STATUS_OK;
}

Status_t GPIO_port_set_reset(GPIO_TypeDef* GPIOx, uint32_t value, uint32_t pin_number) {
    if (pin_number > PIN_15) {
        return STATUS_INVALID;
    }

    if (value == SET) {
        GPIOx->BSRR = (SET << pin_number); // set pin
    } else if (value == RESET) {
        GPIOx->BSRR = (SET << (pin_number + 16U)); // reset pin
    } else {
        return STATUS_INVALID;
    }
    return STATUS_OK;
}

Status_t GPIO_pin_read(GPIO_TypeDef* GPIOx, uint32_t pin_number, PinState_t* state) {
    if (pin_number > 15U) {
        return STATUS_INVALID;
    }
    // if the pin is set, return PIN_SET(1), else PIN_RESET(0)
    *state = (GPIOx->IDR & (PIN_SET << pin_number)) ? PIN_SET : PIN_RESET;
    return STATUS_OK;
}

Status_t GPIO_config_alternate(GPIO_TypeDef* GPIOx, uint32_t pin_number, uint32_t af_value) {
    if (af_value > GPIO_AF15) {
        return STATUS_INVALID;
    }
    uint32_t bit_offset;

    if (pin_number <= PIN_7) {
        // first clean then set port to the given function
        bit_offset = pin_number * AFR_FIELD_WIDTH;
        GPIOx->AFR[0] &= ~(0xFU << bit_offset);
        GPIOx->AFR[0] |= (af_value << bit_offset);

    } else if (pin_number <= PIN_15) {
        bit_offset = (pin_number - 8U) * AFR_FIELD_WIDTH; // to offsize pin 8 -> 15
        GPIOx->AFR[1] &= ~(0xFU << bit_offset);
        GPIOx->AFR[1] |= (af_value << bit_offset);
    } else {
        return STATUS_INVALID;
    }
    return STATUS_OK;
}

Status_t GPIO_pin_output_type(GPIO_TypeDef* GPIOx, uint32_t type, uint32_t pin_number) {
    if (pin_number > PIN_15) {
        return STATUS_INVALID;
    }
    if (type == GPIO_OUTPUT_OPENDRAIN) {
        GPIOx->OTYPER |= GPIO_OTYPER_OPENDRAIN_MASK(pin_number);
    } else if (type == GPIO_OUTPUT_PUSHPULL) {
        GPIOx->OTYPER &= GPIO_OTYPER_PUSHPULL_MASK(pin_number);
    } else {
        return STATUS_INVALID;
    }
    return STATUS_OK;
}

Status_t GPIO_pin_pullup_pulldown(GPIO_TypeDef* GPIOx, GPIO_PullMode_t mode, uint32_t pin_number) {
    if (pin_number > PIN_15)
        return STATUS_INVALID;

    uint32_t shift = pin_number * 2U;
    uint32_t mask  = (3U << shift); // each pin uses 2 bits in PUPDR

    // clear current pull configuration
    GPIOx->PUPDR &= ~mask;

    // apply new configuration if valid
    if (mode == GPIO_NO_PULL || mode == GPIO_PULLUP || mode == GPIO_PULLDOWN) {
        GPIOx->PUPDR |= (mode << shift);
        return STATUS_OK;
    }
    return STATUS_INVALID;
}
