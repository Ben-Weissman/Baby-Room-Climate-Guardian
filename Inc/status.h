#ifndef STATUS_H
#define STATUS_H

typedef enum {
    STATUS_OK      = 0,  // Success
    STATUS_ERROR   = -1, // Generic error
    STATUS_INVALID = -2, // Invalid parameter
    STATUS_TIMEOUT = -3  // Peripheral timeout
} Status_t;

typedef enum { PIN_RESET = 0, PIN_SET = 1 } PinState_t;

#endif // STATUS_H
