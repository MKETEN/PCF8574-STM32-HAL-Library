/*
 * pcf8574.h
 *
 *  Created on: Aug 01, 2025
 *      Author: Mehmet Keten
 */

#ifndef PCF8574_H
#define PCF8574_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h" // Add the correct HAL header according to your project (e.g. stm32f1xx_hal.h, stm32f3xx_hal.h, stm32f4xx_hal.h ...)
#include "pcf8574_cfg.h"

/**
* \file pcf8574.h
* \brief STM32 HAL PCF8574 driver header file.
* \details
* - Holds the shadow byte for quasi-bidirectional pin logic
* - Takes a 7-bit device address and converts it to 8-bit for HAL calls.
* - Provides basic pin/port read-write, toggle, and masking APIs.
*/


/** @brief Driver return states */
typedef enum {
PCF8574_OK = 0,
PCF8574_ERR_I2C,
PCF8574_ERR_PARAM,
PCF8574_ERR_NULL
} PCF8574_Status;

/** @brief Optional interrupt pin active polarity */
typedef enum {
PCF8574_INT_ACTIVE_LOW = 0,
PCF8574_INT_ACTIVE_HIGH = 1
} PCF8574_IntPolarity;


/** @brief PCF8574 driver handle structure */
typedef struct {
I2C_HandleTypeDef *hi2c; /**< HAL I2C handle pointer */
uint8_t addr7; /**< 7-bit device address (typical: 0x20–0x27) */
uint8_t shadow; /**< Shadow write byte (last written output image) */
uint32_t timeout; /**< I2C timeout in ms */
const char *name; /**< (Ops.) for debugging/tracing */


/* Optional INT pin definitions (set to NULL/0 if not used) */
GPIO_TypeDef *int_gpio_port; /**< INT pin port */
uint16_t int_gpio_pin; /**< INT pin number */
PCF8574_IntPolarity int_pol; /**< INT active polarity */
} PCF8574_HandleTypeDef;

/**
* @brief Prepares the driver and writes the initial shadow value to hardware.
* @param hpcf Driver handle (expects a populated structure)
* @param hi2c HAL I2C handle to be used
* @param addr7 7-bit device address (e.g. 0x20)
* @param initial_shadow Initial output/input image. Typical: 0xFF (all 1 → input‑like)
* @param timeout_ms I2C timeout (ms)
* @return PCF8574_Status
*/
PCF8574_Status PCF8574_Init(PCF8574_HandleTypeDef *hpcf,
I2C_HandleTypeDef *hi2c,
uint8_t addr7,
uint8_t initial_shadow,
uint32_t timeout_ms);

/**
* @brief Reads the entire port (8 bits).
*/
PCF8574_Status PCF8574_ReadPort(PCF8574_HandleTypeDef *hpcf, uint8_t *port_val);


/**
* @brief Writes the entire port (8 bits). Shadow value is updated.
*/
PCF8574_Status PCF8574_WritePort(PCF8574_HandleTypeDef *hpcf, uint8_t value);


/**
* @brief Writes the specified pin (0..7). Drives 1 → input‑like, 0 → LOW.
*/
PCF8574_Status PCF8574_WritePin(PCF8574_HandleTypeDef *hpcf, uint8_t pin, GPIO_PinState state);


/**
* @brief Reads the specified pin (0..7). Returned value: 0/1.
*/
PCF8574_Status PCF8574_ReadPin(PCF8574_HandleTypeDef *hpcf, uint8_t pin, uint8_t *state);


/**
* @brief Toggles the specified pin (via shadow) and writes to port.
*/
PCF8574_Status PCF8574_TogglePin(PCF8574_HandleTypeDef *hpcf, uint8_t pin);


/**
* @brief Makes multiple pins 1 (input‑like) with mask.
*/
PCF8574_Status PCF8574_SetMask(PCF8574_HandleTypeDef *hpcf, uint8_t mask);


/**
* @brief Makes multiple pins 0 (LOW) with mask.
*/
PCF8574_Status PCF8574_ClearMask(PCF8574_HandleTypeDef *hpcf, uint8_t mask);


/**
* @brief Optional: Reads the current level of the INT pin (via GPIO).
*/
bool PCF8574_IsInterruptAsserted(const PCF8574_HandleTypeDef *hpcf);


#ifdef __cplusplus
}
#endif


#endif /* PCF8574_H */
