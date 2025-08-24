/*
 * pcf8574.c
 *
 *  Created on: Aug 01, 2025
 *      Author: Mehmet Keten
 */

#include "pcf8574.h"


/* Dahili yardımcılar */
static inline uint16_t addr7_to_addr8(uint8_t addr7) {
	return (uint16_t)(addr7 << 1); /* HAL 8-bit format */
}

static inline bool pin_valid(uint8_t pin) {
	return pin < 8u;
}


static PCF8574_Status i2c_write_byte(PCF8574_HandleTypeDef *hpcf, uint8_t byte) {
	if (!hpcf || !hpcf->hi2c) return PCF8574_ERR_NULL;
	if (HAL_I2C_Master_Transmit(hpcf->hi2c, addr7_to_addr8(hpcf->addr7), &byte, 1, hpcf->timeout) != HAL_OK) {
		return PCF8574_ERR_I2C;
	}
	return PCF8574_OK;
}

static PCF8574_Status i2c_read_byte(PCF8574_HandleTypeDef *hpcf, uint8_t *byte) {
if (!hpcf || !hpcf->hi2c || !byte) return PCF8574_ERR_NULL;
if (HAL_I2C_Master_Receive(hpcf->hi2c, addr7_to_addr8(hpcf->addr7), byte, 1, hpcf->timeout) != HAL_OK) {
return PCF8574_ERR_I2C;
}
return PCF8574_OK;
}


PCF8574_Status PCF8574_Init(PCF8574_HandleTypeDef *hpcf,
							I2C_HandleTypeDef *hi2c,
							uint8_t addr7,
							uint8_t initial_shadow,
							uint32_t timeout_ms) {
	if (!hpcf || !hi2c) return PCF8574_ERR_NULL;


	hpcf->hi2c = hi2c;
	hpcf->addr7 = addr7 & 0x7Fu; /* 7-bit güvence */
	hpcf->timeout = (timeout_ms == 0u) ? PCF8574_DEFAULT_TIMEOUT_MS : timeout_ms;
	hpcf->shadow = initial_shadow;
	hpcf->name = NULL;
	hpcf->int_gpio_port = NULL;
	hpcf->int_gpio_pin = 0u;
	hpcf->int_pol = PCF8574_INT_ACTIVE_LOW;


	/* Write initial shadow to hardware */
	return i2c_write_byte(hpcf, hpcf->shadow);
}

PCF8574_Status PCF8574_ReadPort(PCF8574_HandleTypeDef *hpcf, uint8_t *port_val) {
	if (!port_val) return PCF8574_ERR_PARAM;
	return i2c_read_byte(hpcf, port_val);
}

PCF8574_Status PCF8574_WritePort(PCF8574_HandleTypeDef *hpcf, uint8_t value) {
	PCF8574_Status st = i2c_write_byte(hpcf, value);
	if (st == PCF8574_OK) {
		hpcf->shadow = value;
	}
	return st;
}

PCF8574_Status PCF8574_WritePin(PCF8574_HandleTypeDef *hpcf, uint8_t pin, GPIO_PinState state) {
	if (!pin_valid(pin)) return PCF8574_ERR_PARAM;

	if (state == GPIO_PIN_SET) {
		hpcf->shadow |= (uint8_t)(1u << pin); /* 1 → input-like */
	} else {
		hpcf->shadow &= (uint8_t)~(1u << pin); /* 0 → drive LOW */
	}
	return PCF8574_WritePort(hpcf, hpcf->shadow);
}

PCF8574_Status PCF8574_ReadPin(PCF8574_HandleTypeDef *hpcf, uint8_t pin, uint8_t *state) {
	if (!pin_valid(pin) || !state) return PCF8574_ERR_PARAM;
	uint8_t val = 0u;
	PCF8574_Status st = PCF8574_ReadPort(hpcf, &val);
	if (st != PCF8574_OK) return st;
	*state = (uint8_t)((val >> pin) & 0x01u);
	return PCF8574_OK;
}

PCF8574_Status PCF8574_TogglePin(PCF8574_HandleTypeDef *hpcf, uint8_t pin) {
	if (!pin_valid(pin)) return PCF8574_ERR_PARAM;
	hpcf->shadow ^= (uint8_t)(1u << pin);
	return PCF8574_WritePort(hpcf, hpcf->shadow);
}

PCF8574_Status PCF8574_SetMask(PCF8574_HandleTypeDef *hpcf, uint8_t mask) {
	hpcf->shadow |= mask;
	return PCF8574_WritePort(hpcf, hpcf->shadow);
}

PCF8574_Status PCF8574_ClearMask(PCF8574_HandleTypeDef *hpcf, uint8_t mask) {
	hpcf->shadow &= (uint8_t)~mask;
	return PCF8574_WritePort(hpcf, hpcf->shadow);
}

bool PCF8574_IsInterruptAsserted(const PCF8574_HandleTypeDef *hpcf) {
	if (!hpcf || !hpcf->int_gpio_port) return false;
	GPIO_PinState s = HAL_GPIO_ReadPin(hpcf->int_gpio_port, hpcf->int_gpio_pin);
	if (hpcf->int_pol == PCF8574_INT_ACTIVE_LOW) {
		return (s == GPIO_PIN_RESET);
	} else {
		return (s == GPIO_PIN_SET);
	}
}
