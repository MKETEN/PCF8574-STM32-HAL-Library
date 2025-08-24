/*
 * pcf8574_cfg.h
 *
 *  Created on: Aug 01, 2025
 *      Author: Mehmet Keten
 */

#ifndef PCF8574_CFG_H
#define PCF8574_CFG_H


/**
* \file pcf8574_cfg.h
* \brief Compile-time configurations.
*/


/** @brief Default I2C timeout (ms) */
#ifndef PCF8574_DEFAULT_TIMEOUT_MS
#define PCF8574_DEFAULT_TIMEOUT_MS 100u
#endif


/** @brief Turn parameter checks on/off (1/0). */
#ifndef PCF8574_PARAM_CHECK
#define PCF8574_PARAM_CHECK 1
#endif


#endif /* PCF8574_CFG_H */
