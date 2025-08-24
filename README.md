# STM32 HAL PCF8574 Driver

A small, well‑documented driver for the **PCF8574** I²C GPIO expander using the **STM32 HAL**. The goal is to provide a clean, readable, and production‑friendly API with clear examples and comments.

> **Why this driver?**
> PCF8574 uses a *quasi‑bidirectional* pin model. That is often confusing at first. This driver hides the details with a **shadow byte** and a set of simple functions (`WritePin`, `ReadPin`, `TogglePin`, `Read/WritePort`, masks). You pass a 7‑bit address; the driver does the HAL‑expected shifting internally.


## Features

* Works with **STM32 HAL** (`HAL_I2C_Master_Transmit/Receive`).
* Takes **7‑bit I²C address** (e.g. `0x20`); internally shifts to HAL’s 8‑bit form.
* Maintains a **shadow byte** to make the PCF8574’s quasi‑bidirectional model easy.
* Pin‑level and port‑level read/write, bit‑mask helpers, toggle.
* Optional **INT** pin support for change detection (polarity configurable).
* Lightweight (no dynamic allocation) and portable across STM32 families.

> **Example Addressing:** PCF8574 (0x20–0x27 depending on A2..A0). The `A` variant (PCF8574A) commonly uses 0x38–0x3F. Check your module’s silk screen and datasheet.

---

## Hardware Background

**PCF8574 pins are not classic push‑pull outputs.** A bit written as **0** actively drives the pin **LOW**. A bit written as **1** *releases* the pin via a weak pull‑up, so the line goes **HIGH** if there is an external pull‑up or no external load; this is also how you read inputs. There is no direction register.

This driver treats **1** as *input‑like/high with weak pull‑up* and **0** as *low output*. Reading uses an I²C receive of one byte.

---

## Repository Layout

```
Core/
  Inc/
    pcf8574.h        # Public API
    pcf8574_cfg.h    # Build‑time configuration (timeouts, guards)
  Src/
    pcf8574.c        # Implementation (HAL based)
```

You can also place the files under `Drivers/` or your preferred layout.

---

## Getting Started

### Prerequisites

* STM32 project using **HAL** (CubeMX/CubeIDE or your own build system).
* I²C bus configured (100 kHz or 400 kHz).
* Proper **I²C pull‑ups** (typically 4.7 kΩ) to VDD on SDA/SCL.

### Add Files

Copy `pcf8574.h`, `pcf8574.c`, and `pcf8574_cfg.h` into your project (`Inc/` and `Src/`). Add `pcf8574.c` to your build.

### Wire It Up

* **VDD**: 3.3 V (check your board; many modules support 3.3–5 V).
* **GND**: Ground.
* **SCL/SDA**: To MCU I²C pins with pull‑ups.
* **A2/A1/A0**: Set hardware address (0/1). Base is commonly `0x20` for PCF8574.
* **INT** (optional): Open‑drain interrupt output; connect to a GPIO/EXTI with a pull‑up.

> **Address cheatsheet:**
> If base is `0x20`, then:
> `A2 A1 A0` = 000 → `0x20`, 001 → `0x21`, …, 111 → `0x27`.

### CubeIDE / HAL Setup

1. Enable an I²C peripheral (e.g., **I2C1**).
2. Choose speed (Standard 100 kHz or Fast 400 kHz) to match your bus.
3. Enable GPIO alternate‑function pins for SDA/SCL.
4. Ensure external pull‑ups exist on SDA/SCL.
5. Generate code; you should have `I2C_HandleTypeDef hi2cX` in your project.

---

## Configuration

`pcf8574_cfg.h` exposes small build‑time knobs:

```c
#ifndef PCF8574_DEFAULT_TIMEOUT_MS
#define PCF8574_DEFAULT_TIMEOUT_MS  100u  // HAL I2C timeout in ms
#endif

#ifndef PCF8574_PARAM_CHECK
#define PCF8574_PARAM_CHECK         1     // Enable/disable parameter guards
#endif
```

You can override these via compiler flags or edit the header.

---

## API Overview

```c
// Status codes
typedef enum {
    PCF8574_OK = 0,
    PCF8574_ERR_I2C,
    PCF8574_ERR_PARAM,
    PCF8574_ERR_NULL
} PCF8574_Status;

// Optional INT polarity
typedef enum {
    PCF8574_INT_ACTIVE_LOW = 0,
    PCF8574_INT_ACTIVE_HIGH = 1
} PCF8574_IntPolarity;

// Handle
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t  addr7;        // 7-bit device address (e.g., 0x20)
    uint8_t  shadow;       // last written byte
    uint32_t timeout;      // I2C timeout (ms)
    const char *name;      // optional tag for logging
    GPIO_TypeDef *int_gpio_port; // optional INT GPIO
    uint16_t int_gpio_pin;       // optional INT pin
    PCF8574_IntPolarity int_pol; // INT active level
} PCF8574_HandleTypeDef;

// Init & I/O
PCF8574_Status PCF8574_Init(PCF8574_HandleTypeDef *h,
                            I2C_HandleTypeDef *hi2c,
                            uint8_t addr7,
                            uint8_t initial_shadow,
                            uint32_t timeout_ms);
PCF8574_Status PCF8574_ReadPort (PCF8574_HandleTypeDef *h, uint8_t *port_val);
PCF8574_Status PCF8574_WritePort(PCF8574_HandleTypeDef *h, uint8_t value);
PCF8574_Status PCF8574_WritePin (PCF8574_HandleTypeDef *h, uint8_t pin, GPIO_PinState state);
PCF8574_Status PCF8574_ReadPin  (PCF8574_HandleTypeDef *h, uint8_t pin, uint8_t *state);
PCF8574_Status PCF8574_TogglePin(PCF8574_HandleTypeDef *h, uint8_t pin);
PCF8574_Status PCF8574_SetMask  (PCF8574_HandleTypeDef *h, uint8_t mask);
PCF8574_Status PCF8574_ClearMask(PCF8574_HandleTypeDef *h, uint8_t mask);

// Optional helper
bool PCF8574_IsInterruptAsserted(const PCF8574_HandleTypeDef *h);
```

Key points:

* **Addresses are 7‑bit**; the driver shifts to the HAL’s 8‑bit form.
* **`initial_shadow`** is written at init time (typical: `0xFF` → all pins input‑like/high).
* `GPIO_PIN_SET` means *release/high/input‑like*, `GPIO_PIN_RESET` means *drive LOW*.

---

## Usage Examples

### Minimal Init

```c
#include "pcf8574.h"

static PCF8574_HandleTypeDef hpcf;

void PCF_AppInit(void)
{
    const uint8_t addr7 = 0x20;    // A2..A0 = 000
    PCF8574_Status st = PCF8574_Init(&hpcf, &hi2c1, addr7, 0xFFu, 100);
    if (st != PCF8574_OK) {
        Error_Handler();
    }
}
```

### Blink an LED

```c
// Assume LED is on P0 (through a resistor to VDD)
void PCF_Blink(void)
{
    // Drive LOW → LED on 
    (void)PCF8574_WritePin(&hpcf, 0, GPIO_PIN_RESET);
    HAL_Delay(200);

    // Release → LED off
    (void)PCF8574_WritePin(&hpcf, 0, GPIO_PIN_SET);
    HAL_Delay(200);
}
```

### Read an Input

```c
// Assume a push button pulls P1 to GND when pressed; external pull-up present.
void PCF_ReadButton(void)
{
    // Ensure P1 is released
    (void)PCF8574_WritePin(&hpcf, 1, GPIO_PIN_SET);

    uint8_t state = 0;
    if (PCF8574_ReadPin(&hpcf, 1, &state) == PCF8574_OK) {
        if (state == 0) {
            // Button pressed (line low)
        } else {
            // Button released
        }
    }
}
```

### Toggle and Masks

```c
// Toggle P2
(void)PCF8574_TogglePin(&hpcf, 2);

// Drive P3..P5 LOW together
(void)PCF8574_ClearMask(&hpcf, (uint8_t)((1u<<3)|(1u<<4)|(1u<<5)));

// Release P6..P7 (input-like/high)
(void)PCF8574_SetMask(&hpcf, (uint8_t)((1u<<6)|(1u<<7)));
```

### Multiple Expanders

```c
static PCF8574_HandleTypeDef h_io0; // 0x20
static PCF8574_HandleTypeDef h_io1; // 0x21

void PCF_TwoChipsInit(void)
{
    (void)PCF8574_Init(&h_io0, &hi2c1, 0x20, 0xFFu, 100);
    (void)PCF8574_Init(&h_io1, &hi2c1, 0x21, 0x00u, 100); // start with all LOW
}
```

### Optional INT Pin

The **INT** pin goes active when an input changes. It is open‑drain; use a pull‑up and connect to an EXTI‑capable pin. Example (polling style):

```c
// After init, fill these fields if you use INT\ nhpcf.int_gpio_port = GPIOB;
hpcf.int_gpio_pin  = GPIO_PIN_0;
hpcf.int_pol       = PCF8574_INT_ACTIVE_LOW;

void Loop(void)
{
    if (PCF8574_IsInterruptAsserted(&hpcf)) {
        uint8_t port = 0;
        if (PCF8574_ReadPort(&hpcf, &port) == PCF8574_OK) {
            // Handle change
        }
        // Clear condition by reading; chip releases INT after port read
    }
}
```

For **EXTI**, configure a falling‑edge (active‑low) interrupt and call `PCF8574_ReadPort()` in the ISR/deferral.

### FreeRTOS Tip

If multiple tasks talk to the same I²C or handle, guard calls with a mutex:

```c
// Pseudocode
xSemaphoreTake(i2cMutex, portMAX_DELAY);
PCF8574_Status st = PCF8574_WritePin(&hpcf, 0, GPIO_PIN_RESET);
xSemaphoreGive(i2cMutex);
```

---

## Error Handling

All functions return `PCF8574_Status`:

* `PCF8574_OK` – success
* `PCF8574_ERR_I2C` – HAL transmit/receive failed (bus error, timeout)
* `PCF8574_ERR_PARAM` – invalid pin or argument
* `PCF8574_ERR_NULL` – null handle or missing pointer

Recommended pattern:

```c
PCF8574_Status st = PCF8574_WritePort(&hpcf, 0xAA);
if (st != PCF8574_OK) {
    // log, retry, or signal fault
}
```

---

## Roadmap

* Debounce helper for inputs.
* Optional retry/back‑off on I²C errors.
* Thread‑safe wrapper macros for RTOS builds.
* Example EXTI project for INT‑driven reads.
* Examples will be added


## License

This project is released under the **MIT License**. See `LICENSE` for details.
