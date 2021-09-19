/*
 * Copyright (c) 2020, Pycom Limited.
 *
 * This software is licensed under the GNU GPL version 3 or any
 * later version, with permitted additional terms. For more information
 * see the Pycom Licence v1.0 document supplied with this file, or
 * available at https://www.pycom.io/opensource/licensing
 */

#ifndef _INCLUDED_MPHAL_H_
#define _INCLUDED_MPHAL_H_

#if defined (LOPY) || defined(LOPY4) || defined(FIPY)
void HAL_set_tick_cb (void *cb);
#endif
void mp_hal_init(bool soft_reset);
void mp_hal_feed_watchdog(void);
void mp_hal_delay_us(uint32_t us);
int mp_hal_stdin_rx_chr(void);
void mp_hal_stdout_tx_str(const char *str);
void mp_hal_stdout_tx_strn(const char *str, uint32_t len);
void mp_hal_stdout_tx_strn_cooked(const char *str, uint32_t len);
uint32_t mp_hal_ticks_s(void);
uint32_t mp_hal_ticks_ms(void);
uint32_t mp_hal_ticks_us(void);
uint64_t mp_hal_ticks_ms_non_blocking(void);
uint64_t mp_hal_ticks_us_non_blocking(void);
void mp_hal_delay_ms(uint32_t delay);
void mp_hal_set_interrupt_char(int c);
void mp_hal_set_reset_char(int c);
void mp_hal_reset_safe_and_boot(bool reset);

// Wake up the main task if it is sleeping
void mp_hal_wake_main_task_from_isr(void);

// C-level pin HAL
#include "py/obj.h"
#include "machpin.h"
#include "gpio.h"

#define mp_hal_delay_us_fast(us) mp_hal_delay_us(us)
#define MP_HAL_PIN_FMT "%u"
#define mp_hal_pin_obj_t mp_obj_t
#define mp_hal_get_pin_obj(pin) (pin)
#define mp_hal_pin_name(p) (p)

#define mp_hal_pin_read(p) pin_get_value(pin_find(p))
#define mp_hal_pin_write(p, v) do { \
		pin_obj_t *npin = pin_find(p); \
		npin->value = v; \
		pin_set_value(npin); \
	} while (0)
#define mp_hal_pin_od_low(pin) do { \
		pin_obj_t *npin = pin_find(pin); \
		npin->value = 0; \
		pin_set_value(npin); \
	} while (0)
#define mp_hal_pin_od_high(pin) do { \
		pin_obj_t *npin = pin_find(pin); \
		npin->value = 1; \
		pin_set_value(npin); \
	} while (0)
#define mp_hal_pin_open_drain(pin) do { \
		pin_config (pin, -1, -1, GPIO_MODE_INPUT_OUTPUT_OD, MACHPIN_PULL_UP, 1); \
	} while (0)

#endif // _INCLUDED_MPHAL_H_
