/* SPDX-License-Identifier: AGPL-3.0-or-later */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*hal_dio_isr_cb_t)(void);
typedef void (*hal_timer_cb_t)(void);

/* Initialize clocks (128 MHz CPU, 32 MHz HFXO), timers, GPIOs, UARTE22 (USB), UARTE21 (CRSF), SPIM00 (LR2021) */
void hal_init(void);

/* Read factory-programmed 64-bit hardware device ID from FICR.INFO.DEVICEID[0..1] */
void hal_get_device_id(uint8_t out_id[8]);

/* Monotonic 1 MHz (1 us) hardware clock on TIMER10 */
uint32_t hal_micros(void);
uint32_t hal_millis(void);
void hal_delay_us(uint32_t us);
void hal_delay_ms(uint32_t ms);

/* Board LED (P2.00, active low) & User Button (P0.00, active low) */
void hal_led_set(bool on);
void hal_led_toggle(void);
bool hal_button_pressed(void);

/* USB CDC Console UART (UARTE22 on P1.09 TX / P1.08 RX @ 115200 baud) */
void hal_usb_uart_write(const uint8_t *data, size_t len);
void hal_usb_uart_puts(const char *s);
void hal_usb_uart_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
int  hal_usb_uart_read_byte(void); /* Returns -1 if empty */
size_t hal_usb_uart_available(void);

/* External Grove CRSF UART (UARTE21 on P2.08 TX / P2.07 RX @ 420000 baud) */
void hal_crsf_uart_set_baud(uint32_t baud_hz);
void hal_crsf_uart_write(const uint8_t *data, size_t len);
int  hal_crsf_uart_read_byte(void); /* Returns -1 if empty */
size_t hal_crsf_uart_available(void);

/* LR2021 Radio Control & SPI (SPIM00 on P2.01 SCK, P2.02 MOSI, P2.04 MISO; P1.07 NSS, P1.06 RST, P1.05 BUSY, P1.04 DIO8) */
void hal_radio_nss_set(bool high);
void hal_radio_rst_set(bool high);
bool hal_radio_busy_read(void);
bool hal_radio_dio_read(void);
void hal_radio_reset(void);
bool hal_radio_wait_on_busy(uint32_t timeout_us);

/* Full-duplex SPI transfer with manual NSS assertion around the transaction */
void hal_radio_spi_transfer(uint8_t *inout_buf, size_t len);

/* DIO8 (P1.04) rising-edge interrupt registration */
void hal_radio_set_dio_isr(hal_dio_isr_cb_t cb);
void hal_radio_irq_enable(bool enable);

/* ExpressLRS Hardware Timer (TIMER20 @ 1 MHz) with tick/tock half-periods and phaseShift */
void hal_hwtimer_init(uint32_t interval_us, hal_timer_cb_t tick_cb, hal_timer_cb_t tock_cb);
void hal_hwtimer_start(void);
void hal_hwtimer_anchor_tock_in_us(uint32_t delay_us);
void hal_hwtimer_stop(void);
void hal_hwtimer_update_interval(uint32_t interval_us);
void hal_hwtimer_phase_shift(int32_t shift_us);
bool hal_hwtimer_is_running(void);

#ifdef __cplusplus
}
#endif
