/* SPDX-License-Identifier: AGPL-3.0-or-later */
/* Bare-metal HAL for Seeed Studio XIAO nRF54L15 + Wio-LR2021 Expansion Board */

#include "hal_nrf54l15.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "nrf.h"

/* Pin definitions */
#define PIN_USR_BTN     0U  /* P0.00 (active low) */

#define PIN_LR_DIO8     4U  /* P1.04 (active high rising-edge IRQ from LR2021 DIO8) */
#define PIN_LR_BUSY     5U  /* P1.05 (active high BUSY from LR2021) */
#define PIN_LR_RST      6U  /* P1.06 (active low NRESET to LR2021) */
#define PIN_LR_NSS      7U  /* P1.07 (active low CS to LR2021) */
#define PIN_USB_RXD     8U  /* P1.08 (SAMD11/CMSIS-DAP bridge -> nRF54L15) */
#define PIN_USB_TXD     9U  /* P1.09 (nRF54L15 -> SAMD11/CMSIS-DAP bridge) */

#define PIN_LED0        0U  /* P2.00 (active low user LED) */
#define PIN_SPI_SCK     1U  /* P2.01 (XIAO D8) */
#define PIN_SPI_MOSI    2U  /* P2.02 (XIAO D10) */
#define PIN_SPI_MISO    4U  /* P2.04 (XIAO D9) */
#define PIN_CRSF_RXD    7U  /* P2.07 (XIAO D7 / Grove RX) */
#define PIN_CRSF_TXD    8U  /* P2.08 (XIAO D6 / Grove TX) */

#define PSEL_PORT_PIN(port, pin) (((uint32_t)(port) << 5U) | ((uint32_t)(pin) & 0x1FU))

/* Ring buffers for UARTE22 (USB) and UARTE21 (CRSF) */
#define UART_RX_RING_SIZE 1024U
#define UART_RX_RING_MASK (UART_RX_RING_SIZE - 1U)

typedef struct {
    volatile uint8_t  buf[UART_RX_RING_SIZE];
    volatile uint32_t head;
    volatile uint32_t tail;
    volatile uint8_t  dma_rx_byte;
} uart_rx_ring_t;

static uart_rx_ring_t g_usb_rx_ring;
static uart_rx_ring_t g_crsf_rx_ring;

static uint8_t g_usb_tx_dma_buf[256] __attribute__((aligned(4)));
static uint8_t g_crsf_tx_dma_buf[128] __attribute__((aligned(4)));
static uint8_t g_spi_tx_dma_buf[264] __attribute__((aligned(4)));
static uint8_t g_spi_rx_dma_buf[264] __attribute__((aligned(4)));

/* DIO8 ISR callback */
static volatile hal_dio_isr_cb_t g_dio_isr_cb = NULL;

/* Hardware Timer (TIMER20) state for ExpressLRS tick/tock FHSS engine */
static volatile uint32_t       g_hwtimer_interval_us = 4000U;
static volatile int32_t        g_hwtimer_phase_shift = 0;
static volatile bool           g_hwtimer_running     = false;
static volatile bool           g_hwtimer_is_tick     = false;
static volatile hal_timer_cb_t g_hwtimer_tick_cb     = NULL;
static volatile hal_timer_cb_t g_hwtimer_tock_cb     = NULL;

static inline void ring_push(uart_rx_ring_t *r, uint8_t b) {
    uint32_t next = (r->head + 1U) & UART_RX_RING_MASK;
    if (next != r->tail) {
        r->buf[r->head] = b;
        r->head = next;
    }
}

static inline int ring_pop(uart_rx_ring_t *r) {
    if (r->head == r->tail) {
        return -1;
    }
    uint8_t b = r->buf[r->tail];
    r->tail = (r->tail + 1U) & UART_RX_RING_MASK;
    return (int)b;
}

static inline size_t ring_count(const uart_rx_ring_t *r) {
    return (size_t)((r->head - r->tail) & UART_RX_RING_MASK);
}

static void uarte_init_instance(NRF_UARTE_Type *uarte,
                                uint32_t psel_txd,
                                uint32_t psel_rxd,
                                uint32_t baud_reg,
                                uart_rx_ring_t *rx_ring,
                                IRQn_Type irqn,
                                uint32_t irq_prio) {
    uarte->ENABLE = UARTE_ENABLE_ENABLE_Disabled;

    uarte->PSEL.TXD = psel_txd;
    uarte->PSEL.RXD = psel_rxd;
    uarte->PSEL.CTS = 0xFFFFFFFFUL;
    uarte->PSEL.RTS = 0xFFFFFFFFUL;

    uarte->CONFIG = 0UL; /* 8N1, no flow control */
    uarte->BAUDRATE = baud_reg;

    uarte->EVENTS_DMA.RX.END = 0UL;
    uarte->EVENTS_DMA.TX.END = 0UL;
    uarte->EVENTS_ERROR      = 0UL;
    uarte->ERRORSRC          = 0xFFFFFFFFUL;

    rx_ring->head = 0U;
    rx_ring->tail = 0U;

    uarte->DMA.RX.PTR    = (uint32_t)(uintptr_t)&rx_ring->dma_rx_byte;
    uarte->DMA.RX.MAXCNT = 1UL;

    uarte->INTENCLR = 0xFFFFFFFFUL;
    uarte->INTENSET = UARTE_INTEN_DMARXEND_Msk;

    uarte->ENABLE = UARTE_ENABLE_ENABLE_Enabled;

    NVIC_SetPriority(irqn, irq_prio);
    NVIC_ClearPendingIRQ(irqn);
    NVIC_EnableIRQ(irqn);

    uarte->TASKS_DMA.RX.START = 1UL;
}

static void uarte_write_blocking(NRF_UARTE_Type *uarte,
                                 uint8_t *dma_buf,
                                 size_t dma_cap,
                                 const uint8_t *data,
                                 size_t len) {
    size_t offset = 0U;
    while (offset < len) {
        size_t chunk = len - offset;
        if (chunk > dma_cap) {
            chunk = dma_cap;
        }
        memcpy(dma_buf, data + offset, chunk);
        uarte->EVENTS_DMA.TX.END = 0UL;
        uarte->DMA.TX.PTR        = (uint32_t)(uintptr_t)dma_buf;
        uarte->DMA.TX.MAXCNT     = (uint32_t)chunk;
        uarte->TASKS_DMA.TX.START = 1UL;
        while (uarte->EVENTS_DMA.TX.END == 0UL) {
            __NOP();
        }
        uarte->EVENTS_DMA.TX.END = 0UL;
        offset += chunk;
    }
}

void UARTE22_IRQHandler(void) {
    if (NRF_UARTE22_S->EVENTS_DMA.RX.END != 0UL) {
        NRF_UARTE22_S->EVENTS_DMA.RX.END = 0UL;
        uint8_t b = g_usb_rx_ring.dma_rx_byte;
        ring_push(&g_usb_rx_ring, b);
        NRF_UARTE22_S->DMA.RX.PTR    = (uint32_t)(uintptr_t)&g_usb_rx_ring.dma_rx_byte;
        NRF_UARTE22_S->DMA.RX.MAXCNT = 1UL;
        NRF_UARTE22_S->TASKS_DMA.RX.START = 1UL;
    }
    if (NRF_UARTE22_S->EVENTS_ERROR != 0UL) {
        NRF_UARTE22_S->EVENTS_ERROR = 0UL;
        NRF_UARTE22_S->ERRORSRC     = 0xFFFFFFFFUL;
    }
}

void UARTE21_IRQHandler(void) {
    if (NRF_UARTE21_S->EVENTS_DMA.RX.END != 0UL) {
        NRF_UARTE21_S->EVENTS_DMA.RX.END = 0UL;
        uint8_t b = g_crsf_rx_ring.dma_rx_byte;
        ring_push(&g_crsf_rx_ring, b);
        NRF_UARTE21_S->DMA.RX.PTR    = (uint32_t)(uintptr_t)&g_crsf_rx_ring.dma_rx_byte;
        NRF_UARTE21_S->DMA.RX.MAXCNT = 1UL;
        NRF_UARTE21_S->TASKS_DMA.RX.START = 1UL;
    }
    if (NRF_UARTE21_S->EVENTS_ERROR != 0UL) {
        NRF_UARTE21_S->EVENTS_ERROR = 0UL;
        NRF_UARTE21_S->ERRORSRC     = 0xFFFFFFFFUL;
    }
}

void GPIOTE20_0_IRQHandler(void) {
    if (NRF_GPIOTE20_S->EVENTS_IN[0] != 0UL) {
        NRF_GPIOTE20_S->EVENTS_IN[0] = 0UL;
        hal_dio_isr_cb_t cb = g_dio_isr_cb;
        if (cb != NULL) {
            cb();
        }
    }
}

void TIMER20_IRQHandler(void) {
    if (NRF_TIMER20_S->EVENTS_COMPARE[0] != 0UL) {
        NRF_TIMER20_S->EVENTS_COMPARE[0] = 0UL;
        if (!g_hwtimer_running) {
            return;
        }

        uint32_t half_interval = g_hwtimer_interval_us >> 1U;
        if (g_hwtimer_is_tick) {
            NRF_TIMER20_S->CC[0] += half_interval;
            g_hwtimer_is_tick = false;
            hal_timer_cb_t cb = g_hwtimer_tick_cb;
            if (cb != NULL) {
                cb();
            }
        } else {
            int32_t next_delay = (int32_t)half_interval + g_hwtimer_phase_shift;
            if (next_delay < 50) {
                next_delay = 50;
            }
            g_hwtimer_phase_shift = 0;
            NRF_TIMER20_S->CC[0] += (uint32_t)next_delay;
            g_hwtimer_is_tick = true;
            hal_timer_cb_t cb = g_hwtimer_tock_cb;
            if (cb != NULL) {
                cb();
            }
        }
    }
}

void hal_init(void) {
    /* 1. Start 32 MHz HFXO crystal oscillator with 16 pF internal load caps */
    NRF_OSCILLATORS_S->XOSC32M.CONFIG.INTCAP = 0x20UL;
    NRF_CLOCK_S->EVENTS_XOSTARTED = 0UL;
    NRF_CLOCK_S->TASKS_XOSTART    = 1UL;
    while (NRF_CLOCK_S->EVENTS_XOSTARTED == 0UL) {
        __NOP();
    }

    /* Enable high-speed pad bias for Port 2 (SPIM00 / UARTE21) */
    #if defined(NRF_GPIOHSPADCTRL_S)
    NRF_GPIOHSPADCTRL_S->BIAS |= 3UL;
    #endif

    /* 2. Start TIMER10 as 1 MHz (1 us) 32-bit free-running monotonic counter */
    NRF_TIMER10_S->TASKS_STOP  = 1UL;
    NRF_TIMER10_S->TASKS_CLEAR = 1UL;
    NRF_TIMER10_S->MODE        = 0UL; /* Timer mode */
    NRF_TIMER10_S->BITMODE     = 3UL; /* 32-bit width */
    NRF_TIMER10_S->PRESCALER   = 5UL; /* 32 MHz PCLK32M / 2^5 = 1 MHz */
    NRF_TIMER10_S->TASKS_START = 1UL;

    /* 3. Configure GPIOs */
    /* P2.00 LED0 (output, active low -> default high = off) */
    NRF_P2_S->OUTSET = (1UL << PIN_LED0);
    NRF_P2_S->PIN_CNF[PIN_LED0] =
        (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |
        (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);

    /* P0.00 USR_BTN (input with pull-up, active low) */
    NRF_P0_S->PIN_CNF[PIN_USR_BTN] =
        (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
        (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
        (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);

    /* LR2021 NSS (P1.07 output, default high) & RESET (P1.06 output, default high) */
    NRF_P1_S->OUTSET = (1UL << PIN_LR_NSS) | (1UL << PIN_LR_RST);
    NRF_P1_S->PIN_CNF[PIN_LR_NSS] =
        (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |
        (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);
    NRF_P1_S->PIN_CNF[PIN_LR_RST] =
        (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |
        (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);

    /* LR2021 BUSY (P1.05 input, pull-up) */
    NRF_P1_S->PIN_CNF[PIN_LR_BUSY] =
        (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
        (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
        (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);

    /* LR2021 DIO8 IRQ (P1.04 input, pull-down) */
    NRF_P1_S->PIN_CNF[PIN_LR_DIO8] =
        (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
        (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
        (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos);

    /* 4. Configure UARTE22 (USB CDC bridge on P1.09 TXD, P1.08 RXD @ 115200 baud) */
    NRF_P1_S->OUTSET = (1UL << PIN_USB_TXD);
    NRF_P1_S->PIN_CNF[PIN_USB_TXD] =
        (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |
        (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);
    NRF_P1_S->PIN_CNF[PIN_USB_RXD] =
        (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
        (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
        (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);

    uarte_init_instance(
        NRF_UARTE22_S,
        PSEL_PORT_PIN(1U, PIN_USB_TXD),
        PSEL_PORT_PIN(1U, PIN_USB_RXD),
        0x01D60000UL, /* 115200 baud */
        &g_usb_rx_ring,
        UARTE22_IRQn,
        0U
    );

    /* 5. Configure UARTE21 (External Grove CRSF on P2.08 TXD, P2.07 RXD @ 420000 baud) */
    NRF_P2_S->OUTSET = (1UL << PIN_CRSF_TXD);
    NRF_P2_S->PIN_CNF[PIN_CRSF_TXD] =
        (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |
        (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos);
    NRF_P2_S->PIN_CNF[PIN_CRSF_RXD] =
        (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
        (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
        (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);

    uarte_init_instance(
        NRF_UARTE21_S,
        PSEL_PORT_PIN(2U, PIN_CRSF_TXD),
        PSEL_PORT_PIN(2U, PIN_CRSF_RXD),
        0x06B85000UL, /* 420000 baud (420000 * 2^32 / 16000000 = 0x06B851EB) */
        &g_crsf_rx_ring,
        UARTE21_IRQn,
        0U
    );

    /* 6. Configure SPIM00 for LR2021 (P2.01 SCK, P2.02 MOSI, P2.04 MISO @ 8 MHz) */
    NRF_SPIM00_S->ENABLE = SPIM_ENABLE_ENABLE_Disabled;

    NRF_P2_S->OUTCLR = (1UL << PIN_SPI_SCK) | (1UL << PIN_SPI_MOSI);
    NRF_P2_S->PIN_CNF[PIN_SPI_SCK] =
        (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |
        (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
        (GPIO_PIN_CNF_DRIVE0_E0 << GPIO_PIN_CNF_DRIVE0_Pos) |
        (GPIO_PIN_CNF_DRIVE1_E1 << GPIO_PIN_CNF_DRIVE1_Pos);
    NRF_P2_S->PIN_CNF[PIN_SPI_MOSI] =
        (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |
        (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
        (GPIO_PIN_CNF_DRIVE0_E0 << GPIO_PIN_CNF_DRIVE0_Pos) |
        (GPIO_PIN_CNF_DRIVE1_E1 << GPIO_PIN_CNF_DRIVE1_Pos);
    NRF_P2_S->PIN_CNF[PIN_SPI_MISO] =
        (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
        (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
        (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos);

    NRF_SPIM00_S->PSEL.SCK  = PSEL_PORT_PIN(2U, PIN_SPI_SCK);
    NRF_SPIM00_S->PSEL.MOSI = PSEL_PORT_PIN(2U, PIN_SPI_MOSI);
    NRF_SPIM00_S->PSEL.MISO = PSEL_PORT_PIN(2U, PIN_SPI_MISO);
    NRF_SPIM00_S->PSEL.CSN  = 0xFFFFFFFFUL; /* Manual NSS on P1.07 */

    NRF_SPIM00_S->CONFIG    = 0UL;  /* Mode 0 (CPOL=0, CPHA=0), MSB first */
    NRF_SPIM00_S->PRESCALER = 16UL; /* 128 MHz / 16 = 8 MHz */
    NRF_SPIM00_S->ORC       = 0x00UL;
    NRF_SPIM00_S->ENABLE    = SPIM_ENABLE_ENABLE_Enabled;

    /* 7. Configure GPIOTE20 channel 0 for rising-edge IRQ on P1.04 (LR2021 DIO8) */
    NRF_GPIOTE20_S->INTENCLR0 = 0xFFFFFFFFUL;
    NRF_GPIOTE20_S->EVENTS_IN[0] = 0UL;
    NRF_GPIOTE20_S->CONFIG[0] =
        (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) |
        (PIN_LR_DIO8 << GPIOTE_CONFIG_PSEL_Pos) |
        (1UL << GPIOTE_CONFIG_PORT_Pos) |
        (GPIOTE_CONFIG_POLARITY_LoToHi << GPIOTE_CONFIG_POLARITY_Pos);

    NVIC_SetPriority(GPIOTE20_0_IRQn, 1U);
    NVIC_ClearPendingIRQ(GPIOTE20_0_IRQn);
    NVIC_EnableIRQ(GPIOTE20_0_IRQn);

    /* 8. Configure TIMER20 (1 MHz, 32-bit) for ExpressLRS hwTimer */
    NRF_TIMER20_S->TASKS_STOP  = 1UL;
    NRF_TIMER20_S->TASKS_CLEAR = 1UL;
    NRF_TIMER20_S->MODE        = 0UL;
    NRF_TIMER20_S->BITMODE     = 3UL;
    NRF_TIMER20_S->PRESCALER   = 4UL; /* 16 MHz / 2^4 = 1 MHz */
    NRF_TIMER20_S->INTENCLR    = 0xFFFFFFFFUL;
    NVIC_SetPriority(TIMER20_IRQn, 1U);
    NVIC_ClearPendingIRQ(TIMER20_IRQn);
    NVIC_EnableIRQ(TIMER20_IRQn);
}

uint32_t hal_micros(void) {
    NRF_TIMER10_S->TASKS_CAPTURE[0] = 1UL;
    return NRF_TIMER10_S->CC[0];
}

void hal_get_device_id(uint8_t out_id[8]) {
    uint32_t id0 = NRF_FICR_NS->INFO.DEVICEID[0];
    uint32_t id1 = NRF_FICR_NS->INFO.DEVICEID[1];
    out_id[0] = (uint8_t)(id0 & 0xFFU);
    out_id[1] = (uint8_t)((id0 >> 8U) & 0xFFU);
    out_id[2] = (uint8_t)((id0 >> 16U) & 0xFFU);
    out_id[3] = (uint8_t)((id0 >> 24U) & 0xFFU);
    out_id[4] = (uint8_t)(id1 & 0xFFU);
    out_id[5] = (uint8_t)((id1 >> 8U) & 0xFFU);
    out_id[6] = (uint8_t)((id1 >> 16U) & 0xFFU);
    out_id[7] = (uint8_t)((id1 >> 24U) & 0xFFU);
}

uint32_t hal_millis(void) {
    return hal_micros() / 1000U;
}

void hal_delay_us(uint32_t us) {
    uint32_t start = hal_micros();
    while ((hal_micros() - start) < us) {
        __NOP();
    }
}

void hal_delay_ms(uint32_t ms) {
    while (ms > 0U) {
        hal_delay_us(1000U);
        ms--;
    }
}

void hal_led_set(bool on) {
    if (on) {
        NRF_P2_S->OUTCLR = (1UL << PIN_LED0);
    } else {
        NRF_P2_S->OUTSET = (1UL << PIN_LED0);
    }
}

void hal_led_toggle(void) {
    uint32_t mask = (1UL << PIN_LED0);
    if ((NRF_P2_S->OUT & mask) != 0UL) {
        NRF_P2_S->OUTCLR = mask;
    } else {
        NRF_P2_S->OUTSET = mask;
    }
}

bool hal_button_pressed(void) {
    return (NRF_P0_S->IN & (1UL << PIN_USR_BTN)) == 0UL;
}

void hal_usb_uart_write(const uint8_t *data, size_t len) {
    uarte_write_blocking(NRF_UARTE22_S, g_usb_tx_dma_buf, sizeof(g_usb_tx_dma_buf), data, len);
}

void hal_usb_uart_puts(const char *s) {
    hal_usb_uart_write((const uint8_t *)s, strlen(s));
}

void hal_usb_uart_printf(const char *fmt, ...) {
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) {
        size_t len = (size_t)n;
        if (len >= sizeof(buf)) {
            len = sizeof(buf) - 1U;
        }
        hal_usb_uart_write((const uint8_t *)buf, len);
    }
}

int hal_usb_uart_read_byte(void) {
    return ring_pop(&g_usb_rx_ring);
}

size_t hal_usb_uart_available(void) {
    return ring_count(&g_usb_rx_ring);
}

void hal_crsf_uart_set_baud(uint32_t baud_hz) {
    /* BAUDRATE register = round(baud_hz * 2^32 / 16000000), aligned to 0x1000 */
    uint64_t num = ((uint64_t)baud_hz << 32U) + 8000000ULL;
    uint32_t reg = (uint32_t)(num / 16000000ULL) & 0xFFFFF000UL;
    NRF_UARTE21_S->BAUDRATE = reg;
}

void hal_crsf_uart_write(const uint8_t *data, size_t len) {
    uarte_write_blocking(NRF_UARTE21_S, g_crsf_tx_dma_buf, sizeof(g_crsf_tx_dma_buf), data, len);
}

int hal_crsf_uart_read_byte(void) {
    return ring_pop(&g_crsf_rx_ring);
}

size_t hal_crsf_uart_available(void) {
    return ring_count(&g_crsf_rx_ring);
}

void hal_radio_nss_set(bool high) {
    if (high) {
        NRF_P1_S->OUTSET = (1UL << PIN_LR_NSS);
    } else {
        NRF_P1_S->OUTCLR = (1UL << PIN_LR_NSS);
    }
}

void hal_radio_rst_set(bool high) {
    if (high) {
        NRF_P1_S->OUTSET = (1UL << PIN_LR_RST);
    } else {
        NRF_P1_S->OUTCLR = (1UL << PIN_LR_RST);
    }
}

bool hal_radio_busy_read(void) {
    return (NRF_P1_S->IN & (1UL << PIN_LR_BUSY)) != 0UL;
}

bool hal_radio_dio_read(void) {
    return (NRF_P1_S->IN & (1UL << PIN_LR_DIO8)) != 0UL;
}

void hal_radio_reset(void) {
    hal_radio_nss_set(true);
    hal_radio_rst_set(false);
    hal_delay_ms(2U);
    hal_radio_rst_set(true);
    /* LR2021 internal boot takes ~230 ms; wait 260 ms then poll BUSY low */
    hal_delay_ms(260U);
    (void)hal_radio_wait_on_busy(100000U);
}

bool hal_radio_wait_on_busy(uint32_t timeout_us) {
    uint32_t start = hal_micros();
    while (hal_radio_busy_read()) {
        if (timeout_us != 0U && (hal_micros() - start) > timeout_us) {
            return false;
        }
    }
    return true;
}

void hal_radio_spi_transfer(uint8_t *inout_buf, size_t len) {
    if (len == 0U || len > sizeof(g_spi_tx_dma_buf)) {
        return;
    }

    memcpy(g_spi_tx_dma_buf, inout_buf, len);
    memset(g_spi_rx_dma_buf, 0, len);

    hal_radio_nss_set(false);

    NRF_SPIM00_S->EVENTS_END    = 0UL;
    NRF_SPIM00_S->DMA.TX.PTR    = (uint32_t)(uintptr_t)g_spi_tx_dma_buf;
    NRF_SPIM00_S->DMA.TX.MAXCNT = (uint32_t)len;
    NRF_SPIM00_S->DMA.RX.PTR    = (uint32_t)(uintptr_t)g_spi_rx_dma_buf;
    NRF_SPIM00_S->DMA.RX.MAXCNT = (uint32_t)len;

    NRF_SPIM00_S->TASKS_START = 1UL;
    while (NRF_SPIM00_S->EVENTS_END == 0UL) {
        __NOP();
    }
    NRF_SPIM00_S->EVENTS_END = 0UL;

    hal_radio_nss_set(true);
    /* Allow t_CSH and LR2021 NSS-rising-to-BUSY-rising propagation (~600ns-1.5us) */
    hal_delay_us(2U);

    memcpy(inout_buf, g_spi_rx_dma_buf, len);
}

void hal_radio_set_dio_isr(hal_dio_isr_cb_t cb) {
    g_dio_isr_cb = cb;
}

void hal_radio_irq_enable(bool enable) {
    if (enable) {
        NRF_GPIOTE20_S->EVENTS_IN[0] = 0UL;
        NRF_GPIOTE20_S->INTENSET0    = 1UL;
    } else {
        NRF_GPIOTE20_S->INTENCLR0    = 1UL;
    }
}

void hal_hwtimer_init(uint32_t interval_us, hal_timer_cb_t tick_cb, hal_timer_cb_t tock_cb) {
    g_hwtimer_interval_us = interval_us;
    g_hwtimer_tick_cb     = tick_cb;
    g_hwtimer_tock_cb     = tock_cb;
    g_hwtimer_phase_shift = 0;
}

void hal_hwtimer_start(void) {
    if (g_hwtimer_running) {
        return;
    }
    NRF_TIMER20_S->TASKS_STOP        = 1UL;
    NRF_TIMER20_S->TASKS_CLEAR       = 1UL;
    NRF_TIMER20_S->EVENTS_COMPARE[0] = 0UL;
    g_hwtimer_is_tick                = false;
    g_hwtimer_phase_shift            = 0;
    NRF_TIMER20_S->CC[0]             = (g_hwtimer_interval_us >> 1U);
    NRF_TIMER20_S->INTENSET          = (1UL << 16U); /* COMPARE[0] interrupt */
    g_hwtimer_running                = true;
    NRF_TIMER20_S->TASKS_START       = 1UL;
}

void hal_hwtimer_anchor_tock_in_us(uint32_t delay_us) {
    if (delay_us < 30U) {
        delay_us = 30U;
    }
    NRF_TIMER20_S->TASKS_STOP        = 1UL;
    NRF_TIMER20_S->TASKS_CLEAR       = 1UL;
    NRF_TIMER20_S->EVENTS_COMPARE[0] = 0UL;
    g_hwtimer_is_tick                = false; /* Next compare is TOCK */
    g_hwtimer_phase_shift            = 0;
    NRF_TIMER20_S->CC[0]             = delay_us;
    NRF_TIMER20_S->INTENSET          = (1UL << 16U);
    g_hwtimer_running                = true;
    NRF_TIMER20_S->TASKS_START       = 1UL;
}

void hal_hwtimer_stop(void) {
    g_hwtimer_running          = false;
    NRF_TIMER20_S->INTENCLR    = (1UL << 16U);
    NRF_TIMER20_S->TASKS_STOP  = 1UL;
    NRF_TIMER20_S->EVENTS_COMPARE[0] = 0UL;
}

void hal_hwtimer_update_interval(uint32_t interval_us) {
    g_hwtimer_interval_us = interval_us;
}

void hal_hwtimer_phase_shift(int32_t shift_us) {
    g_hwtimer_phase_shift = shift_us;
}

bool hal_hwtimer_is_running(void) {
    return g_hwtimer_running;
}
