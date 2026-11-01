/* SPDX-License-Identifier: AGPL-3.0-or-later */
/*
 * Bare-metal Semtech LR2021 driver for Seeed Studio Wio-LR2021 + XIAO nRF54L15.
 *
 * Adapted from ExpressLRS (GPL-3.0-or-later, src/lib/LR2021Driver/) and
 * Semtech Corporation LR20xx radio driver definitions (BSD-3-Clause).
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "LR2021_Regs.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LR2021_MOD_LORA = 0,
    LR2021_MOD_FLRC = 1,
    LR2021_MOD_FSK  = 2,
} lr2021_mod_type_t;

typedef struct {
    int8_t  rssi_dbm;
    int8_t  snr_q2_db; /* SNR * 4 (0.25 dB steps) */
    uint8_t status_raw[8];
} lr2021_pkt_stats_t;

typedef void (*lr2021_rx_done_cb_t)(const uint8_t *payload, uint8_t len, const lr2021_pkt_stats_t *stats);
typedef void (*lr2021_tx_done_cb_t)(void);

/* Initialize LR2021 on Wio-LR2021:
 * - Hardware reset
 * - Verify chip version == 0x0118
 * - Load Semtech PRAM patch (0x801000) & enable PRAM
 * - Configure 1.8V TCXO (2 ms startup), FS fallback, DCDC regmode
 * - Configure DIO8 (connected to XIAO D0 / P1.04) as active-high IRQ for RX_DONE | TX_DONE
 * - Calibrate all blocks + front-end for Sub-GHz (low_freq_hz) and 2.4 GHz (high_freq_hz)
 */
bool lr2021_begin(uint32_t low_freq_hz, uint32_t high_freq_hz, uint16_t *out_version);

/* Read chip version (expected 0x0118 for LR2021) */
uint16_t lr2021_get_version(void);

/* Read and clear system errors */
uint16_t lr2021_get_errors(void);
void lr2021_clear_errors(void);

/* Set operating mode (SLEEP, STDBY_RC, STDBY_XOSC, FS, RX_CONT, TX) */
void lr2021_set_mode(lr20xx_RadioOperatingModes_t mode);

/* Configure LoRa modulation & packet parameters (Sub-GHz or 2.4 GHz) */
void lr2021_config_lora(uint8_t bw,
                        uint8_t sf,
                        uint8_t cr,
                        uint32_t freq_hz,
                        uint16_t preamble_len,
                        bool invert_iq,
                        uint8_t payload_len,
                        bool explicit_header,
                        bool crc_en);

/* Configure 2.4 GHz FLRC modulation & packet parameters (ELRS D500 / F500 / F1000 or High-Speed Bridge) */
void lr2021_config_flrc(uint8_t bw_br,
                        uint8_t cr,
                        uint8_t bt,
                        uint32_t freq_hz,
                        uint16_t preamble_bits,
                        uint32_t sync_word,
                        uint16_t crc_seed,
                        uint8_t payload_len);

/* Configure GFSK modulation & packet parameters */
void lr2021_config_fsk(uint32_t bitrate_bps,
                       uint8_t bw_f,
                       uint32_t fdev_hz,
                       uint32_t freq_hz,
                       uint16_t preamble_bits,
                       uint8_t sync_word1,
                       uint8_t sync_word2,
                       uint8_t payload_len,
                       bool use_fec);

/* Set RF center frequency in Hz (optionally immediately entering continuous RX) */
void lr2021_set_frequency(uint32_t freq_hz, bool enter_rx);

/* Set output power in half-dBm steps (or dBm via helper) for current band */
void lr2021_set_tx_power_half_dbm(int8_t power_half_dbm, bool is_sub_ghz);
void lr2021_set_tx_power_dbm(int8_t power_dbm, bool is_sub_ghz);

/* Non-blocking transmit & continuous receive */
void lr2021_tx_nb(const uint8_t *data, uint8_t len);
void lr2021_rx_nb(void);

/* Instantaneous RSSI in dBm (while in RX mode) */
int8_t lr2021_get_rssi_inst_dbm(void);

/* Register RX_DONE and TX_DONE callbacks (invoked from DIO8 ISR or poll) */
void lr2021_set_callbacks(lr2021_rx_done_cb_t rx_cb, lr2021_tx_done_cb_t tx_cb);

/* Poll/process pending IRQ if needed outside ISR context */
uint32_t lr2021_get_and_clear_irq(void);
void lr2021_clear_irq(void);
void lr2021_decode_last_pkt_stats(lr2021_pkt_stats_t *out_stats);

/* Raw FIFO read for custom length */
void lr2021_read_rx_fifo(uint8_t *out_buf, uint8_t len);

/* Diagnostic counters */
uint32_t lr2021_get_irq_count(void);
uint32_t lr2021_get_rx_done_count(void);
uint32_t lr2021_get_tx_done_count(void);
void lr2021_reset_counters(void);

#ifdef __cplusplus
}
#endif
