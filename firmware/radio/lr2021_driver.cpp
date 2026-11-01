/* SPDX-License-Identifier: AGPL-3.0-or-later */
/*
 * Bare-metal Semtech LR2021 driver for Seeed Studio Wio-LR2021 + XIAO nRF54L15.
 *
 * Adapted from ExpressLRS (GPL-3.0-or-later, src/lib/LR2021Driver/) and
 * Semtech Corporation LR20xx radio driver definitions (BSD-3-Clause).
 */

#include "lr2021_driver.h"
#include "LR2021PRAM.h"
#include "../hal/hal_nrf54l15.h"

#include <string.h>

#define LR20XX_PRAM_BASE_ADDRESS (0x00801000UL)
#define LR2021_DIO_IRQ_PIN       (8U) /* Wio-LR2021 routes LR2021 DIO8 to XIAO D0 (P1.04) */

static lr2021_mod_type_t   g_mod_type       = LR2021_MOD_LORA;
static bool                g_is_sub_ghz     = true;
static uint8_t             g_payload_len    = 8U;
static uint16_t            g_lora_preamble  = 8U;
static bool                g_lora_invert_iq = false;
static bool                g_lora_explicit  = false;
static bool                g_lora_crc_en    = false;
static bool                g_fsk_use_fec    = false;
static int8_t              g_pwr_lf_half_db = 20; /* +10 dBm default (20 * 0.5 dBm) */
static int8_t              g_pwr_hf_half_db = 20; /* +10 dBm default (20 * 0.5 dBm) */

static lr2021_rx_done_cb_t g_rx_done_cb     = NULL;
static lr2021_tx_done_cb_t g_tx_done_cb     = NULL;

static volatile uint32_t   g_irq_count      = 0U;
static volatile uint32_t   g_rx_done_count  = 0U;
static volatile uint32_t   g_tx_done_count  = 0U;

/* Hamming(7,4) + Interleaving for 2.4 GHz FSK 1000 Hz mode (8B -> 14B) */
static const uint8_t k_hamming_enc[16] = {
    0x00, 0x71, 0x62, 0x13, 0x54, 0x25, 0x36, 0x47,
    0x38, 0x49, 0x5A, 0x2B, 0x6C, 0x1D, 0x0E, 0x7F
};

static const uint8_t k_hamming_dec[128] = {
    0x00, 0x00, 0x00, 0x03, 0x00, 0x05, 0x0E, 0x07,
    0x00, 0x09, 0x0E, 0x0B, 0x0E, 0x0D, 0x0E, 0x0E,
    0x00, 0x03, 0x03, 0x03, 0x04, 0x0D, 0x06, 0x03,
    0x08, 0x0D, 0x0A, 0x03, 0x0D, 0x0D, 0x0E, 0x0D,
    0x00, 0x05, 0x02, 0x0B, 0x05, 0x05, 0x06, 0x05,
    0x08, 0x0B, 0x0B, 0x0B, 0x0C, 0x05, 0x0E, 0x0B,
    0x08, 0x01, 0x06, 0x03, 0x06, 0x05, 0x06, 0x06,
    0x08, 0x08, 0x08, 0x0B, 0x08, 0x0D, 0x06, 0x0F,
    0x00, 0x09, 0x02, 0x07, 0x04, 0x07, 0x07, 0x07,
    0x09, 0x09, 0x0A, 0x09, 0x0C, 0x09, 0x0E, 0x07,
    0x04, 0x01, 0x0A, 0x03, 0x04, 0x04, 0x04, 0x07,
    0x0A, 0x09, 0x0A, 0x0A, 0x04, 0x0D, 0x0A, 0x0F,
    0x02, 0x01, 0x02, 0x02, 0x0C, 0x05, 0x02, 0x07,
    0x0C, 0x09, 0x02, 0x0B, 0x0C, 0x0C, 0x0C, 0x0F,
    0x01, 0x01, 0x02, 0x01, 0x04, 0x01, 0x06, 0x0F,
    0x08, 0x01, 0x0A, 0x0F, 0x0C, 0x0F, 0x0F, 0x0F
};

static void fec_encode_8_to_14(const uint8_t *in8, uint8_t *out14) {
    uint8_t enc[16] = {0};
    memset(out14, 0, 14);
    for (uint8_t i = 0; i < 8; i++) {
        enc[i * 2 + 0] = k_hamming_enc[in8[i] & 0x0FU];
        enc[i * 2 + 1] = k_hamming_enc[in8[i] >> 4U];
    }
    for (uint8_t i = 0; i < 7; i++) {
        for (uint8_t j = 0; j < 8; j++) {
            out14[i * 2 + 0] |= (uint8_t)(((enc[j + 0] >> i) & 0x01U) << j);
            out14[i * 2 + 1] |= (uint8_t)(((enc[j + 8] >> i) & 0x01U) << j);
        }
    }
}

static void fec_decode_14_to_8(const uint8_t *in14, uint8_t *out8) {
    uint8_t enc[16] = {0};
    for (uint8_t i = 0; i < 8; i++) {
        for (uint8_t j = 0; j < 7; j++) {
            enc[i + 0] |= (uint8_t)(((in14[j * 2 + 0] >> i) & 0x01U) << j);
            enc[i + 8] |= (uint8_t)(((in14[j * 2 + 1] >> i) & 0x01U) << j);
        }
    }
    for (uint8_t i = 0; i < 8; i++) {
        out8[i]  = k_hamming_dec[enc[i * 2 + 0] & 0x7FU];
        out8[i] |= (uint8_t)(k_hamming_dec[enc[i * 2 + 1] & 0x7FU] << 4U);
    }
}

static inline uint32_t irq_save(void) {
    uint32_t primask;
    __asm volatile("mrs %0, primask\n\tcpsid i" : "=r"(primask) :: "memory");
    return primask;
}

static inline void irq_restore(uint32_t primask) {
    if ((primask & 1UL) == 0UL) {
        __asm volatile("cpsie i" ::: "memory");
    }
}

static uint16_t lr2021_write_cmd(uint16_t opcode, const uint8_t *args, uint16_t arg_len, uint32_t wait_us) {
    uint8_t buf[2 + 256];
    if (arg_len > 255U) {
        arg_len = 255U;
    }
    buf[0] = (uint8_t)(opcode >> 8U);
    buf[1] = (uint8_t)(opcode & 0xFFU);
    if (arg_len > 0U && args != NULL) {
        memcpy(&buf[2], args, arg_len);
    }

    uint32_t pm = irq_save();
    (void)hal_radio_wait_on_busy(wait_us);
    hal_radio_spi_transfer(buf, (size_t)arg_len + 2U);
    (void)hal_radio_wait_on_busy(wait_us);
    irq_restore(pm);
    return (uint16_t)(((uint16_t)buf[0] << 8U) | buf[1]);
}

static uint16_t lr2021_read_response(uint8_t *buf, uint16_t len, uint32_t wait_us) {
    uint32_t pm = irq_save();
    (void)hal_radio_wait_on_busy(wait_us);
    hal_radio_spi_transfer(buf, len);
    (void)hal_radio_wait_on_busy(wait_us);
    irq_restore(pm);
    return (uint16_t)(((uint16_t)buf[0] << 8U) | buf[1]);
}

static uint16_t lr2021_write_regmem32(uint32_t address, const uint32_t *words, uint8_t word_count) {
    uint8_t buf[5 + 32 * 4];
    if (word_count > 32U) {
        word_count = 32U;
    }
    buf[0] = (uint8_t)(LR2021_REGMEM_WRITE_REGMEM32_OC >> 8U);
    buf[1] = (uint8_t)(LR2021_REGMEM_WRITE_REGMEM32_OC & 0xFFU);
    buf[2] = (uint8_t)(address >> 16U);
    buf[3] = (uint8_t)(address >> 8U);
    buf[4] = (uint8_t)(address & 0xFFU);

    for (uint8_t i = 0U; i < word_count; i++) {
        uint32_t w = words[i];
        uint8_t *p = &buf[5U + i * 4U];
        p[0] = (uint8_t)(w >> 24U);
        p[1] = (uint8_t)(w >> 16U);
        p[2] = (uint8_t)(w >> 8U);
        p[3] = (uint8_t)(w & 0xFFU);
    }

    uint32_t pm = irq_save();
    (void)hal_radio_wait_on_busy(20000U);
    hal_radio_spi_transfer(buf, 5U + (size_t)word_count * 4U);
    (void)hal_radio_wait_on_busy(20000U);
    irq_restore(pm);
    return (uint16_t)(((uint16_t)buf[0] << 8U) | buf[1]);
}

static bool lr2021_load_patch_ram(void) {
    const uint32_t total_words = (uint32_t)(sizeof(pram_lr2021) / sizeof(uint32_t));
    const uint8_t  chunk_words = 12U;
    const uint32_t n_chunks    = total_words / chunk_words;

    for (uint32_t c = 0U; c < n_chunks; c++) {
        const uint32_t *src = &pram_lr2021[c * chunk_words];
        uint32_t addr = LR20XX_PRAM_BASE_ADDRESS + c * chunk_words * 4U;
        uint16_t ret = lr2021_write_regmem32(addr, src, chunk_words);
        if ((ret & 0x0400U) == 0U) {
            return false;
        }
    }

    uint8_t rem_words = (uint8_t)(total_words - n_chunks * chunk_words);
    if (rem_words > 0U) {
        const uint32_t *src = &pram_lr2021[n_chunks * chunk_words];
        uint32_t addr = LR20XX_PRAM_BASE_ADDRESS + n_chunks * chunk_words * 4U;
        uint16_t ret = lr2021_write_regmem32(addr, src, rem_words);
        if ((ret & 0x0400U) == 0U) {
            return false;
        }
    }

    uint8_t enable = 0x00U;
    uint16_t ret = lr2021_write_cmd(LR2021_PATCH_ENABLE_PRAM_OC, &enable, 1U, 20000U);
    return (ret & 0x0400U) != 0U;
}

static void lr2021_dio8_isr_handler(void) {
    g_irq_count++;
    uint32_t irq = lr2021_get_and_clear_irq();
    if ((irq & LR2021_IRQ_TX_DONE) != 0UL) {
        g_tx_done_count++;
        if (g_tx_done_cb != NULL) {
            g_tx_done_cb();
        }
    }
    if ((irq & LR2021_IRQ_RX_DONE) != 0UL) {
        g_rx_done_count++;
        lr2021_pkt_stats_t stats;
        lr2021_decode_last_pkt_stats(&stats);

        if (g_mod_type == LR2021_MOD_LORA && g_lora_explicit) {
            /* Check hardware LoRa header error (bit 9 = 0x200) and CRC error (bit 22 = 0x400000) */
            if ((irq & (1UL << 9)) != 0UL || (g_lora_crc_en && (irq & (1UL << 22)) != 0UL)) {
                (void)lr2021_write_cmd(LR2021_SYSTEM_CLEAR_RX_FIFO_OC, NULL, 0U, 5000U);
                return;
            }
            uint8_t rx_len = stats.status_raw[3];
            if (rx_len == 0U) {
                (void)lr2021_write_cmd(LR2021_SYSTEM_CLEAR_RX_FIFO_OC, NULL, 0U, 5000U);
                return;
            }
            uint8_t fifo_buf[258];
            memset(fifo_buf, 0, (size_t)rx_len + 2U);
            fifo_buf[0] = (uint8_t)(LR2021_RADIO_READ_RX_FIFO >> 8U);
            fifo_buf[1] = (uint8_t)(LR2021_RADIO_READ_RX_FIFO & 0xFFU);
            (void)lr2021_read_response(fifo_buf, (uint16_t)rx_len + 2U, 10000U);
            (void)lr2021_write_cmd(LR2021_SYSTEM_CLEAR_RX_FIFO_OC, NULL, 0U, 5000U);

            if (g_rx_done_cb != NULL) {
                g_rx_done_cb(&fifo_buf[2], rx_len, &stats);
            }
            return;
        }

        uint8_t raw_len = (g_mod_type == LR2021_MOD_FSK && g_fsk_use_fec) ? 14U : g_payload_len;
        uint8_t fifo_buf[66];
        memset(fifo_buf, 0, sizeof(fifo_buf));
        fifo_buf[0] = (uint8_t)(LR2021_RADIO_READ_RX_FIFO >> 8U);
        fifo_buf[1] = (uint8_t)(LR2021_RADIO_READ_RX_FIFO & 0xFFU);
        (void)lr2021_read_response(fifo_buf, (uint16_t)(raw_len + 2U), 5000U);

        uint8_t decoded[64];
        if (g_mod_type == LR2021_MOD_FSK && g_fsk_use_fec) {
            fec_decode_14_to_8(&fifo_buf[2], decoded);
        } else {
            memcpy(decoded, &fifo_buf[2], g_payload_len);
        }

        (void)lr2021_write_cmd(LR2021_SYSTEM_CLEAR_RX_FIFO_OC, NULL, 0U, 5000U);

        if (g_rx_done_cb != NULL) {
            g_rx_done_cb(decoded, g_payload_len, &stats);
        }
    }
}

uint16_t lr2021_get_version(void) {
    uint8_t resp[4] = {0};
    (void)lr2021_write_cmd(LR2021_SYSTEM_GET_VERSION_OC, NULL, 0U, 10000U);
    (void)lr2021_read_response(resp, sizeof(resp), 10000U);
    return (uint16_t)(((uint16_t)resp[2] << 8U) | resp[3]);
}

uint16_t lr2021_get_errors(void) {
    uint8_t resp[4] = {0};
    (void)lr2021_write_cmd(LR2021_SYSTEM_GET_ERRORS_OC, NULL, 0U, 10000U);
    (void)lr2021_read_response(resp, sizeof(resp), 10000U);
    return (uint16_t)(((uint16_t)resp[2] << 8U) | resp[3]);
}

void lr2021_clear_errors(void) {
    (void)lr2021_write_cmd(LR2021_SYSTEM_CLEAR_ERRORS_OC, NULL, 0U, 10000U);
}

bool lr2021_begin(uint32_t low_freq_hz, uint32_t high_freq_hz, uint16_t *out_version) {
    hal_radio_irq_enable(false);
    hal_radio_reset();

    uint16_t ver = lr2021_get_version();
    if (out_version != NULL) {
        *out_version = ver;
    }
    if (ver != 0x0118U) {
        return false;
    }

    if (!lr2021_load_patch_ram()) {
        return false;
    }

    lr2021_clear_errors();
    lr2021_set_mode(LR2021_MODE_STDBY_RC);

    /* Apply Semtech LR2021 rev-B system register init (0x0080004C = 0x002CCCCC)
     * and LFCLK = RC (0x00) from Wio-LR2021 (LR2021EVK1XBS1_XIAO) BSP.
     * Note: Wio-LR2021 uses a passive 32 MHz crystal (has_tcxo = false);
     * do NOT call SET_TCXO_MODE (0x0120), which disables the passive crystal!
     */
    const uint32_t sys_reg_val = 0x002CCCCCUL;
    (void)lr2021_write_regmem32(0x0080004CUL, &sys_reg_val, 1U);
    const uint8_t lfclk_cfg = 0x00U;
    (void)lr2021_write_cmd(0x0118U, &lfclk_cfg, 1U, 10000U);

    /* Fallback mode = FS (0x03) for fast RX<->TX turnarounds */
    const uint8_t fb_mode = LR2021_RADIO_FALLBACK_FS;
    (void)lr2021_write_cmd(LR2021_RADIO_SET_RX_TX_FALLBACK_MODE_OC, &fb_mode, 1U, 10000U);

    /* Default RX/TX timeout: continuous RX (0xFFFFFF), disabled TX timeout (0x000000) */
    const uint8_t timeouts[6] = {0xFFU, 0xFFU, 0xFFU, 0x00U, 0x00U, 0x00U};
    (void)lr2021_write_cmd(LR2021_RADIO_SET_DEFAULT_RX_TX_TIMEOUT_OC, timeouts, sizeof(timeouts), 10000U);

    /* Wio-LR2021 uses a switchless direct-tie RF front-end and routes DIO8 to XIAO D0 (P1.04) */
    const uint8_t dio_fn[2] = {LR2021_DIO_IRQ_PIN, 0x10U}; /* Function 0x1 = IRQ, sleep drive = None */
    (void)lr2021_write_cmd(LR2021_SYSTEM_SET_DIO_FUNCTION_OC, dio_fn, sizeof(dio_fn), 10000U);

    const uint32_t irq_mask = (uint32_t)LR2021_IRQ_RX_DONE | (uint32_t)LR2021_IRQ_TX_DONE;
    const uint8_t dio_irq[5] = {
        LR2021_DIO_IRQ_PIN,
        (uint8_t)(irq_mask >> 24U),
        (uint8_t)(irq_mask >> 16U),
        (uint8_t)(irq_mask >> 8U),
        (uint8_t)(irq_mask & 0xFFU)
    };
    (void)lr2021_write_cmd(LR2021_SYSTEM_SET_DIOIRQPARAMS_OC, dio_irq, sizeof(dio_irq), 10000U);

    /* Enable DCDC regulator (0x02) */
    const uint8_t reg_mode = 0x02U;
    (void)lr2021_write_cmd(LR2021_SYSTEM_SET_REGMODE_OC, &reg_mode, 1U, 10000U);

    /* Calibrate all internal blocks (0x7F) */
    const uint8_t cal_mask = 0x7FU;
    (void)lr2021_write_cmd(LR2021_SYSTEM_CALIBRATE_OC, &cal_mask, 1U, 50000U);

    /* Calibrate Sub-GHz and 2.4 GHz front-ends */
    const uint16_t lf_step = (uint16_t)(low_freq_hz / 4000000UL);
    const uint16_t hf_step = (uint16_t)((high_freq_hz / 4000000UL) | 0x8000U);
    const uint8_t cal_fe[4] = {
        (uint8_t)(lf_step >> 8U),
        (uint8_t)(lf_step & 0xFFU),
        (uint8_t)(hf_step >> 8U),
        (uint8_t)(hf_step & 0xFFU)
    };
    (void)lr2021_write_cmd(LR2021_SYSTEM_CALIBRATE_FRONTEND_OC, cal_fe, sizeof(cal_fe), 100000U);

    lr2021_clear_errors();
    lr2021_clear_irq();

    hal_radio_set_dio_isr(lr2021_dio8_isr_handler);
    hal_radio_irq_enable(true);
    return true;
}

void lr2021_set_mode(lr20xx_RadioOperatingModes_t mode) {
    uint8_t buf[5] = {0};
    switch (mode) {
        case LR2021_MODE_SLEEP:
            (void)lr2021_write_cmd(LR2021_SYSTEM_SET_SLEEP_OC, buf, 5U, 10000U);
            break;
        case LR2021_MODE_STDBY_RC:
            buf[0] = 0x00U;
            (void)lr2021_write_cmd(LR2021_SYSTEM_SET_STANDBY_OC, buf, 1U, 10000U);
            break;
        case LR2021_MODE_STDBY_XOSC:
            buf[0] = 0x01U;
            (void)lr2021_write_cmd(LR2021_SYSTEM_SET_STANDBY_OC, buf, 1U, 10000U);
            break;
        case LR2021_MODE_FS:
            (void)lr2021_write_cmd(LR2021_SYSTEM_SET_FS_OC, NULL, 0U, 10000U);
            break;
        case LR2021_MODE_RX_CONT:
            (void)lr2021_write_cmd(LR2021_RADIO_SET_RX_OC, NULL, 0U, 10000U);
            break;
        case LR2021_MODE_TX:
            (void)lr2021_write_cmd(LR2021_RADIO_SET_TX_OC, NULL, 0U, 10000U);
            break;
        default:
            break;
    }
}

static void lr2021_apply_rx_path_and_pa(uint32_t freq_hz) {
    g_is_sub_ghz = (freq_hz < 1000000000UL);

    /* Match factory ral_lr20xx_set_rf_freq (0x39530) sequence:
     * 1. SET_RF_FREQUENCY (0x0200)
     * 2. CALIBRATE_FRONTEND (0x0123)
     * 3. SET_RX_PATH (0x0201)
     * 4. SET_PA_CFG (0x0202) + SET_TX_PARAMS (0x0203)
     */
    lr2021_set_frequency(freq_hz, false);

    const uint16_t fe_step = (uint16_t)(((freq_hz + 3999999UL) / 4000000UL) |
                                        (g_is_sub_ghz ? 0x0000U : 0x8000U));
    const uint8_t cal_fe[2] = {
        (uint8_t)(fe_step >> 8U),
        (uint8_t)(fe_step & 0xFFU)
    };
    if (g_is_sub_ghz) {
        const uint8_t rx_path[2] = {0x00U, 0x00U}; /* LF path */
        (void)lr2021_write_cmd(LR2021_RADIO_SET_RX_PATH_OC, rx_path, sizeof(rx_path), 10000U);
        (void)lr2021_write_cmd(LR2021_SYSTEM_CALIBRATE_FRONTEND_OC, cal_fe, sizeof(cal_fe), 50000U);
        lr2021_set_frequency(freq_hz, false);

        /* Factory LR2021EVK1XBS1_XIAO LF PA config (+10..+22 dBm safe table) */
        const uint8_t pa_cfg[3] = {
            0x01U, /* (pa_sel=0 << 7) | pa_supply=1 */
            (uint8_t)((3U << 4U) | 1U),
            16U
        };
        (void)lr2021_write_cmd(LR2021_RADIO_SET_PA_CFG_OC, pa_cfg, sizeof(pa_cfg), 10000U);

        const uint8_t tx_params[2] = {(uint8_t)g_pwr_lf_half_db, LR2021_RADIO_RAMP_48_US};
        (void)lr2021_write_cmd(LR2021_RADIO_SET_TX_PARAMS_OC, tx_params, sizeof(tx_params), 10000U);
    } else {
        const uint8_t rx_path[2] = {0x01U, 0x07U}; /* HF path, factory boost=7 */
        (void)lr2021_write_cmd(LR2021_RADIO_SET_RX_PATH_OC, rx_path, sizeof(rx_path), 10000U);
        (void)lr2021_write_cmd(LR2021_SYSTEM_CALIBRATE_FRONTEND_OC, cal_fe, sizeof(cal_fe), 50000U);
        lr2021_set_frequency(freq_hz, false);

        if (g_pwr_hf_half_db > 13) {
            g_pwr_hf_half_db = 13; /* Safe +10 dBm factory HF table entry */
        }
        const uint8_t pa_cfg[3] = {
            0x81U, /* (pa_sel=1 << 7) | pa_supply=1 */
            (uint8_t)((6U << 4U) | 7U),
            0x1BU
        };
        (void)lr2021_write_cmd(LR2021_RADIO_SET_PA_CFG_OC, pa_cfg, sizeof(pa_cfg), 10000U);

        const uint8_t tx_params[2] = {(uint8_t)g_pwr_hf_half_db, LR2021_RADIO_RAMP_48_US};
        (void)lr2021_write_cmd(LR2021_RADIO_SET_TX_PARAMS_OC, tx_params, sizeof(tx_params), 10000U);
    }
}

void lr2021_config_lora(uint8_t bw,
                        uint8_t sf,
                        uint8_t cr,
                        uint32_t freq_hz,
                        uint16_t preamble_len,
                        bool invert_iq,
                        uint8_t payload_len,
                        bool explicit_header,
                        bool crc_en) {
    g_mod_type       = LR2021_MOD_LORA;
    g_payload_len    = payload_len;
    g_lora_preamble  = preamble_len;
    g_lora_invert_iq = invert_iq;
    g_lora_explicit  = explicit_header;
    g_lora_crc_en    = crc_en;
    g_is_sub_ghz     = (freq_hz < 1000000000UL);

    lr2021_set_mode(LR2021_MODE_STDBY_RC);

    const uint8_t pkt_type = LR2021_RADIO_PKT_TYPE_LORA;
    (void)lr2021_write_cmd(LR2021_RADIO_SET_PKT_TYPE_OC, &pkt_type, 1U, 10000U);

    lr2021_apply_rx_path_and_pa(freq_hz);

    /* Enable LDRO (0x01) automatically for high spreading factors (SF11/SF12 at <=125kHz, SF12 at 250kHz) */
    uint8_t ldro = 0U;
    if ((bw == LR2021_RADIO_LORA_BW_125 && sf >= 11U) ||
        (bw == LR2021_RADIO_LORA_BW_62  && sf >= 10U) ||
        (bw == LR2021_RADIO_LORA_BW_250 && sf >= 12U)) {
        ldro = 1U;
    }

    const uint8_t mod_params[2] = {
        (uint8_t)((sf << 4U) | (bw & 0x0FU)),
        (uint8_t)((cr << 4U) | ldro)
    };
    (void)lr2021_write_cmd(LR2021_RADIO_SET_LORA_MODULATION_PARAM_OC, mod_params, sizeof(mod_params), 10000U);

    /* SX1276 implicit-header SF6 compatibility register (0x00F30A14 bits [19:18]):
     * Must be 0b10 (0x08) for Sub-GHz SF6 implicit header, and 0b00 (0x00) for all
     * 2.4 GHz LoRa and non-SF6 modes so 2.4 GHz SF6 (e.g. 2G4_250HZ) demodulates properly.
     */
    {
        uint8_t compat_byte = (g_is_sub_ghz && sf == LR2021_RADIO_LORA_SF6 && !explicit_header) ? 0x08U : 0x00U;
        const uint8_t wrbuf[11] = {
            (uint8_t)(LR20XX_WORKAROUND_LORA_SX1276_COMPATIBILITY_REGISTER_ADDRESS >> 16U),
            (uint8_t)(LR20XX_WORKAROUND_LORA_SX1276_COMPATIBILITY_REGISTER_ADDRESS >> 8U),
            (uint8_t)(LR20XX_WORKAROUND_LORA_SX1276_COMPATIBILITY_REGISTER_ADDRESS & 0xFFU),
            (uint8_t)(LR20XX_WORKAROUND_LORA_SX1276_COMPATIBILITY_REGISTER_MASK >> 24U),
            (uint8_t)(LR20XX_WORKAROUND_LORA_SX1276_COMPATIBILITY_REGISTER_MASK >> 16U),
            (uint8_t)(LR20XX_WORKAROUND_LORA_SX1276_COMPATIBILITY_REGISTER_MASK >> 8U),
            (uint8_t)(LR20XX_WORKAROUND_LORA_SX1276_COMPATIBILITY_REGISTER_MASK & 0xFFU),
            0x00U, compat_byte, 0x00U, 0x00U
        };
        (void)lr2021_write_cmd(LR2021_REGMEM_WRITE_REGMEM32_MASK_OC, wrbuf, sizeof(wrbuf), 10000U);
    }

    /* Sub-GHz ELRS always uses standard IQ */
    uint8_t iq_bit = (!g_is_sub_ghz && invert_iq) ? LR2021_RADIO_LORA_IQ_INVERTED : LR2021_RADIO_LORA_IQ_STANDARD;
    uint8_t hdr_bit = explicit_header ? LR2021_LORA_PACKET_EXPLICIT : LR2021_LORA_PACKET_IMPLICIT;
    uint8_t crc_bit = crc_en ? 1U : 0U;

    const uint8_t pkt_params[4] = {
        (uint8_t)(preamble_len >> 8U),
        (uint8_t)(preamble_len & 0xFFU),
        payload_len,
        (uint8_t)((hdr_bit << 2U) | (crc_bit << 1U) | iq_bit)
    };
    (void)lr2021_write_cmd(LR2021_RADIO_SET_LORA_PACKET_PARAMS_OC, pkt_params, sizeof(pkt_params), 10000U);

    lr2021_clear_errors();
    lr2021_clear_irq();
    (void)lr2021_write_cmd(LR2021_SYSTEM_CLEAR_RX_FIFO_OC, NULL, 0U, 10000U);
    (void)lr2021_write_cmd(LR2021_SYSTEM_CLEAR_TX_FIFO_OC, NULL, 0U, 10000U);
}

void lr2021_set_lora_syncword(uint8_t syncword) {
    (void)lr2021_write_cmd(LR2021_RADIO_SET_LORA_SYNCWORD_OC, &syncword, 1U, 10000U);
}

uint8_t lr2021_lora_bw_from_hz(uint32_t bw_hz) {
    if (bw_hz <= 31250UL)  return LR2021_RADIO_LORA_BW_31;
    if (bw_hz <= 41670UL)  return LR2021_RADIO_LORA_BW_41;
    if (bw_hz <= 62500UL)  return LR2021_RADIO_LORA_BW_62;
    if (bw_hz <= 83340UL)  return LR2021_RADIO_LORA_BW_83;
    if (bw_hz <= 101563UL) return LR2021_RADIO_LORA_BW_101;
    if (bw_hz <= 125000UL) return LR2021_RADIO_LORA_BW_125;
    if (bw_hz <= 203125UL) return LR2021_RADIO_LORA_BW_200;
    if (bw_hz <= 250000UL) return LR2021_RADIO_LORA_BW_250;
    if (bw_hz <= 406250UL) return LR2021_RADIO_LORA_BW_400;
    if (bw_hz <= 500000UL) return LR2021_RADIO_LORA_BW_500;
    if (bw_hz <= 812500UL) return LR2021_RADIO_LORA_BW_800;
    return LR2021_RADIO_LORA_BW_1000;
}

void lr2021_config_flrc(uint8_t bw_br,
                        uint8_t cr,
                        uint8_t bt,
                        uint32_t freq_hz,
                        uint16_t preamble_bits,
                        uint32_t sync_word,
                        uint16_t crc_seed,
                        uint8_t payload_len) {
    g_mod_type    = LR2021_MOD_FLRC;
    g_payload_len = payload_len;
    g_is_sub_ghz  = (freq_hz < 1000000000UL);

    lr2021_set_mode(LR2021_MODE_STDBY_RC);

    const uint8_t pkt_type = LR2021_RADIO_PKT_TYPE_FLRC;
    (void)lr2021_write_cmd(LR2021_RADIO_SET_PKT_TYPE_OC, &pkt_type, 1U, 10000U);

    lr2021_apply_rx_path_and_pa(freq_hz);

    const uint8_t mod_params[2] = {
        bw_br,
        (uint8_t)((cr << 4U) | (bt & 0x0FU))
    };
    (void)lr2021_write_cmd(LR2021_RADIO_SET_FLRC_MODULATION_PARAMS_OC, mod_params, sizeof(mod_params), 10000U);

    uint8_t pbl_code = (preamble_bits >= 4U) ? (uint8_t)((preamble_bits / 4U) - 1U) : 0U;
    const uint8_t sw_idx = 1U;
    const uint8_t pkt_params[4] = {
        (uint8_t)((pbl_code << 2U) | LR2021_RADIO_FLRC_SYNC_32BITS),
        (uint8_t)((sw_idx << 6U) | (sw_idx << 3U) | LR2021_RADIO_FLRC_PAYLOAD_FIXED_LEN | LR2021_RADIO_FLRC_CRC_24BITS),
        0U,
        payload_len
    };
    (void)lr2021_write_cmd(LR2021_RADIO_SET_FLRC_PACKET_PARAMS_OC, pkt_params, sizeof(pkt_params), 10000U);

    const uint32_t poly = 0x005D6DCBUL;
    const uint8_t crc_params[8] = {
        (uint8_t)(poly >> 24U),
        (uint8_t)(poly >> 16U),
        (uint8_t)(poly >> 8U),
        (uint8_t)(poly & 0xFFU),
        0x00U,
        0xFFU,
        (uint8_t)(crc_seed >> 8U),
        (uint8_t)(crc_seed & 0xFFU)
    };
    (void)lr2021_write_cmd(LR2021_RADIO_SET_FSK_CRC_PARAMS_OC, crc_params, sizeof(crc_params), 10000U);

    uint8_t sw_buf[5] = {
        1U,
        (uint8_t)(sync_word >> 24U),
        (uint8_t)(sync_word >> 16U),
        (uint8_t)(sync_word >> 8U),
        (uint8_t)(sync_word & 0xFFU)
    };
    if ((sw_buf[1] == 0x8CU && sw_buf[2] == 0x38U) || (sw_buf[1] == 0x63U && sw_buf[2] == 0x0EU)) {
        uint8_t tmp = sw_buf[1];
        sw_buf[1] = sw_buf[2];
        sw_buf[2] = tmp;
    }
    (void)lr2021_write_cmd(LR2021_RADIO_SET_FLRC_SYNCWORD_OC, sw_buf, sizeof(sw_buf), 10000U);

    lr2021_clear_errors();
    lr2021_clear_irq();
    (void)lr2021_write_cmd(LR2021_SYSTEM_CLEAR_RX_FIFO_OC, NULL, 0U, 10000U);
    (void)lr2021_write_cmd(LR2021_SYSTEM_CLEAR_TX_FIFO_OC, NULL, 0U, 10000U);
}

void lr2021_config_fsk(uint32_t bitrate_bps,
                       uint8_t bw_f,
                       uint32_t fdev_hz,
                       uint32_t freq_hz,
                       uint16_t preamble_bits,
                       uint8_t sync_word1,
                       uint8_t sync_word2,
                       uint8_t payload_len,
                       bool use_fec) {
    g_mod_type    = LR2021_MOD_FSK;
    g_payload_len = payload_len;
    g_fsk_use_fec = use_fec;
    g_is_sub_ghz  = (freq_hz < 1000000000UL);

    lr2021_set_mode(LR2021_MODE_STDBY_RC);

    const uint8_t pkt_type = LR2021_RADIO_PKT_TYPE_FSK;
    (void)lr2021_write_cmd(LR2021_RADIO_SET_PKT_TYPE_OC, &pkt_type, 1U, 10000U);

    lr2021_apply_rx_path_and_pa(freq_hz);

    const uint8_t mod_params[9] = {
        (uint8_t)(bitrate_bps >> 24U),
        (uint8_t)(bitrate_bps >> 16U),
        (uint8_t)(bitrate_bps >> 8U),
        (uint8_t)(bitrate_bps & 0xFFU),
        LR2021_RADIO_GFSK_PULSE_SHAPE_OFF,
        bw_f,
        (uint8_t)(fdev_hz >> 16U),
        (uint8_t)(fdev_hz >> 8U),
        (uint8_t)(fdev_hz & 0xFFU)
    };
    (void)lr2021_write_cmd(LR2021_RADIO_SET_FSK_MODULATION_PARAMS_OC, mod_params, sizeof(mod_params), 10000U);

    uint8_t wire_len = use_fec ? 14U : payload_len;
    const uint8_t pkt_params[7] = {
        (uint8_t)(preamble_bits >> 8U),
        (uint8_t)(preamble_bits & 0xFFU),
        LR2021_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_8BITS,
        0x00U,
        0U,
        wire_len,
        (uint8_t)(LR2021_RADIO_GFSK_CRC_OFF | LR2021_RADIO_GFSK_DC_FREE_WHITENING)
    };
    (void)lr2021_write_cmd(LR2021_RADIO_SET_FSK_PACKET_PARAMS_OC, pkt_params, sizeof(pkt_params), 10000U);

    const uint8_t whitening[2] = {0x01U, 0x00U};
    (void)lr2021_write_cmd(LR2021_RADIO_SET_FSK_WHITENING_PARAMS_OC, whitening, sizeof(whitening), 10000U);

    const uint8_t sync_buf[9] = {
        0x55U, 0x55U, 0x55U, 0x55U, 0x55U, 0x55U,
        sync_word1, sync_word2,
        (uint8_t)(LR2021_GFSK_SYNCWORD_BIT_ORDER_MSB | 16U)
    };
    (void)lr2021_write_cmd(LR2021_RADIO_SET_FSK_SYNC_WORD_OC, sync_buf, sizeof(sync_buf), 10000U);

    lr2021_clear_errors();
    lr2021_clear_irq();
    (void)lr2021_write_cmd(LR2021_SYSTEM_CLEAR_RX_FIFO_OC, NULL, 0U, 10000U);
    (void)lr2021_write_cmd(LR2021_SYSTEM_CLEAR_TX_FIFO_OC, NULL, 0U, 10000U);
}

void lr2021_set_frequency(uint32_t freq_hz, bool enter_rx) {
    const uint8_t buf[4] = {
        (uint8_t)(freq_hz >> 24U),
        (uint8_t)(freq_hz >> 16U),
        (uint8_t)(freq_hz >> 8U),
        (uint8_t)(freq_hz & 0xFFU)
    };
    (void)lr2021_write_cmd(LR2021_RADIO_SET_RF_FREQUENCY_OC, buf, sizeof(buf), 5000U);
    if (enter_rx) {
        lr2021_set_mode(LR2021_MODE_RX_CONT);
    }
}

void lr2021_set_tx_power_half_dbm(int8_t power_half_dbm, bool is_sub_ghz) {
    if (is_sub_ghz) {
        if (power_half_dbm < LR2021_POWER_MIN_LF_PA) power_half_dbm = LR2021_POWER_MIN_LF_PA;
        if (power_half_dbm > 30) power_half_dbm = 30; /* Factory LF PA table max = 30 (+22 dBm) */
        g_pwr_lf_half_db = power_half_dbm;
    } else {
        if (power_half_dbm < LR2021_POWER_MIN_HF_PA) power_half_dbm = LR2021_POWER_MIN_HF_PA;
        if (power_half_dbm > 16) power_half_dbm = 16; /* Factory HF PA table max = 16 (+12 dBm) */
        g_pwr_hf_half_db = power_half_dbm;
    }
    int8_t active_pwr = g_is_sub_ghz ? g_pwr_lf_half_db : g_pwr_hf_half_db;
    const uint8_t tx_params[2] = {(uint8_t)active_pwr, LR2021_RADIO_RAMP_48_US};
    (void)lr2021_write_cmd(LR2021_RADIO_SET_TX_PARAMS_OC, tx_params, sizeof(tx_params), 5000U);
}

void lr2021_set_tx_power_dbm(int8_t power_dbm, bool is_sub_ghz) {
    if (is_sub_ghz) {
        int8_t half_db = (power_dbm >= 14) ? 29 : 26;
        if (power_dbm < 10) half_db = (int8_t)(power_dbm * 2);
        lr2021_set_tx_power_half_dbm(half_db, true);
    } else {
        int8_t half_db = (power_dbm >= 12) ? 16 : 13;
        if (power_dbm < 10) half_db = (int8_t)(power_dbm + 2);
        lr2021_set_tx_power_half_dbm(half_db, false);
    }
}

void lr2021_tx_nb(const uint8_t *data, uint8_t len) {
    uint8_t out_buf[255];
    uint8_t wire_len = len;
    if (g_mod_type == LR2021_MOD_FSK && g_fsk_use_fec && len == 8U) {
        fec_encode_8_to_14(data, out_buf);
        wire_len = 14U;
    } else {
        memcpy(out_buf, data, wire_len);
    }

    lr2021_set_mode(LR2021_MODE_FS);
    if (g_mod_type == LR2021_MOD_LORA && g_lora_explicit) {
        uint8_t iq_bit = (!g_is_sub_ghz && g_lora_invert_iq) ? LR2021_RADIO_LORA_IQ_INVERTED : LR2021_RADIO_LORA_IQ_STANDARD;
        uint8_t crc_bit = g_lora_crc_en ? 1U : 0U;
        const uint8_t pkt_params[4] = {
            (uint8_t)(g_lora_preamble >> 8U),
            (uint8_t)(g_lora_preamble & 0xFFU),
            wire_len,
            (uint8_t)((LR2021_LORA_PACKET_EXPLICIT << 2U) | (crc_bit << 1U) | iq_bit)
        };
        (void)lr2021_write_cmd(LR2021_RADIO_SET_LORA_PACKET_PARAMS_OC, pkt_params, sizeof(pkt_params), 10000U);
    }
    (void)lr2021_write_cmd(LR2021_RADIO_WRITE_TX_FIFO, out_buf, wire_len, 10000U);
    lr2021_set_mode(LR2021_MODE_TX);
}

void lr2021_rx_nb(void) {
    if (g_mod_type == LR2021_MOD_LORA && g_lora_explicit) {
        uint8_t iq_bit = (!g_is_sub_ghz && g_lora_invert_iq) ? LR2021_RADIO_LORA_IQ_INVERTED : LR2021_RADIO_LORA_IQ_STANDARD;
        uint8_t crc_bit = g_lora_crc_en ? 1U : 0U;
        const uint8_t pkt_params[4] = {
            (uint8_t)(g_lora_preamble >> 8U),
            (uint8_t)(g_lora_preamble & 0xFFU),
            g_payload_len,
            (uint8_t)((LR2021_LORA_PACKET_EXPLICIT << 2U) | (crc_bit << 1U) | iq_bit)
        };
        (void)lr2021_write_cmd(LR2021_RADIO_SET_LORA_PACKET_PARAMS_OC, pkt_params, sizeof(pkt_params), 10000U);
    }
    lr2021_set_mode(LR2021_MODE_RX_CONT);
}

int8_t lr2021_get_rssi_inst_dbm(void) {
    uint8_t status[4] = {0};
    (void)lr2021_write_cmd(LR2021_RADIO_GET_RSSI_INST_OC, NULL, 0U, 5000U);
    (void)lr2021_read_response(status, sizeof(status), 5000U);
    return -(int8_t)status[2];
}

void lr2021_set_callbacks(lr2021_rx_done_cb_t rx_cb, lr2021_tx_done_cb_t tx_cb) {
    g_rx_done_cb = rx_cb;
    g_tx_done_cb = tx_cb;
}

uint32_t lr2021_get_and_clear_irq(void) {
    uint8_t status[6] = {
        (uint8_t)(LR2021_SYSTEM_CLEAR_IRQ_OC >> 8U),
        (uint8_t)(LR2021_SYSTEM_CLEAR_IRQ_OC & 0xFFU),
        0xFFU, 0xFFU, 0xFFU, 0xFFU
    };
    (void)lr2021_read_response(status, sizeof(status), 5000U);
    return ((uint32_t)status[2] << 24U) |
           ((uint32_t)status[3] << 16U) |
           ((uint32_t)status[4] << 8U)  |
           ((uint32_t)status[5]);
}

void lr2021_clear_irq(void) {
    const uint8_t mask[4] = {0xFFU, 0xFFU, 0xFFU, 0xFFU};
    (void)lr2021_write_cmd(LR2021_SYSTEM_CLEAR_IRQ_OC, mask, sizeof(mask), 5000U);
}

void lr2021_decode_last_pkt_stats(lr2021_pkt_stats_t *out_stats) {
    uint8_t buf[8] = {0};
    if (g_mod_type == LR2021_MOD_FSK) {
        (void)lr2021_write_cmd(LR2021_RADIO_GET_FSK_PACKET_STATUS_OC, NULL, 0U, 5000U);
        (void)lr2021_read_response(buf, 8U, 5000U);
        out_stats->rssi_dbm  = -(int8_t)buf[4];
        out_stats->snr_q2_db = 0;
    } else if (g_mod_type == LR2021_MOD_FLRC) {
        (void)lr2021_write_cmd(LR2021_RADIO_GET_FLRC_PACKET_STATUS_OC, NULL, 0U, 5000U);
        (void)lr2021_read_response(buf, 7U, 5000U);
        out_stats->rssi_dbm  = -(int8_t)buf[4];
        out_stats->snr_q2_db = 0;
    } else {
        (void)lr2021_write_cmd(LR2021_RADIO_GET_LORA_PACKET_STATUS_OC, NULL, 0U, 5000U);
        (void)lr2021_read_response(buf, 8U, 5000U);
        out_stats->rssi_dbm  = -(int8_t)buf[6];
        out_stats->snr_q2_db = (int8_t)buf[4];
    }
    memcpy(out_stats->status_raw, buf, sizeof(buf));
}

void lr2021_read_rx_fifo(uint8_t *out_buf, uint8_t len) {
    uint8_t fifo_buf[66];
    if (len > 64U) {
        len = 64U;
    }
    memset(fifo_buf, 0, len + 2U);
    fifo_buf[0] = (uint8_t)(LR2021_RADIO_READ_RX_FIFO >> 8U);
    fifo_buf[1] = (uint8_t)(LR2021_RADIO_READ_RX_FIFO & 0xFFU);
    (void)lr2021_read_response(fifo_buf, (uint8_t)(len + 2U), 5000U);
    memcpy(out_buf, &fifo_buf[2], len);
}

uint32_t lr2021_get_irq_count(void) {
    return g_irq_count;
}

uint32_t lr2021_get_rx_done_count(void) {
    return g_rx_done_count;
}

uint32_t lr2021_get_tx_done_count(void) {
    return g_tx_done_count;
}

void lr2021_reset_counters(void) {
    g_irq_count     = 0U;
    g_rx_done_count = 0U;
    g_tx_done_count = 0U;
}
