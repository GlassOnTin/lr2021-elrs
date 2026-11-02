/* SPDX-License-Identifier: AGPL-3.0-or-later */
#include "lr1121_driver.h"
#include <string.h>

const uint8_t k_xr4_rfsw_ctrl[8] = {15, 0, 12, 8, 8, 6, 0, 5};

typedef struct {
    uint8_t  pkt_type;
    uint8_t  payload_len;
    bool     use_fec_14b;
    bool     is_sub_ghz;
    uint32_t curr_freq_hz;
} lr1121_radio_ctx_t;

static lr1121_radio_ctx_t g_radio_ctx[LR1121_RADIO_MAX];
static lr1121_hal_ops_t   g_hal_ops = {NULL, NULL, NULL, NULL};

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

void lr1121_fec_encode_8_to_14(const uint8_t in8[8], uint8_t out14[14]) {
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

void lr1121_fec_decode_14_to_8(const uint8_t in14[14], uint8_t out8[8]) {
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

void lr1121_set_hal_ops(const lr1121_hal_ops_t *ops) {
    if (ops != NULL) {
        g_hal_ops = *ops;
    }
}

static void write_cmd(lr1121_radio_idx_t radio, uint16_t opcode, const uint8_t *params, uint16_t len) {
    if (g_hal_ops.write_cmd != NULL) {
        g_hal_ops.write_cmd(radio, opcode, params, len);
    }
}

static void read_cmd(lr1121_radio_idx_t radio, uint16_t opcode, const uint8_t *args, uint16_t arg_len, uint8_t *out, uint16_t out_len) {
    if (g_hal_ops.read_cmd != NULL) {
        g_hal_ops.read_cmd(radio, opcode, args, arg_len, out, out_len);
    }
}

bool lr1121_get_version(lr1121_radio_idx_t radio, lr1121_version_t *out_ver) {
    uint8_t rsp[5] = {0};
    read_cmd(radio, LR1121_OC_GET_VERSION, NULL, 0U, rsp, sizeof(rsp));
    if (out_ver != NULL) {
        out_ver->hw_version = rsp[1];
        out_ver->chip_type  = rsp[2];
        out_ver->fw_version = (uint16_t)(((uint16_t)rsp[3] << 8U) | rsp[4]);
    }
    return (rsp[2] == 0x03U || rsp[2] == 0xF3U);
}

void lr1121_set_mode(lr1121_radio_idx_t radio, lr1121_op_mode_t mode) {
    uint8_t buf[5] = {0};
    switch (mode) {
    case LR1121_MODE_SLEEP:
        write_cmd(radio, LR1121_OC_SET_SLEEP, buf, 5U);
        break;
    case LR1121_MODE_STDBY_RC:
        buf[0] = 0x00U;
        write_cmd(radio, LR1121_OC_SET_STANDBY, buf, 1U);
        break;
    case LR1121_MODE_STDBY_XOSC:
        buf[0] = 0x01U;
        write_cmd(radio, LR1121_OC_SET_STANDBY, buf, 1U);
        break;
    case LR1121_MODE_FS:
        write_cmd(radio, LR1121_OC_SET_FS, NULL, 0U);
        break;
    case LR1121_MODE_RX_CONT:
        buf[0] = 0xFFU;
        buf[1] = 0xFFU;
        buf[2] = 0xFFU;
        write_cmd(radio, LR1121_OC_SET_RX, buf, 3U);
        break;
    case LR1121_MODE_TX:
        buf[0] = 0x00U;
        buf[1] = 0x00U;
        buf[2] = 0x00U;
        write_cmd(radio, LR1121_OC_SET_TX, buf, 3U);
        break;
    default:
        break;
    }
}

void lr1121_set_rf_switch(lr1121_radio_idx_t radio, const uint8_t rfsw_ctrl[8]) {
    const uint8_t *cfg = (rfsw_ctrl != NULL) ? rfsw_ctrl : k_xr4_rfsw_ctrl;
    write_cmd(radio, LR1121_OC_SET_DIO_AS_RF_SWITCH, cfg, 8U);
}

bool lr1121_begin(lr1121_radio_idx_t radio, uint32_t low_freq_hz, uint32_t high_freq_hz, lr1121_version_t *out_ver) {
    if (radio >= LR1121_RADIO_MAX) {
        return false;
    }
    memset(&g_radio_ctx[radio], 0, sizeof(g_radio_ctx[radio]));
    if (g_hal_ops.reset_radio != NULL) {
        g_hal_ops.reset_radio(radio);
    }

    lr1121_version_t ver = {0, 0, 0};
    if (!lr1121_get_version(radio, &ver)) {
        return false;
    }
    if (out_ver != NULL) {
        *out_ver = ver;
    }

    write_cmd(radio, LR1121_OC_CLEAR_ERRORS, NULL, 0U);
    lr1121_set_mode(radio, LR1121_MODE_STDBY_RC);

    /* Fallback to FS mode after TX/RX */
    const uint8_t fb_fs = 0x03U;
    write_cmd(radio, LR1121_OC_SET_RX_TX_FALLBACK_MODE, &fb_fs, 1U);

    /* Enable RX Boosted LNA */
    const uint8_t rx_boost = 0x01U;
    write_cmd(radio, LR1121_OC_SET_RX_BOOSTED, &rx_boost, 1U);

    /* Configure RadioMaster XR4 DIO5..DIO8 RF Switch Matrix */
    lr1121_set_rf_switch(radio, k_xr4_rfsw_ctrl);

    /* Route TX_DONE | RX_DONE to DIO1 */
    uint8_t irq_cfg[8] = {0};
    irq_cfg[3] = (uint8_t)(LR1121_IRQ_TX_DONE | LR1121_IRQ_RX_DONE);
    write_cmd(radio, LR1121_OC_SET_DIO_IRQ_PARAMS, irq_cfg, sizeof(irq_cfg));

    /* Enable DC-DC buck regulator (XR4 radio_dcdc = true) */
    const uint8_t dcdc_en = 0x01U;
    write_cmd(radio, LR1121_OC_SET_REGMODE, &dcdc_en, 1U);

    /* Calibrate image rejection across Sub-GHz band */
    uint8_t cal_img[2] = {
        (uint8_t)(((low_freq_hz / 1000000UL) - 1UL) / 4UL),
        (uint8_t)(1UL + (((high_freq_hz / 1000000UL) + 1UL) / 4UL))
    };
    write_cmd(radio, LR1121_OC_CALIBRATE_IMAGE, cal_img, sizeof(cal_img));

    return true;
}

void lr1121_set_frequency(lr1121_radio_idx_t radio, uint32_t freq_hz, bool enter_rx) {
    if (radio >= LR1121_RADIO_MAX) {
        return;
    }
    g_radio_ctx[radio].curr_freq_hz = freq_hz;
    g_radio_ctx[radio].is_sub_ghz   = (freq_hz < 1000000000UL);

    uint8_t buf[4] = {
        (uint8_t)(freq_hz >> 24U),
        (uint8_t)(freq_hz >> 16U),
        (uint8_t)(freq_hz >> 8U),
        (uint8_t)(freq_hz & 0xFFU)
    };
    write_cmd(radio, LR1121_OC_SET_RF_FREQUENCY, buf, sizeof(buf));
    if (enter_rx) {
        lr1121_set_mode(radio, LR1121_MODE_RX_CONT);
    }
}

void lr1121_set_pa_and_power_dbm(lr1121_radio_idx_t radio, int8_t power_dbm, bool is_sub_ghz) {
    uint8_t pa_cfg[4] = {0};
    int8_t clamped_pwr = power_dbm;

    if (is_sub_ghz) {
        if (clamped_pwr < -9)  clamped_pwr = -9;
        if (clamped_pwr > 22)  clamped_pwr = 22;
        pa_cfg[0] = LR1121_PA_SEL_HP; /* Sub-GHz High-Power PA */
        pa_cfg[1] = 0x01U;            /* VBAT supply */
        pa_cfg[2] = 0x04U;            /* PaDutyCycle */
        pa_cfg[3] = 0x07U;            /* PaHPSel */
    } else {
        if (clamped_pwr < -18) clamped_pwr = -18;
        if (clamped_pwr > 13)  clamped_pwr = 13;
        pa_cfg[0] = LR1121_PA_SEL_HF; /* 2.4 GHz High-Frequency PA */
        pa_cfg[1] = 0x00U;            /* VREG supply */
        pa_cfg[2] = 0x00U;
        pa_cfg[3] = 0x00U;
    }
    write_cmd(radio, LR1121_OC_SET_PA_CFG, pa_cfg, sizeof(pa_cfg));

    uint8_t tx_params[2] = {(uint8_t)clamped_pwr, LR1121_RAMP_48_US};
    write_cmd(radio, LR1121_OC_SET_TX_PARAMS, tx_params, sizeof(tx_params));
}

uint8_t lr1121_lora_bw_from_hz(uint32_t bw_hz) {
    if (bw_hz <= 62500UL)  return LR1121_LORA_BW_62;
    if (bw_hz <= 125000UL) return LR1121_LORA_BW_125;
    if (bw_hz <= 250000UL) return LR1121_LORA_BW_250;
    return LR1121_LORA_BW_500;
}

void lr1121_config_lora(lr1121_radio_idx_t radio,
                        uint8_t bw_code,
                        uint8_t sf,
                        uint8_t cr_code,
                        uint32_t freq_hz,
                        uint16_t preamble_len,
                        bool invert_iq,
                        uint8_t payload_len,
                        bool explicit_header,
                        bool crc_en) {
    if (radio >= LR1121_RADIO_MAX) {
        return;
    }
    bool is_sub_ghz = (freq_hz < 1000000000UL);
    g_radio_ctx[radio].pkt_type     = LR1121_PKT_TYPE_LORA;
    g_radio_ctx[radio].payload_len  = payload_len;
    g_radio_ctx[radio].use_fec_14b  = false;
    g_radio_ctx[radio].is_sub_ghz   = is_sub_ghz;
    g_radio_ctx[radio].curr_freq_hz = freq_hz;

    lr1121_set_mode(radio, LR1121_MODE_STDBY_RC);

    const uint8_t pkt_type = LR1121_PKT_TYPE_LORA;
    write_cmd(radio, LR1121_OC_SET_PKT_TYPE, &pkt_type, 1U);

    uint8_t mod_params[4] = {sf, bw_code, cr_code, 0x00U};
    write_cmd(radio, LR1121_OC_SET_MODULATION_PARAM, mod_params, sizeof(mod_params));

    /* Apply SX127x implicit SF6 compatibility register mask on Sub-GHz SF6 */
    if (is_sub_ghz && sf == 6U) {
        const uint8_t sf6_fix[12] = {
            0x00, 0xF2, 0x04, 0x14,
            0x00, 0x84, 0x00, 0x00,
            0x00, 0x04, 0x00, 0x00
        };
        write_cmd(radio, LR1121_OC_WRITE_REGMEM32_MASK, sf6_fix, sizeof(sf6_fix));
    }

    uint8_t pkt_params[6] = {
        (uint8_t)(preamble_len >> 8U),
        (uint8_t)(preamble_len & 0xFFU),
        (uint8_t)(explicit_header ? 0x00U : 0x01U),
        payload_len,
        (uint8_t)(crc_en ? 0x01U : 0x00U),
        (uint8_t)((!is_sub_ghz && invert_iq) ? 0x01U : 0x00U)
    };
    write_cmd(radio, LR1121_OC_SET_PKT_PARAM, pkt_params, sizeof(pkt_params));

    lr1121_set_frequency(radio, freq_hz, false);
    lr1121_set_pa_and_power_dbm(radio, is_sub_ghz ? 14 : 10, is_sub_ghz);
}

void lr1121_config_fsk_k1000(lr1121_radio_idx_t radio,
                             uint32_t freq_hz,
                             uint8_t sync_word1,
                             uint8_t sync_word2,
                             bool use_fec_14b) {
    if (radio >= LR1121_RADIO_MAX) {
        return;
    }
    bool is_sub_ghz = (freq_hz < 1000000000UL);
    uint8_t raw_len = use_fec_14b ? 14U : 8U;
    g_radio_ctx[radio].pkt_type     = LR1121_PKT_TYPE_GFSK;
    g_radio_ctx[radio].payload_len  = raw_len;
    g_radio_ctx[radio].use_fec_14b  = use_fec_14b;
    g_radio_ctx[radio].is_sub_ghz   = is_sub_ghz;
    g_radio_ctx[radio].curr_freq_hz = freq_hz;

    lr1121_set_mode(radio, LR1121_MODE_STDBY_RC);

    const uint8_t pkt_type = LR1121_PKT_TYPE_GFSK;
    write_cmd(radio, LR1121_OC_SET_PKT_TYPE, &pkt_type, 1U);

    /* 300 kbps GFSK, 600 kHz BW, 150 kHz Fdev */
    const uint32_t bitrate = 300000UL;
    const uint32_t fdev    = 150000UL;
    uint8_t mod_params[10] = {
        (uint8_t)(bitrate >> 24U),
        (uint8_t)(bitrate >> 16U),
        (uint8_t)(bitrate >> 8U),
        (uint8_t)(bitrate & 0xFFU),
        0x00U, /* Pulse shape OFF */
        LR1121_GFSK_BWF_600KHZ,
        (uint8_t)(fdev >> 24U),
        (uint8_t)(fdev >> 16U),
        (uint8_t)(fdev >> 8U),
        (uint8_t)(fdev & 0xFFU)
    };
    write_cmd(radio, LR1121_OC_SET_MODULATION_PARAM, mod_params, sizeof(mod_params));

    uint8_t pkt_params[9] = {
        0x00U,
        16U,   /* 16-bit preamble */
        0x04U, /* 8-bit preamble detector */
        16U,   /* 16-bit syncword */
        0x00U, /* Address filtering disabled */
        0x00U, /* Fixed length */
        raw_len,
        0x01U, /* Hardware CRC OFF (uses OTA CRC14 + Hamming FEC) */
        0x01U  /* SX127x/LR11xx whitening enabled */
    };
    write_cmd(radio, LR1121_OC_SET_PKT_PARAM, pkt_params, sizeof(pkt_params));

    uint8_t sync_buf[8] = {sync_word1, sync_word2, 0x55U, 0x55U, 0x55U, 0x55U, 0x55U, 0x55U};
    write_cmd(radio, LR1121_OC_SET_GFSK_SYNC_WORD, sync_buf, sizeof(sync_buf));

    lr1121_set_frequency(radio, freq_hz, false);
    lr1121_set_pa_and_power_dbm(radio, is_sub_ghz ? 14 : 10, is_sub_ghz);
}

void lr1121_tx_nb(lr1121_radio_idx_t radio, const uint8_t *data, uint8_t len) {
    if (radio >= LR1121_RADIO_MAX || data == NULL || len == 0U) {
        return;
    }
    uint8_t encoded[14];
    const uint8_t *tx_ptr = data;
    uint8_t tx_len = len;

    if (g_radio_ctx[radio].use_fec_14b && len == 8U) {
        lr1121_fec_encode_8_to_14(data, encoded);
        tx_ptr = encoded;
        tx_len = 14U;
    } else if (g_radio_ctx[radio].pkt_type == LR1121_PKT_TYPE_LORA &&
               g_radio_ctx[radio].payload_len != tx_len) {
        g_radio_ctx[radio].payload_len = tx_len;
        uint8_t pkt_params[6] = {
            0x00U, 8U, 0x00U, tx_len, 0x01U, 0x00U
        };
        write_cmd(radio, LR1121_OC_SET_PKT_PARAM, pkt_params, sizeof(pkt_params));
    }

    write_cmd(radio, LR1121_OC_WRITE_BUFFER8, tx_ptr, tx_len);
    lr1121_set_mode(radio, LR1121_MODE_TX);
}

bool lr1121_rx_read(lr1121_radio_idx_t radio, uint8_t *out_buf, uint8_t max_len, lr1121_pkt_stats_t *out_stats) {
    if (radio >= LR1121_RADIO_MAX || out_buf == NULL || max_len == 0U) {
        return false;
    }
    uint8_t rx_stat[3] = {0};
    read_cmd(radio, LR1121_OC_GET_RXBUFFER_STATUS, NULL, 0U, rx_stat, sizeof(rx_stat));
    uint8_t pkt_len = rx_stat[1];
    uint8_t offset  = rx_stat[2];

    if (pkt_len == 0U) {
        pkt_len = g_radio_ctx[radio].payload_len;
    }
    if (pkt_len == 0U) {
        return false;
    }

    uint8_t raw_buf[255] = {0};
    uint8_t read_args[2] = {offset, pkt_len};
    uint8_t rsp_buf[256] = {0};
    read_cmd(radio, LR1121_OC_READ_BUFFER8, read_args, sizeof(read_args), rsp_buf, (uint16_t)(pkt_len + 1U));
    memcpy(raw_buf, &rsp_buf[1], pkt_len);

    uint8_t pkt_st[4] = {0};
    read_cmd(radio, LR1121_OC_GET_PKT_STATUS, NULL, 0U, pkt_st, sizeof(pkt_st));
    if (out_stats != NULL) {
        out_stats->rssi_dbm  = -(int8_t)(pkt_st[1] / 2U);
        out_stats->snr_q2_db = (g_radio_ctx[radio].pkt_type == LR1121_PKT_TYPE_GFSK) ? 0 : (int8_t)pkt_st[2];
        out_stats->raw_len   = pkt_len;
    }

    if (g_radio_ctx[radio].use_fec_14b && pkt_len == 14U && max_len >= 8U) {
        lr1121_fec_decode_14_to_8(raw_buf, out_buf);
        if (out_stats != NULL) {
            out_stats->raw_len = 8U;
        }
        return true;
    }

    uint8_t copy_len = (pkt_len <= max_len) ? pkt_len : max_len;
    memcpy(out_buf, raw_buf, copy_len);
    return true;
}

uint32_t lr1121_get_and_clear_irq(lr1121_radio_idx_t radio) {
    const uint8_t clr_mask[4] = {0xFFU, 0xFFU, 0xFFU, 0xFFU};
    uint8_t st[4] = {0};
    read_cmd(radio, LR1121_OC_CLEAR_IRQ, clr_mask, sizeof(clr_mask), st, sizeof(st));
    return ((uint32_t)st[0] << 24U) |
           ((uint32_t)st[1] << 16U) |
           ((uint32_t)st[2] << 8U)  |
           ((uint32_t)st[3]);
}

/* ============================================================================
 * RadioMaster XR4 Simultaneous Dual-Radio Coordinator Implementation
 * ============================================================================ */
static void xr4_handle_completed_rns_payload(xr4_dual_radio_t *xr4,
                                             const uint8_t *pkt,
                                             uint8_t len,
                                             bool from_elrs_tunnel,
                                             uint32_t now_ms) {
    if (xr4 == NULL || pkt == NULL || len == 0U) {
        return;
    }

    /* 1. Multi-Drone Command Bundle (RNSM 0xA8, 64 bytes) */
    rns_multi_drone_pkt_t md;
    if (rns_parse_multi_drone_pkt(pkt, len, &md)) {
        rns_drone_cmd_t slot_cmd;
        if (rns_extract_multi_slot_cmd(&md, xr4->drone_slot_id, &slot_cmd)) {
            xr4->mav_seq++;
            xr4->last_mavlink_len = (uint8_t)rns_build_mavlink_command(
                &slot_cmd,
                xr4->mav_seq,
                xr4->last_mavlink_frame,
                sizeof(xr4->last_mavlink_frame));
        }

        if (from_elrs_tunnel) {
            /* Bridge immediately onto Radio 2 (868 MHz LoRa) while Radio 1 stays on 2.4 GHz ELRS! */
            uint8_t out_pkt[64];
            memcpy(out_pkt, pkt, 64U);
            rns_relay_record_sent(&xr4->relay, out_pkt, 64U, now_ms);
            lr1121_tx_nb(LR1121_RADIO_2, out_pkt, 64U);
            xr4->mesh_tx_cnt++;
            xr4->mesh_bridge_cnt++;
        } else {
            /* Received on Radio 2: schedule deterministic Slot-TDMA relay on Radio 2 */
            uint8_t fwd_pkt[64];
            memcpy(fwd_pkt, pkt, 64U);
            if (rns_relay_should_forward(&xr4->relay, fwd_pkt, 64U, now_ms)) {
                uint32_t slot_delay = rns_relay_compute_slot_delay_ms(
                    &xr4->relay, pkt, 64U, xr4->drone_slot_id, 0U);
                if (slot_delay == 0U) {
                    lr1121_tx_nb(LR1121_RADIO_2, fwd_pkt, 64U);
                    xr4->mesh_tx_cnt++;
                } else {
                    memcpy(xr4->pending_relay_pkt, fwd_pkt, 64U);
                    xr4->pending_relay_len    = 64U;
                    xr4->pending_relay_due_ms = now_ms + slot_delay;
                    xr4->pending_relay_valid  = true;
                }
            }
        }
        return;
    }

    /* 2. Swarm Formation Packet (RNSF 0xA7, 64 bytes) */
    rns_formation_pkt_t form;
    if (rns_parse_formation_pkt(pkt, len, &form)) {
        rns_drone_cmd_t slot_cmd;
        if (rns_compute_formation_slot_cmd(&form, xr4->drone_slot_id, &slot_cmd)) {
            xr4->mav_seq++;
            xr4->last_mavlink_len = (uint8_t)rns_build_mavlink_command(
                &slot_cmd,
                xr4->mav_seq,
                xr4->last_mavlink_frame,
                sizeof(xr4->last_mavlink_frame));
        }

        if (from_elrs_tunnel) {
            uint8_t out_pkt[64];
            memcpy(out_pkt, pkt, 64U);
            rns_relay_record_sent(&xr4->relay, out_pkt, 64U, now_ms);
            lr1121_tx_nb(LR1121_RADIO_2, out_pkt, 64U);
            xr4->mesh_tx_cnt++;
            xr4->mesh_bridge_cnt++;
        } else {
            uint8_t fwd_pkt[64];
            memcpy(fwd_pkt, pkt, 64U);
            if (rns_relay_should_forward(&xr4->relay, fwd_pkt, 64U, now_ms)) {
                uint32_t slot_delay = rns_relay_compute_slot_delay_ms(
                    &xr4->relay, pkt, 64U, xr4->drone_slot_id, 0U);
                if (slot_delay == 0U) {
                    lr1121_tx_nb(LR1121_RADIO_2, fwd_pkt, 64U);
                    xr4->mesh_tx_cnt++;
                } else {
                    memcpy(xr4->pending_relay_pkt, fwd_pkt, 64U);
                    xr4->pending_relay_len    = 64U;
                    xr4->pending_relay_due_ms = now_ms + slot_delay;
                    xr4->pending_relay_valid  = true;
                }
            }
        }
        return;
    }

    /* 3. Targeted Node Command (RNSC) */
    uint8_t sender_dest[16];
    uint8_t cmd_seq = 0U;
    char cmd_str[32];
    if (rns_parse_targeted_cmd_pkt(pkt, len, xr4->identity.dest_hash, sender_dest, &cmd_seq, cmd_str, sizeof(cmd_str))) {
        uint8_t plain_pkt[128];
        size_t plain_len = rns_build_drone_command_pkt(cmd_str, plain_pkt, sizeof(plain_pkt));
        rns_drone_cmd_t dcmd;
        if (plain_len > 0U && rns_parse_drone_command_pkt(plain_pkt, plain_len, &dcmd)) {
            xr4->mav_seq++;
            xr4->last_mavlink_len = (uint8_t)rns_build_mavlink_command(
                &dcmd,
                xr4->mav_seq,
                xr4->last_mavlink_frame,
                sizeof(xr4->last_mavlink_frame));
        }
        uint8_t ack_pkt[64];
        size_t ack_len = rns_build_targeted_cmd_pkt(
            sender_dest, xr4->identity.dest_hash, cmd_seq, "ACK:OK", ack_pkt, sizeof(ack_pkt));
        if (ack_len > 0U) {
            if (from_elrs_tunnel) {
                (void)elrs_tunnel_enqueue_tx(&xr4->elrs_tunnel, ack_pkt, (uint16_t)ack_len, false);
            } else {
                lr1121_tx_nb(LR1121_RADIO_2, ack_pkt, (uint8_t)ack_len);
                xr4->mesh_tx_cnt++;
            }
        }
        return;
    }

    /* 4. Broadcast Drone Command ("elrs.drone.cmd") */
    rns_drone_cmd_t bcmd;
    if (rns_parse_drone_command_pkt(pkt, len, &bcmd)) {
        xr4->mav_seq++;
        xr4->last_mavlink_len = (uint8_t)rns_build_mavlink_command(
            &bcmd,
            xr4->mav_seq,
            xr4->last_mavlink_frame,
            sizeof(xr4->last_mavlink_frame));
        if (from_elrs_tunnel) {
            rns_relay_record_sent(&xr4->relay, pkt, len, now_ms);
            lr1121_tx_nb(LR1121_RADIO_2, pkt, len);
            xr4->mesh_tx_cnt++;
            xr4->mesh_bridge_cnt++;
        }
    }
}

bool xr4_dual_radio_init(xr4_dual_radio_t *xr4,
                         const char *bind_phrase,
                         elrs_domain_t elrs_domain,
                         uint8_t elrs_rate_idx,
                         uint32_t mesh_freq_hz,
                         uint32_t mesh_bw_hz,
                         uint8_t mesh_sf,
                         uint8_t mesh_cr,
                         int8_t mesh_power_dbm,
                         uint8_t drone_slot_id,
                         const uint8_t hw_id[8]) {
    if (xr4 == NULL) {
        return false;
    }
    memset(xr4, 0, sizeof(*xr4));
    xr4->elrs_domain    = elrs_domain;
    xr4->elrs_rate_idx  = elrs_rate_idx;
    xr4->mesh_freq_hz   = mesh_freq_hz;
    xr4->mesh_bw_hz     = mesh_bw_hz;
    xr4->mesh_sf        = mesh_sf;
    xr4->mesh_cr        = mesh_cr;
    xr4->mesh_power_dbm = mesh_power_dbm;
    xr4->drone_slot_id  = (drone_slot_id >= 1U && drone_slot_id <= 4U) ? drone_slot_id : 1U;
    for (uint8_t i = 0; i < ELRS_NUM_CHANNELS; i++) {
        xr4->rc_channels[i] = ELRS_CRSF_CH_MID;
    }

    elrs_uid_from_bind_phrase((bind_phrase != NULL) ? bind_phrase : "WinniePico", xr4->uid);
    elrs_ota_init(&xr4->ota, xr4->uid, ELRS_OTA4_PACKET_SIZE, ELRS_SW_HYBRID_OR_16CH);
    elrs_fhss_init(&xr4->fhss, elrs_domain, xr4->ota.uid_seed);
    elrs_tunnel_init(&xr4->elrs_tunnel, 1U);
    rns_relay_init(&xr4->relay);
    rns_identity_init(&xr4->identity, hw_id, xr4->uid, RNS_NODE_ROLE_RX, "RM-XR4-DUAL");
    rns_peer_table_init(&xr4->peers);

    lr1121_version_t v1 = {0, 0, 0}, v2 = {0, 0, 0};
    if (!lr1121_begin(LR1121_RADIO_1, 863000000UL, 870000000UL, &v1)) {
        return false;
    }
    if (!lr1121_begin(LR1121_RADIO_2, 863000000UL, 870000000UL, &v2)) {
        return false;
    }

    /* Configure Radio 1 for 2.4 GHz 2G4_K1000 ELRS */
    uint32_t elrs_f0 = elrs_fhss_get_initial_freq(&xr4->fhss);
    lr1121_config_fsk_k1000(LR1121_RADIO_1, elrs_f0, xr4->uid[4], xr4->uid[5], true);
    lr1121_set_mode(LR1121_RADIO_1, LR1121_MODE_RX_CONT);

    /* Configure Radio 2 for 868 MHz Reticulum LoRa Mesh simultaneously */
    lr1121_config_lora(LR1121_RADIO_2,
                       lr1121_lora_bw_from_hz(mesh_bw_hz),
                       mesh_sf,
                       mesh_cr,
                       mesh_freq_hz,
                       8U,
                       false,
                       64U,
                       true,
                       true);
    lr1121_set_pa_and_power_dbm(LR1121_RADIO_2, mesh_power_dbm, true);
    lr1121_set_mode(LR1121_RADIO_2, LR1121_MODE_RX_CONT);

    return true;
}

void xr4_dual_radio_elrs_tock(xr4_dual_radio_t *xr4) {
    if (xr4 == NULL) {
        return;
    }
    xr4->elrs_fhss_ticks++;
    xr4->ota.ota_nonce++;
    uint32_t next_freq = elrs_fhss_get_curr_freq(&xr4->fhss);
    bool did_hop = false;
    if ((xr4->ota.ota_nonce % 2U) == 0U) {
        next_freq = elrs_fhss_advance_and_get_freq(&xr4->fhss);
        did_hop = true;
    }

    /* Odd nonces: transmit downlink ELRS DATA tunnel chunk or LINKSTATS on Radio 1 */
    if (xr4->elrs_locked && (xr4->ota.ota_nonce & 1U) == 1U) {
        uint8_t tx8[8] = {0};
        if (!elrs_tunnel_next_tx_frame(&xr4->elrs_tunnel, &xr4->ota, tx8)) {
            elrs_link_stats_data_t ls = {40, 42, 0, 1, 100, 1, 40};
            elrs_ota_pack_linkstats(&xr4->ota, &ls, (uint8_t)(xr4->elrs_tlm_tx_cnt & 1U), true, tx8);
        }
        lr1121_tx_nb(LR1121_RADIO_1, tx8, 8U);
        xr4->elrs_tlm_tx_cnt++;
    } else if (did_hop) {
        lr1121_set_frequency(LR1121_RADIO_1, next_freq, true);
    }
}

bool xr4_dual_radio_on_elrs_rx(xr4_dual_radio_t *xr4,
                               const uint8_t pkt8[8],
                               int8_t rssi_dbm,
                               int8_t snr_q2,
                               uint32_t now_ms) {
    (void)rssi_dbm;
    (void)snr_q2;
    if (xr4 == NULL || pkt8 == NULL) {
        return false;
    }
    uint8_t type = pkt8[0] & 0x03U;

    if (type == ELRS_PKT_TYPE_SYNC) {
        elrs_sync_data_t sync_out;
        if (elrs_ota_unpack_sync(&xr4->ota, pkt8, &sync_out)) {
            xr4->ota.ota_nonce = sync_out.nonce;
            elrs_fhss_set_index(&xr4->fhss, sync_out.fhss_index);
            xr4->elrs_locked = true;
            xr4->elrs_rx_ok_cnt++;
            return true;
        }
        xr4->elrs_crc_err_cnt++;
        return false;
    }

    if (!elrs_ota_validate_crc(&xr4->ota, pkt8, xr4->ota.ota_nonce)) {
        xr4->elrs_crc_err_cnt++;
        return false;
    }

    if (type == ELRS_PKT_TYPE_RCDATA) {
        bool stubborn_ack = false;
        if (elrs_ota_unpack_rc(&xr4->ota, pkt8, xr4->rc_channels, &xr4->is_armed, &stubborn_ack)) {
            xr4->elrs_rx_ok_cnt++;
            return true;
        }
        xr4->elrs_crc_err_cnt++;
        return false;
    }

    if (type == ELRS_PKT_TYPE_DATA) {
        xr4->elrs_rx_ok_cnt++;
        uint8_t rns_pkt[ELRS_TUNNEL_MAX_MTU];
        uint16_t rns_len = 0U;
        if (elrs_tunnel_process_rx_frame(&xr4->elrs_tunnel, pkt8, false, rns_pkt, &rns_len) && rns_len > 0U && rns_len <= 255U) {
            xr4_handle_completed_rns_payload(xr4, rns_pkt, (uint8_t)rns_len, true, now_ms);
        }
        return true;
    }

    return false;
}

bool xr4_dual_radio_on_mesh_rx(xr4_dual_radio_t *xr4,
                               const uint8_t *lora_pkt,
                               uint8_t len,
                               int8_t rssi_dbm,
                               int8_t snr_q2,
                               uint32_t now_ms) {
    (void)rssi_dbm;
    (void)snr_q2;
    if (xr4 == NULL || lora_pkt == NULL || len == 0U) {
        return false;
    }
    xr4->mesh_rx_cnt++;
    xr4_handle_completed_rns_payload(xr4, lora_pkt, len, false, now_ms);
    return true;
}

bool xr4_dual_radio_service_tdma(xr4_dual_radio_t *xr4, uint32_t now_ms) {
    if (xr4 == NULL || !xr4->pending_relay_valid) {
        return false;
    }
    if ((int32_t)(now_ms - xr4->pending_relay_due_ms) >= 0) {
        xr4->pending_relay_valid = false;
        lr1121_tx_nb(LR1121_RADIO_2, xr4->pending_relay_pkt, xr4->pending_relay_len);
        xr4->mesh_tx_cnt++;
        return true;
    }
    return false;
}
