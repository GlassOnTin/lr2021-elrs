/* SPDX-License-Identifier: AGPL-3.0-or-later */
/*
 * LilyGo T-Display P4 Standalone Drone TRX Unit Core Implementation.
 */
#include "p4_trx_core.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---------------------------------------------------------------------------
 * 1. Built-in 5x7 ASCII Font for Crisp RGB565 Landscape HUD Rendering
 * --------------------------------------------------------------------------- */
static const uint8_t k_font5x7[96][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, /* 0x20 ' ' */
    {0x00, 0x00, 0x5F, 0x00, 0x00}, /* 0x21 '!' */
    {0x00, 0x07, 0x00, 0x07, 0x00}, /* 0x22 '"' */
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, /* 0x23 '#' */
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, /* 0x24 '$' */
    {0x23, 0x13, 0x08, 0x64, 0x62}, /* 0x25 '%' */
    {0x36, 0x49, 0x55, 0x22, 0x50}, /* 0x26 '&' */
    {0x00, 0x05, 0x03, 0x00, 0x00}, /* 0x27 ''' */
    {0x00, 0x1C, 0x22, 0x41, 0x00}, /* 0x28 '(' */
    {0x00, 0x41, 0x22, 0x1C, 0x00}, /* 0x29 ')' */
    {0x14, 0x08, 0x3E, 0x08, 0x14}, /* 0x2A '*' */
    {0x08, 0x08, 0x3E, 0x08, 0x08}, /* 0x2B '+' */
    {0x00, 0x50, 0x30, 0x00, 0x00}, /* 0x2C ',' */
    {0x08, 0x08, 0x08, 0x08, 0x08}, /* 0x2D '-' */
    {0x00, 0x60, 0x60, 0x00, 0x00}, /* 0x2E '.' */
    {0x20, 0x10, 0x08, 0x04, 0x02}, /* 0x2F '/' */
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, /* 0x30 '0' */
    {0x00, 0x42, 0x7F, 0x40, 0x00}, /* 0x31 '1' */
    {0x42, 0x61, 0x51, 0x49, 0x46}, /* 0x32 '2' */
    {0x21, 0x41, 0x45, 0x4B, 0x31}, /* 0x33 '3' */
    {0x18, 0x14, 0x12, 0x7F, 0x10}, /* 0x34 '4' */
    {0x27, 0x45, 0x45, 0x45, 0x39}, /* 0x35 '5' */
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, /* 0x36 '6' */
    {0x01, 0x71, 0x09, 0x05, 0x03}, /* 0x37 '7' */
    {0x36, 0x49, 0x49, 0x49, 0x36}, /* 0x38 '8' */
    {0x06, 0x49, 0x49, 0x29, 0x1E}, /* 0x39 '9' */
    {0x00, 0x36, 0x36, 0x00, 0x00}, /* 0x3A ':' */
    {0x00, 0x56, 0x36, 0x00, 0x00}, /* 0x3B ';' */
    {0x08, 0x14, 0x22, 0x41, 0x00}, /* 0x3C '<' */
    {0x14, 0x14, 0x14, 0x14, 0x14}, /* 0x3D '=' */
    {0x00, 0x41, 0x22, 0x14, 0x08}, /* 0x3E '>' */
    {0x02, 0x01, 0x51, 0x09, 0x06}, /* 0x3F '?' */
    {0x32, 0x49, 0x79, 0x41, 0x3E}, /* 0x40 '@' */
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, /* 0x41 'A' */
    {0x7F, 0x49, 0x49, 0x49, 0x36}, /* 0x42 'B' */
    {0x3E, 0x41, 0x41, 0x41, 0x22}, /* 0x43 'C' */
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, /* 0x44 'D' */
    {0x7F, 0x49, 0x49, 0x49, 0x41}, /* 0x45 'E' */
    {0x7F, 0x09, 0x09, 0x09, 0x01}, /* 0x46 'F' */
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, /* 0x47 'G' */
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, /* 0x48 'H' */
    {0x00, 0x41, 0x7F, 0x41, 0x00}, /* 0x49 'I' */
    {0x20, 0x40, 0x41, 0x3F, 0x01}, /* 0x4A 'J' */
    {0x7F, 0x08, 0x14, 0x22, 0x41}, /* 0x4B 'K' */
    {0x7F, 0x40, 0x40, 0x40, 0x40}, /* 0x4C 'L' */
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, /* 0x4D 'M' */
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, /* 0x4E 'N' */
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, /* 0x4F 'O' */
    {0x7F, 0x09, 0x09, 0x09, 0x06}, /* 0x50 'P' */
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, /* 0x51 'Q' */
    {0x7F, 0x09, 0x19, 0x29, 0x46}, /* 0x52 'R' */
    {0x46, 0x49, 0x49, 0x49, 0x31}, /* 0x53 'S' */
    {0x01, 0x01, 0x7F, 0x01, 0x01}, /* 0x54 'T' */
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, /* 0x55 'U' */
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, /* 0x56 'V' */
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, /* 0x57 'W' */
    {0x63, 0x14, 0x08, 0x14, 0x63}, /* 0x58 'X' */
    {0x07, 0x08, 0x70, 0x08, 0x07}, /* 0x59 'Y' */
    {0x61, 0x51, 0x49, 0x45, 0x43}, /* 0x5A 'Z' */
    {0x00, 0x7F, 0x41, 0x41, 0x00}, /* 0x5B '[' */
    {0x02, 0x04, 0x08, 0x10, 0x20}, /* 0x5C '\' */
    {0x00, 0x41, 0x41, 0x7F, 0x00}, /* 0x5D ']' */
    {0x04, 0x02, 0x01, 0x02, 0x04}, /* 0x5E '^' */
    {0x40, 0x40, 0x40, 0x40, 0x40}, /* 0x5F '_' */
    {0x00, 0x01, 0x02, 0x04, 0x00}, /* 0x60 '`' */
    {0x20, 0x54, 0x54, 0x54, 0x78}, /* 0x61 'a' */
    {0x7F, 0x48, 0x44, 0x44, 0x38}, /* 0x62 'b' */
    {0x38, 0x44, 0x44, 0x44, 0x20}, /* 0x63 'c' */
    {0x38, 0x44, 0x44, 0x48, 0x7F}, /* 0x64 'd' */
    {0x38, 0x54, 0x54, 0x54, 0x18}, /* 0x65 'e' */
    {0x08, 0x7E, 0x09, 0x01, 0x02}, /* 0x66 'f' */
    {0x0C, 0x52, 0x52, 0x52, 0x3E}, /* 0x67 'g' */
    {0x7F, 0x08, 0x04, 0x04, 0x78}, /* 0x68 'h' */
    {0x00, 0x44, 0x7D, 0x40, 0x00}, /* 0x69 'i' */
    {0x20, 0x40, 0x44, 0x3D, 0x00}, /* 0x6A 'j' */
    {0x7F, 0x10, 0x28, 0x44, 0x00}, /* 0x6B 'k' */
    {0x00, 0x41, 0x7F, 0x40, 0x00}, /* 0x6C 'l' */
    {0x7C, 0x04, 0x18, 0x04, 0x78}, /* 0x6D 'm' */
    {0x7C, 0x08, 0x04, 0x04, 0x78}, /* 0x6E 'n' */
    {0x38, 0x44, 0x44, 0x44, 0x38}, /* 0x6F 'o' */
    {0x7C, 0x14, 0x14, 0x14, 0x08}, /* 0x70 'p' */
    {0x08, 0x14, 0x14, 0x18, 0x7C}, /* 0x71 'q' */
    {0x7C, 0x08, 0x04, 0x04, 0x08}, /* 0x72 'r' */
    {0x48, 0x54, 0x54, 0x54, 0x20}, /* 0x73 's' */
    {0x04, 0x3F, 0x44, 0x40, 0x20}, /* 0x74 't' */
    {0x3C, 0x40, 0x40, 0x20, 0x7C}, /* 0x75 'u' */
    {0x1C, 0x20, 0x40, 0x20, 0x1C}, /* 0x76 'v' */
    {0x3C, 0x40, 0x30, 0x40, 0x3C}, /* 0x77 'w' */
    {0x44, 0x28, 0x10, 0x28, 0x44}, /* 0x78 'x' */
    {0x0C, 0x50, 0x50, 0x50, 0x3C}, /* 0x79 'y' */
    {0x44, 0x64, 0x54, 0x4C, 0x44}, /* 0x7A 'z' */
    {0x00, 0x08, 0x36, 0x41, 0x00}, /* 0x7B '{' */
    {0x00, 0x00, 0x7F, 0x00, 0x00}, /* 0x7C '|' */
    {0x00, 0x41, 0x36, 0x08, 0x00}, /* 0x7D '}' */
    {0x10, 0x08, 0x08, 0x10, 0x08}, /* 0x7E '~' */
    {0x78, 0x46, 0x41, 0x46, 0x78}  /* 0x7F DEL */
};

static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
    return (uint16_t)(((uint16_t)(r & 0xF8U) << 8U) |
                      ((uint16_t)(g & 0xFCU) << 3U) |
                      ((uint16_t)(b >> 3U)));
}

#define COL_BG_DARK         rgb565(12, 16, 24)
#define COL_PANEL_BG        rgb565(22, 30, 44)
#define COL_PANEL_BORDER    rgb565(56, 76, 110)
#define COL_TOPBAR_BG       rgb565(18, 24, 36)
#define COL_ACCENT_CYAN     rgb565(0, 210, 255)
#define COL_ACCENT_GREEN    rgb565(40, 230, 100)
#define COL_ACCENT_AMBER    rgb565(255, 185, 20)
#define COL_ACCENT_RED      rgb565(245, 55, 65)
#define COL_ACCENT_PURPLE   rgb565(170, 95, 255)
#define COL_TEXT_WHITE      rgb565(240, 246, 255)
#define COL_TEXT_MUTED      rgb565(140, 158, 184)
#define COL_GIMBAL_WELL     rgb565(16, 22, 32)
#define COL_GIMBAL_GRID     rgb565(42, 56, 80)

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static int16_t clamp_i16(int32_t v, int16_t lo, int16_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return (int16_t)v;
}

static void emit_feedback(p4_trx_state_t *st, p4_feedback_event_t ev) {
    st->last_feedback = ev;
    st->feedback_count++;
}

static void apply_rate_rf_defaults(p4_trx_state_t *st) {
    uint8_t rate_cnt = elrs_get_air_rate_count();
    if (st->elrs_rate_idx >= rate_cnt) {
        st->elrs_rate_idx = 0U;
    }
    const elrs_air_rate_config_t *rf = elrs_get_air_rate_by_index(st->elrs_rate_idx);
    if (!rf) {
        st->elrs_rate_idx = 0U;
        rf = elrs_get_air_rate_by_index(0U);
    }
    if (rf->phy_type == ELRS_PHY_LORA_2G4 ||
        rf->phy_type == ELRS_PHY_FLRC_2G4 ||
        rf->phy_type == ELRS_PHY_FSK_2G4) {
        if (st->elrs_domain != ELRS_DOMAIN_ISM2G4 && st->elrs_domain != ELRS_DOMAIN_CE2G4) {
            st->elrs_domain = ELRS_DOMAIN_ISM2G4;
        }
        if (st->tx_power_dbm > P4_LR2021_MAX_HF_DBM) {
            st->tx_power_dbm = P4_LR2021_MAX_HF_DBM;
        }
    } else if (rf->phy_type == ELRS_PHY_LORA_900 || rf->phy_type == ELRS_PHY_FSK_900) {
        if (st->elrs_domain == ELRS_DOMAIN_ISM2G4 || st->elrs_domain == ELRS_DOMAIN_CE2G4) {
            st->elrs_domain = ELRS_DOMAIN_EU868;
        }
    }
    elrs_ota_init(&st->ota, st->uid, rf->payload_len, ELRS_SW_HYBRID_OR_16CH);
    elrs_fhss_init(&st->fhss, st->elrs_domain, st->ota.uid_seed);
}

/* ---------------------------------------------------------------------------
 * 2. Initialization & T-Display P4 Hardware Helpers
 * --------------------------------------------------------------------------- */
void p4_trx_init(p4_trx_state_t *st,
                 p4_panel_variant_t panel,
                 const char *bind_phrase,
                 const uint8_t hw_id[8]) {
    memset(st, 0, sizeof(*st));
    st->panel_variant  = panel;
    st->screen_w       = (panel == P4_PANEL_RM69A10_AMOLED) ? 1232U : 1168U;
    st->screen_h       = (panel == P4_PANEL_RM69A10_AMOLED) ? 568U  : 540U;
    st->brightness_pct = 85U;

    st->active_page            = P4_PAGE_HUD;
    st->haptics_enabled        = true;
    st->audio_enabled          = true;
    st->gimbal_mode            = P4_GIMBAL_MODE_2;
    st->throttle_spring_center = false;
    st->rate_pct               = 100U;
    st->expo_pct               = 20U;

    st->left_x         = 0.0f;
    st->left_y         = -1.0f; /* Idle throttle at bottom in Mode 2 */
    st->right_x        = 0.0f;
    st->right_y        = 0.0f;
    st->left_touch_id  = -1;
    st->right_touch_id = -1;

    st->app_mode       = 2U; /* ELRS_TX */
    st->elrs_rate_idx  = 13U; /* 2G4_K1000 (ELRS_RATE_FSK_2G4_1000HZ) */
    st->elrs_domain    = ELRS_DOMAIN_CE2G4;
    st->tx_power_dbm   = P4_LR2021_MAX_HF_DBM;
    st->antenna_path   = P4_ANTENNA_INTERNAL;

    const char *phrase = (bind_phrase && bind_phrase[0]) ? bind_phrase : "WinniePico";
    snprintf(st->bind_phrase, sizeof(st->bind_phrase), "%s", phrase);
    elrs_uid_from_bind_phrase(st->bind_phrase, st->uid);

    apply_rate_rf_defaults(st);
    elrs_tunnel_init(&st->tunnel, 2U);

    uint8_t default_hw[8] = {0xE4, 0x32, 0x50, 0x34, 0x20, 0x21, 0x00, 0x01};
    if (hw_id) {
        memcpy(default_hw, hw_id, 8U);
    }
    rns_identity_init(&st->rns_id, default_hw, st->uid, RNS_NODE_ROLE_TX, "P4-TRX-GCS");
    rns_peer_table_init(&st->rns_peers);
    rns_relay_init(&st->rns_relay);

    st->rns_freq_hz          = 868000000UL;
    st->rns_bw_hz            = 500000UL;
    st->rns_sf               = 7U;
    st->rns_cr               = 5U;
    st->rns_txpower_dbm      = 14;
    st->formation_spacing_dm = 100U; /* 10.0 m */
    snprintf(st->last_swarm_action, sizeof(st->last_swarm_action), "READY");

    /* Sensible initial battery / GNSS / spectrum defaults */
    st->battery.gauge_present = true;
    st->battery.voltage_mv    = 4120U;
    st->battery.current_ma    = -320;
    st->battery.soc_pct       = 92U;
    st->battery.temp_deci_c   = 285;

    st->gnss.lat_deg_e7       = 515073500L;  /* 51.5073500 N */
    st->gnss.lon_deg_e7       = -1277580L;   /* -0.1277580 W */
    st->gnss.alt_mm           = 45000L;

    st->spectrum_center_hz    = 868000000UL;
    st->spectrum_span_hz      = 4000000UL;
    for (uint8_t i = 0; i < P4_SPECTRUM_BINS; i++) {
        st->spectrum_dbm[i] = (int8_t)(-108 + ((i * 7U) % 15U));
    }

    p4_trx_update_crsf_channels(st);
}

uint16_t p4_trx_build_xl9535_output_word(const p4_trx_state_t *st,
                                         bool screen_rst_high,
                                         bool touch_rst_high,
                                         bool lora_rst_high) {
    uint16_t out = 0U;
    /* CRITICAL: IO0 (3.3V_POWER_EN) is active-LOW and MUST remain 0 at all times! */
    out &= (uint16_t)~P4_XL_BIT_3V3_POWER_EN;

    /* IO1: SKY13453_VCTL (1 = Internal Antenna, 0 = External MMCX1) */
    if (!st || st->antenna_path == P4_ANTENNA_INTERNAL) {
        out |= P4_XL_BIT_SKY13453_VCTL;
    }

    /* IO2 & IO3: Display & Touch active-LOW resets */
    if (screen_rst_high) out |= P4_XL_BIT_SCREEN_RST;
    if (touch_rst_high)  out |= P4_XL_BIT_TOUCH_RST;

    /* IO6: 5.0V & RT9080-33 Audio rail enable */
    out |= P4_XL_BIT_5V0_AUDIO_PWR_EN;

    /* IO10 (bit 8): USB PHY / VCCA rail enable — MUST remain 1 */
    out |= P4_XL_BIT_USB_PHY_PWR_EN;

    /* IO11 (bit 9): L76K GPS Wake-Up = 1 */
    out |= P4_XL_BIT_GPS_WAKE_UP;

    /* IO16 (bit 14): LR2021 NRESET */
    if (lora_rst_high) out |= P4_XL_BIT_LORA_RST;

    return out;
}

void p4_trx_compute_lr2021_rf_setup(uint32_t freq_hz,
                                    int8_t requested_dbm,
                                    p4_lr2021_rf_setup_t *out_setup) {
    if (!out_setup) return;
    memset(out_setup, 0, sizeof(*out_setup));

    /* T-Display P4 uses a 32.0 MHz TCXO at 3.0V (0x06) */
    out_setup->tcxo_tune    = P4_LR2021_TCXO_TUNE_3V0;
    out_setup->tcxo_timeout = P4_LR2021_TCXO_TIMEOUT_TICKS;

    /* T-Display P4 LR2021 DIO RF switch matrix for AT2401C (2.4 GHz) + Sub-GHz diplexer:
     *   DIO6  -> 0x04 (WHEN_RX_HF -> AT2401C RXEN)
     *   DIO7  -> 0x08 (WHEN_TX_HF -> AT2401C TXEN)
     *   DIO8  -> 0x03 (WHEN_RX_LF | WHEN_TX_LF)
     *   DIO10 -> 0x0C (WHEN_RX_HF | WHEN_TX_HF)
     *   DIO11 -> IRQ
     */
    out_setup->dio_rf_sw_map[0] = 0x04U; /* DIO6 */
    out_setup->dio_rf_sw_map[1] = 0x08U; /* DIO7 */
    out_setup->dio_rf_sw_map[2] = 0x03U; /* DIO8 */
    out_setup->dio_rf_sw_map[3] = 0x0CU; /* DIO10 */
    out_setup->dio_rf_sw_map[4] = 0x00U; /* DIO11 (IRQ) */

    if (freq_hz >= 1000000000UL) {
        /* 2.4 GHz HF path feeding external AT2401C PA: cap LR2021 HF PA drive at +5 dBm */
        int8_t dbm = requested_dbm;
        if (dbm > P4_LR2021_MAX_HF_DBM) dbm = P4_LR2021_MAX_HF_DBM;
        if (dbm < -10) dbm = -10;
        out_setup->pa_sel           = 1U; /* HF PA */
        out_setup->pa_lf_mode       = 1U; /* pa_supply = 1 (0x81) */
        out_setup->pa_lf_duty_cycle = 6U;
        out_setup->pa_lf_slices     = 7U;
        out_setup->pa_hf_duty_cycle = 0x1BU;
        out_setup->power_half_dbm   = (int8_t)(dbm * 2);
    } else {
        /* Sub-GHz LF path (868 / 915 MHz): up to +22 dBm */
        int8_t dbm = requested_dbm;
        if (dbm > P4_LR2021_MAX_LF_DBM) dbm = P4_LR2021_MAX_LF_DBM;
        if (dbm < -9) dbm = -9;
        out_setup->pa_sel           = 0U; /* LF PA */
        out_setup->pa_lf_mode       = 1U; /* pa_supply = 1 (0x01) */
        out_setup->pa_lf_duty_cycle = 7U;
        out_setup->pa_lf_slices     = 7U;
        out_setup->pa_hf_duty_cycle = 16U;
        out_setup->power_half_dbm   = (int8_t)(dbm * 2);
    }
}

void p4_trx_map_raw_touch_to_landscape(p4_panel_variant_t panel,
                                       uint16_t raw_x,
                                       uint16_t raw_y,
                                       uint16_t *out_lx,
                                       uint16_t *out_ly) {
    uint16_t port_w = (panel == P4_PANEL_RM69A10_AMOLED) ? 568U  : 540U;
    uint16_t port_h = (panel == P4_PANEL_RM69A10_AMOLED) ? 1232U : 1168U;

    uint16_t px = raw_x;
    uint16_t py = raw_y;
    if (panel == P4_PANEL_RM69A10_AMOLED && (raw_x > port_w || raw_y > port_h)) {
        /* GT9895 reports raw 0..1060 x 0..2400 when unscaled */
        px = (uint16_t)(((uint32_t)raw_x * port_w) / 1060UL);
        py = (uint16_t)(((uint32_t)raw_y * port_h) / 2400UL);
    }
    if (px >= port_w) px = port_w - 1U;
    if (py >= port_h) py = port_h - 1U;

    /* 90-degree clockwise landscape mapping:
     * Landscape X (0 .. port_h-1) = py
     * Landscape Y (0 .. port_w-1) = (port_w - 1) - px
     */
    if (out_lx) *out_lx = py;
    if (out_ly) *out_ly = (uint16_t)((port_w - 1U) - px);
}

/* ---------------------------------------------------------------------------
 * 3. Multi-Touch Gimbal, Switch & Console Interaction Engine
 * --------------------------------------------------------------------------- */
static void handle_center_or_bar_tap(p4_trx_state_t *st, uint16_t x, uint16_t y, uint32_t now_ms) {
    /* 1. Top Bar DFU button (x >= screen_w - 92, y < 44) */
    if (y < 44U) {
        if (x >= (st->screen_w - 92U)) {
            st->dfu_requested = true;
            emit_feedback(st, P4_FEEDBACK_ALERT);
        }
        return;
    }

    /* 2. Bottom 8-Switch Bar (y >= screen_h - 64) */
    if (y >= (st->screen_h - 64U)) {
        uint16_t sw_w = st->screen_w / 8U;
        uint8_t idx = (uint8_t)(x / sw_w);
        if (idx > 7U) idx = 7U;
        switch (idx) {
            case 0:
                st->sw_armed = !st->sw_armed;
                emit_feedback(st, st->sw_armed ? P4_FEEDBACK_ARM : P4_FEEDBACK_DISARM);
                break;
            case 1:
                st->sw_flight_mode = (uint8_t)((st->sw_flight_mode + 1U) % 3U);
                emit_feedback(st, P4_FEEDBACK_CLICK);
                break;
            case 2:
                st->sw_loiter = !st->sw_loiter;
                emit_feedback(st, P4_FEEDBACK_CLICK);
                break;
            case 3:
                st->sw_rtl = !st->sw_rtl;
                emit_feedback(st, P4_FEEDBACK_ALERT);
                break;
            case 4:
                st->sw_formation = (uint8_t)((st->sw_formation + 1U) % 4U);
                (void)p4_trx_build_follow_me_formation_pkt(st, st->pending_rns_pkt);
                st->pending_rns_len = 64U;
                emit_feedback(st, P4_FEEDBACK_SWARM_TX);
                break;
            case 5:
                st->sw_follow_me = !st->sw_follow_me;
                if (st->sw_follow_me) {
                    st->last_follow_me_ms = now_ms;
                    (void)p4_trx_build_follow_me_formation_pkt(st, st->pending_rns_pkt);
                    st->pending_rns_len = 64U;
                }
                emit_feedback(st, P4_FEEDBACK_SWARM_TX);
                break;
            case 6:
                st->sw_imu_tilt_steer = !st->sw_imu_tilt_steer;
                emit_feedback(st, P4_FEEDBACK_CLICK);
                break;
            case 7:
            default:
                st->active_page = (p4_ui_page_t)((st->active_page + 1U) % P4_PAGE_COUNT);
                emit_feedback(st, P4_FEEDBACK_CLICK);
                break;
        }
        return;
    }

    /* 3. Center Console Area (x in [284 .. screen_w - 284]) */
    uint16_t cx0 = 284U;
    uint16_t cx1 = (uint16_t)(st->screen_w - 284U);
    if (x < cx0 || x >= cx1) {
        return;
    }
    uint16_t cw = (uint16_t)(cx1 - cx0);

    /* Page Tabs Row: y in [48 .. 90] */
    if (y >= 48U && y < 90U) {
        uint8_t tab = (uint8_t)(((uint32_t)(x - cx0) * 4UL) / cw);
        if (tab < P4_PAGE_COUNT) {
            st->active_page = (p4_ui_page_t)tab;
            emit_feedback(st, P4_FEEDBACK_CLICK);
        }
        return;
    }

    /* Page-Specific Interactive Buttons */
    uint16_t rel_x = (uint16_t)(x - cx0);
    uint16_t half_w = cw / 2U;

    if (st->active_page == P4_PAGE_ELRS) {
        uint8_t rate_cnt = elrs_get_air_rate_count();
        if (y >= 110U && y < 165U) {
            /* Row 1: Rate - / Rate + */
            if (rel_x < half_w) {
                st->elrs_rate_idx = (st->elrs_rate_idx == 0U) ? (uint8_t)(rate_cnt - 1U) : (uint8_t)(st->elrs_rate_idx - 1U);
            } else {
                st->elrs_rate_idx = (uint8_t)((st->elrs_rate_idx + 1U) % rate_cnt);
            }
            apply_rate_rf_defaults(st);
            emit_feedback(st, P4_FEEDBACK_CLICK);
        } else if (y >= 175U && y < 230U) {
            /* Row 2: Domain / Antenna (INT vs MMCX1) */
            if (rel_x < half_w) {
                st->elrs_domain = (elrs_domain_t)((st->elrs_domain + 1U) % ELRS_DOMAIN_COUNT);
                apply_rate_rf_defaults(st);
            } else {
                st->antenna_path = (st->antenna_path == P4_ANTENNA_INTERNAL)
                                       ? P4_ANTENNA_MMCX1
                                       : P4_ANTENNA_INTERNAL;
            }
            emit_feedback(st, P4_FEEDBACK_CLICK);
        } else if (y >= 240U && y < 295U) {
            /* Row 3: TX Power - / TX Power + */
            int8_t max_p = (st->elrs_domain == ELRS_DOMAIN_ISM2G4 || st->elrs_domain == ELRS_DOMAIN_CE2G4)
                               ? P4_LR2021_MAX_HF_DBM
                               : P4_LR2021_MAX_LF_DBM;
            if (rel_x < half_w) {
                if (st->tx_power_dbm > -5) st->tx_power_dbm -= 2;
            } else {
                if (st->tx_power_dbm < max_p) st->tx_power_dbm += 2;
                if (st->tx_power_dbm > max_p) st->tx_power_dbm = max_p;
            }
            emit_feedback(st, P4_FEEDBACK_CLICK);
        } else if (y >= 305U && y < 360U) {
            /* Row 4: Gimbal Mode 1..4 / App Mode */
            if (rel_x < half_w) {
                st->gimbal_mode = (p4_gimbal_mode_t)((((uint8_t)st->gimbal_mode) % 4U) + 1U);
            } else {
                st->app_mode = (uint8_t)((st->app_mode % 4U) + 1U);
            }
            emit_feedback(st, P4_FEEDBACK_CLICK);
        }
    } else if (st->active_page == P4_PAGE_SWARM) {
        if (y >= 110U && y < 165U) {
            /* Row 1: [OTA PAIR (RNSP)] / [4-DRONE BUNDLE (RNSM)] */
            if (rel_x < half_w) {
                rns_pairing_cfg_t pcfg = {};
                memcpy(pcfg.tx_dest, st->rns_id.dest_hash, 16U);
                pcfg.freq_hz     = st->rns_freq_hz;
                pcfg.bw_hz       = st->rns_bw_hz;
                pcfg.sf          = st->rns_sf;
                pcfg.cr          = st->rns_cr;
                pcfg.txpower_dbm = st->rns_txpower_dbm;
                st->pending_rns_len = rns_build_pairing_pkt(&pcfg, st->pending_rns_pkt, sizeof(st->pending_rns_pkt));
                (void)elrs_tunnel_enqueue_tx(&st->tunnel, st->pending_rns_pkt, (uint16_t)st->pending_rns_len,
                                             st->ota.is_full_res);
                snprintf(st->last_swarm_action, sizeof(st->last_swarm_action), "TX RNSP PAIR (%uB)", (unsigned)st->pending_rns_len);
            } else {
                st->pending_rns_len = p4_trx_build_default_swarm_bundle_pkt(st, st->pending_rns_pkt);
                (void)elrs_tunnel_enqueue_tx(&st->tunnel, st->pending_rns_pkt, (uint16_t)st->pending_rns_len,
                                             st->ota.is_full_res);
                snprintf(st->last_swarm_action, sizeof(st->last_swarm_action), "TX RNSM 4-DRONE #%u", (unsigned)st->multi_seq);
            }
            emit_feedback(st, P4_FEEDBACK_SWARM_TX);
        } else if (y >= 175U && y < 230U) {
            /* Row 2: [FORM: VEE / LINE / COL] / [FOLLOW-ME GPS] */
            if (rel_x < half_w) {
                st->sw_formation = (uint8_t)((st->sw_formation + 1U) % 4U);
                st->pending_rns_len = p4_trx_build_follow_me_formation_pkt(st, st->pending_rns_pkt);
                (void)elrs_tunnel_enqueue_tx(&st->tunnel, st->pending_rns_pkt, (uint16_t)st->pending_rns_len,
                                             st->ota.is_full_res);
                snprintf(st->last_swarm_action, sizeof(st->last_swarm_action), "TX RNSF FORM=%u #%u",
                         (unsigned)st->sw_formation, (unsigned)st->formation_seq);
            } else {
                st->sw_follow_me = !st->sw_follow_me;
                st->pending_rns_len = p4_trx_build_follow_me_formation_pkt(st, st->pending_rns_pkt);
                (void)elrs_tunnel_enqueue_tx(&st->tunnel, st->pending_rns_pkt, (uint16_t)st->pending_rns_len,
                                             st->ota.is_full_res);
                snprintf(st->last_swarm_action, sizeof(st->last_swarm_action), "FOLLOW-ME %s #%u",
                         st->sw_follow_me ? "ON" : "OFF", (unsigned)st->formation_seq);
            }
            emit_feedback(st, P4_FEEDBACK_SWARM_TX);
        } else if (y >= 240U && y < 295U) {
            /* Row 3: Targeted RNSC [CMD: LOITER] / [CMD: RTL] */
            const char *cmd_str = (rel_x < half_w) ? "LOITER" : "RTL";
            uint8_t target_dest[16] = {0};
            if (st->rns_peers.selected_idx >= 0 && st->rns_peers.selected_idx < st->rns_peers.count) {
                memcpy(target_dest, st->rns_peers.peers[st->rns_peers.selected_idx].dest_hash, 16U);
            }
            st->targeted_seq++;
            st->pending_rns_len = rns_build_targeted_cmd_pkt(target_dest, st->rns_id.dest_hash,
                                                             st->targeted_seq, cmd_str,
                                                             st->pending_rns_pkt, sizeof(st->pending_rns_pkt));
            (void)elrs_tunnel_enqueue_tx(&st->tunnel, st->pending_rns_pkt, (uint16_t)st->pending_rns_len,
                                         st->ota.is_full_res);
            snprintf(st->last_swarm_action, sizeof(st->last_swarm_action), "TX RNSC %s #%u",
                     cmd_str, (unsigned)st->targeted_seq);
            emit_feedback(st, P4_FEEDBACK_SWARM_TX);
        }
    } else if (st->active_page == P4_PAGE_SPECTRUM) {
        if (y >= 330U && y < 395U) {
            if (rel_x < half_w) {
                st->haptics_enabled = !st->haptics_enabled;
            } else {
                st->audio_enabled = !st->audio_enabled;
            }
            emit_feedback(st, P4_FEEDBACK_CLICK);
        }
    }
}

void p4_trx_process_touch(p4_trx_state_t *st,
                          const p4_touch_frame_t *touch,
                          uint32_t now_ms) {
    if (!st || !touch) return;

    const float left_cx  = 142.0f;
    const float left_cy  = 256.0f;
    const float right_cx = (float)(st->screen_w - 142U);
    const float right_cy = 256.0f;
    const float radius   = 118.0f;

    bool left_seen  = false;
    bool right_seen = false;
    bool any_ui_touch = false;
    uint16_t ui_x = 0U, ui_y = 0U;

    for (uint8_t i = 0; i < touch->count && i < P4_MAX_TOUCH_POINTS; i++) {
        const p4_touch_point_t *pt = &touch->points[i];
        if (!pt->valid) continue;

        /* Check if this finger is already tracking Left or Right gimbal, or lands inside its well */
        bool is_left_zone  = (pt->x < 284U && pt->y >= 46U && pt->y < (st->screen_h - 64U));
        bool is_right_zone = (pt->x >= (st->screen_w - 284U) && pt->y >= 46U && pt->y < (st->screen_h - 64U));

        if (st->left_touch_id == (int8_t)pt->id || (st->left_touch_id < 0 && is_left_zone)) {
            st->left_touch_id = (int8_t)pt->id;
            left_seen = true;
            float nx = ((float)pt->x - left_cx) / radius;
            float ny = (left_cy - (float)pt->y) / radius; /* Up = +1.0, Down = -1.0 */
            float prev_x = st->left_x;
            st->left_x = clampf(nx, -1.0f, 1.0f);
            st->left_y = clampf(ny, -1.0f, 1.0f);
            if ((prev_x < 0.0f && st->left_x >= 0.0f) || (prev_x > 0.0f && st->left_x <= 0.0f)) {
                emit_feedback(st, P4_FEEDBACK_DETENT);
            }
        } else if (st->right_touch_id == (int8_t)pt->id || (st->right_touch_id < 0 && is_right_zone)) {
            st->right_touch_id = (int8_t)pt->id;
            right_seen = true;
            float nx = ((float)pt->x - right_cx) / radius;
            float ny = (right_cy - (float)pt->y) / radius;
            float prev_x = st->right_x;
            st->right_x = clampf(nx, -1.0f, 1.0f);
            st->right_y = clampf(ny, -1.0f, 1.0f);
            if ((prev_x < 0.0f && st->right_x >= 0.0f) || (prev_x > 0.0f && st->right_x <= 0.0f)) {
                emit_feedback(st, P4_FEEDBACK_DETENT);
            }
        } else {
            any_ui_touch = true;
            ui_x = pt->x;
            ui_y = pt->y;
        }
    }

    /* Spring-return released gimbals (Throttle stays at last Y position unless throttle_spring_center is set) */
    if (!left_seen) {
        st->left_touch_id = -1;
        st->left_x = 0.0f;
        bool left_y_is_thr = (st->gimbal_mode == P4_GIMBAL_MODE_2 || st->gimbal_mode == P4_GIMBAL_MODE_4);
        if (!left_y_is_thr || st->throttle_spring_center) {
            st->left_y = 0.0f;
        }
    }
    if (!right_seen) {
        st->right_touch_id = -1;
        st->right_x = 0.0f;
        bool right_y_is_thr = (st->gimbal_mode == P4_GIMBAL_MODE_1 || st->gimbal_mode == P4_GIMBAL_MODE_3);
        if (!right_y_is_thr || st->throttle_spring_center) {
            st->right_y = 0.0f;
        }
    }

    if (any_ui_touch && !st->ui_tap_latched) {
        st->ui_tap_latched = true;
        handle_center_or_bar_tap(st, ui_x, ui_y, now_ms);
    } else if (!any_ui_touch) {
        st->ui_tap_latched = false;
    }

    p4_trx_update_crsf_channels(st);
}

static float apply_expo_and_rate(float v, uint8_t rate_pct, uint8_t expo_pct) {
    float x = clampf(v, -1.0f, 1.0f);
    float k = clampf((float)expo_pct / 100.0f, 0.0f, 0.8f);
    float shaped = (1.0f - k) * x + k * x * x * x;
    float scale  = clampf((float)rate_pct / 100.0f, 0.2f, 1.0f);
    return clampf(shaped * scale, -1.0f, 1.0f);
}

static uint16_t norm_to_crsf(float v, int16_t trim) {
    float span = (v < 0.0f) ? 820.0f : 819.0f;
    int32_t raw = (int32_t)lroundf((float)ELRS_CRSF_CH_MID + v * span) + (int32_t)trim;
    return (uint16_t)clamp_i16(raw, (int16_t)ELRS_CRSF_CH_MIN, (int16_t)ELRS_CRSF_CH_MAX);
}

void p4_trx_update_crsf_channels(p4_trx_state_t *st) {
    if (!st) return;

    float roll_in = 0.0f, pitch_in = 0.0f, thr_in = -1.0f, yaw_in = 0.0f;
    switch (st->gimbal_mode) {
        case P4_GIMBAL_MODE_1: /* Left: Pitch/Yaw, Right: Throttle/Roll */
            pitch_in = st->left_y;
            yaw_in   = st->left_x;
            thr_in   = st->right_y;
            roll_in  = st->right_x;
            break;
        case P4_GIMBAL_MODE_3: /* Left: Pitch/Roll, Right: Throttle/Yaw */
            pitch_in = st->left_y;
            roll_in  = st->left_x;
            thr_in   = st->right_y;
            yaw_in   = st->right_x;
            break;
        case P4_GIMBAL_MODE_4: /* Left: Throttle/Roll, Right: Pitch/Yaw */
            thr_in   = st->left_y;
            roll_in  = st->left_x;
            pitch_in = st->right_y;
            yaw_in   = st->right_x;
            break;
        case P4_GIMBAL_MODE_2: /* Default Mode 2 — Left: Throttle/Yaw, Right: Pitch/Roll */
        default:
            thr_in   = st->left_y;
            yaw_in   = st->left_x;
            pitch_in = st->right_y;
            roll_in  = st->right_x;
            break;
    }

    /* Optional IMU Tilt-to-Steer override/blending (full deflection at +/- 30 deg) */
    if (st->sw_imu_tilt_steer && st->imu.imu_present) {
        roll_in  = clampf(roll_in  + (st->imu.roll_deg  / 30.0f), -1.0f, 1.0f);
        pitch_in = clampf(pitch_in + (st->imu.pitch_deg / 30.0f), -1.0f, 1.0f);
    }

    float roll_s  = apply_expo_and_rate(roll_in,  st->rate_pct, st->expo_pct);
    float pitch_s = apply_expo_and_rate(pitch_in, st->rate_pct, st->expo_pct);
    float yaw_s   = apply_expo_and_rate(yaw_in,   st->rate_pct, st->expo_pct);
    float thr_s   = clampf(thr_in, -1.0f, 1.0f);

    /* AETR1234 Standard ExpressLRS / Betaflight / ArduPilot Channel Order:
     * CH1 = Roll (Aileron), CH2 = Pitch (Elevator), CH3 = Throttle, CH4 = Yaw (Rudder)
     */
    st->crsf_channels[0] = norm_to_crsf(roll_s,  st->trim_roll);
    st->crsf_channels[1] = norm_to_crsf(pitch_s, st->trim_pitch);
    st->crsf_channels[2] = norm_to_crsf(thr_s,   st->trim_thr);
    st->crsf_channels[3] = norm_to_crsf(yaw_s,   st->trim_yaw);

    /* Aux Switches CH5..CH12 (191 = ~1000us, 992 = 1500us, 1792 = ~2000us) */
    const uint16_t k_sw_lo  = 191U;
    const uint16_t k_sw_mid = ELRS_CRSF_CH_MID;
    const uint16_t k_sw_hi  = 1792U;

    st->crsf_channels[4] = st->sw_armed ? k_sw_hi : k_sw_lo;
    st->crsf_channels[5] = (st->sw_flight_mode == 0U) ? k_sw_lo :
                           (st->sw_flight_mode == 1U) ? k_sw_mid : k_sw_hi;
    st->crsf_channels[6] = st->sw_loiter ? k_sw_hi : k_sw_lo;
    st->crsf_channels[7] = st->sw_rtl    ? k_sw_hi : k_sw_lo;
    st->crsf_channels[8] = (uint16_t)(k_sw_lo + (uint16_t)st->sw_formation * 533U);
    st->crsf_channels[9] = st->sw_follow_me      ? k_sw_hi : k_sw_lo;
    st->crsf_channels[10]= st->sw_imu_tilt_steer ? k_sw_hi : k_sw_lo;
    st->crsf_channels[11]= st->sw_beeper         ? k_sw_hi : k_sw_lo;

    for (uint8_t c = 12U; c < CRSF_NUM_CHANNELS; c++) {
        st->crsf_channels[c] = k_sw_mid;
    }
}

size_t p4_trx_step_elrs_tx_frame(p4_trx_state_t *st,
                                 uint32_t *out_rf_freq_hz,
                                 uint8_t out_ota_pkt[16]) {
    if (!st || !out_ota_pkt) return 0U;

    const elrs_air_rate_config_t *rf = elrs_get_air_rate_by_index(st->elrs_rate_idx);
    if (!rf) return 0U;

    st->ota.ota_nonce++;
    uint32_t freq = elrs_fhss_get_curr_freq(&st->fhss);
    if ((st->ota.ota_nonce % rf->fhss_hop_interval) == 0U) {
        freq = elrs_fhss_advance_and_get_freq(&st->fhss);
    }
    if (out_rf_freq_hz) {
        *out_rf_freq_hz = freq;
    }

    memset(out_ota_pkt, 0, 16U);
    bool on_sync_ch = elrs_fhss_on_sync_channel(&st->fhss);

    /* 1. Send periodic SYNC packet once per sync-channel visit on odd slot (never a telemetry slot) */
    if (on_sync_ch && ((st->ota.ota_nonce % rf->fhss_hop_interval) == 1U)) {
        elrs_sync_data_t sd = {
            .fhss_index      = (uint8_t)(st->fhss.hop_ptr & 0xFFU),
            .nonce           = st->ota.ota_nonce,
            .rf_rate_enum    = (uint8_t)rf->rate_enum,
            .switch_enc_mode = (uint8_t)st->ota.switch_mode,
            .new_tlm_ratio   = 0U, /* TLM_RATIO_NO_TLM: keep multi-RX swarm listeners in continuous RX */
            .gemini_mode     = 0U,
            .ota_protocol    = (uint8_t)((rf->payload_len == ELRS_OTA8_PACKET_SIZE) ? 1U : 0U),
            .uid4            = st->uid[4],
            .uid5            = st->uid[5],
        };
        elrs_ota_pack_sync(&st->ota, &sd, out_ota_pkt);
        st->sync_pkt_count++;
    } else if (elrs_tunnel_has_pending_tx(&st->tunnel) && ((st->ota.ota_nonce & 0x01U) != 0U)) {
        /* 2. Interleave queued Reticulum / Swarm OTA DATA fragments on odd nonces */
        (void)elrs_tunnel_next_tx_frame(&st->tunnel, &st->ota, out_ota_pkt);
    } else {
        /* 3. Standard ExpressLRS RC Channels frame */
        elrs_ota_pack_rc(&st->ota, st->crsf_channels, st->sw_armed, false, out_ota_pkt);
    }

    st->tx_pkt_count++;
    return rf->payload_len;
}

/* ---------------------------------------------------------------------------
 * 4. L76K GNSS NMEA ($GNGGA / $GNRMC) Parser & Follow-Me Formation Builder
 * --------------------------------------------------------------------------- */
static bool verify_nmea_checksum(const char *line) {
    if (!line || line[0] != '$') return false;
    uint8_t csum = 0U;
    const char *p = line + 1;
    while (*p && *p != '*') {
        csum ^= (uint8_t)(*p);
        p++;
    }
    if (*p != '*') return false;
    unsigned int expected = 0U;
    if (sscanf(p + 1, "%2x", &expected) != 1) return false;
    return csum == (uint8_t)expected;
}

/* Convert NMEA ddmm.mmmm (or dddmm.mmmm) + N/S/E/W hemisphere to deg * 1e7 */
static int32_t parse_nmea_coord_e7(const char *field, char hemi) {
    if (!field || !field[0]) return 0;
    double raw = atof(field);
    int32_t deg = (int32_t)(raw / 100.0);
    double mins = raw - (double)(deg * 100);
    double dec  = (double)deg + (mins / 60.0);
    if (hemi == 'S' || hemi == 'W') {
        dec = -dec;
    }
    return (int32_t)llround(dec * 10000000.0);
}

static size_t split_csv_fields(char *line, char *fields[], size_t max_fields) {
    size_t n = 0U;
    char *p = line;
    while (p && n < max_fields) {
        fields[n++] = p;
        char *comma = strchr(p, ',');
        if (!comma) {
            char *star = strchr(p, '*');
            if (star) *star = '\0';
            break;
        }
        *comma = '\0';
        p = comma + 1;
    }
    return n;
}

static bool parse_nmea_sentence(p4_gnss_state_t *gnss, const char *raw_line) {
    if (!verify_nmea_checksum(raw_line)) return false;

    char buf[128];
    snprintf(buf, sizeof(buf), "%s", raw_line);
    char *f[20] = {0};
    size_t nf = split_csv_fields(buf, f, 20U);
    if (nf < 6U) return false;

    if (strncmp(f[0], "$GNGGA", 6) == 0 || strncmp(f[0], "$GPGGA", 6) == 0) {
        if (nf >= 10U) {
            uint8_t qual = (uint8_t)atoi(f[6]);
            gnss->fix_quality = qual;
            gnss->fix_valid   = (qual > 0U);
            gnss->satellites  = (uint8_t)atoi(f[7]);
            if (gnss->fix_valid && f[2][0] && f[4][0]) {
                gnss->lat_deg_e7 = parse_nmea_coord_e7(f[2], f[3][0]);
                gnss->lon_deg_e7 = parse_nmea_coord_e7(f[4], f[5][0]);
                gnss->alt_mm     = (int32_t)llround(atof(f[9]) * 1000.0);
            }
            gnss->sentences_ok++;
            return true;
        }
    } else if (strncmp(f[0], "$GNRMC", 6) == 0 || strncmp(f[0], "$GPRMC", 6) == 0) {
        if (nf >= 9U) {
            bool active = (f[2][0] == 'A');
            gnss->fix_valid = active;
            if (active && f[3][0] && f[5][0]) {
                gnss->lat_deg_e7      = parse_nmea_coord_e7(f[3], f[4][0]);
                gnss->lon_deg_e7      = parse_nmea_coord_e7(f[5], f[6][0]);
                /* Convert knots to cm/s (1 knot = 51.4444 cm/s) */
                gnss->groundspeed_cms = (uint16_t)lround(atof(f[7]) * 51.4444);
                gnss->heading_cdeg    = (uint16_t)(lround(atof(f[8]) * 100.0) % 36000L);
            }
            gnss->sentences_ok++;
            return true;
        }
    }
    return false;
}

bool p4_trx_gnss_feed_byte(p4_gnss_state_t *gnss, char c) {
    if (!gnss) return false;
    if (c == '$') {
        gnss->line_pos = 0U;
    }
    if (c == '\r' || c == '\n') {
        if (gnss->line_pos > 6U) {
            gnss->line_buf[gnss->line_pos] = '\0';
            gnss->line_pos = 0U;
            return parse_nmea_sentence(gnss, gnss->line_buf);
        }
        gnss->line_pos = 0U;
        return false;
    }
    if (gnss->line_pos + 1U < sizeof(gnss->line_buf)) {
        gnss->line_buf[gnss->line_pos++] = c;
    }
    return false;
}

size_t p4_trx_build_follow_me_formation_pkt(p4_trx_state_t *st,
                                            uint8_t out_pkt[64]) {
    if (!st || !out_pkt) return 0U;
    st->formation_seq++;

    rns_formation_pkt_t f = {};
    memcpy(f.leader_dest, st->rns_id.dest_hash, 16U);
    f.seq          = st->formation_seq;
    f.shape        = st->sw_formation;
    f.spacing_dm   = st->formation_spacing_dm;

    /* Use T-Display P4 L76K GNSS coordinates when valid; otherwise fallback to drone telemetry */
    if (st->gnss.fix_valid) {
        f.leader_lat_e7 = st->gnss.lat_deg_e7;
        f.leader_lon_e7 = st->gnss.lon_deg_e7;
        f.leader_alt_mm = st->gnss.alt_mm;
        f.heading_cdeg  = st->gnss.heading_cdeg;
        float hdg_rad   = ((float)st->gnss.heading_cdeg / 100.0f) * (3.14159265f / 180.0f);
        f.vx_cms        = (int16_t)lroundf((float)st->gnss.groundspeed_cms * cosf(hdg_rad));
        f.vy_cms        = (int16_t)lroundf((float)st->gnss.groundspeed_cms * sinf(hdg_rad));
        f.vz_cms        = 0;
    } else {
        f.leader_lat_e7 = st->drone_tlm.lat_deg_e7 ? st->drone_tlm.lat_deg_e7 : st->gnss.lat_deg_e7;
        f.leader_lon_e7 = st->drone_tlm.lon_deg_e7 ? st->drone_tlm.lon_deg_e7 : st->gnss.lon_deg_e7;
        f.leader_alt_mm = st->drone_tlm.rel_alt_mm ? st->drone_tlm.rel_alt_mm : 25000;
        f.heading_cdeg  = st->drone_tlm.heading_cdeg;
    }

    return rns_build_formation_pkt(&f, out_pkt, 64U);
}

size_t p4_trx_build_default_swarm_bundle_pkt(p4_trx_state_t *st,
                                             uint8_t out_pkt[64]) {
    if (!st || !out_pkt) return 0U;
    st->multi_seq++;

    rns_multi_drone_pkt_t mp = {};
    mp.seq          = st->multi_seq;
    mp.slot_mask    = 0x0FU; /* Slots 1..4 active */
    mp.ref_lat_e7   = st->gnss.lat_deg_e7;
    mp.ref_lon_e7   = st->gnss.lon_deg_e7;
    mp.ref_alt_mm   = st->gnss.alt_mm ? st->gnss.alt_mm : 25000;
    mp.heading_cdeg = st->gnss.heading_cdeg;

    /* Slot 1: -10m N, -10m E, +5m Alt; Slot 2: -10m N, +10m E, +5m Alt; Slot 3: LOITER; Slot 4: RTL */
    mp.slots[0] = {RNS_MULTI_ACT_OFFSET_NED, -100, -100, 5};
    mp.slots[1] = {RNS_MULTI_ACT_OFFSET_NED, -100,  100, 5};
    mp.slots[2] = {RNS_MULTI_ACT_LOITER,        0,    0, 0};
    mp.slots[3] = {RNS_MULTI_ACT_RTL,           0,    0, 0};

    return rns_build_multi_drone_pkt(&mp, out_pkt, 64U);
}

/* ---------------------------------------------------------------------------
 * 5. Landscape RGB565 Software Renderer (1168x540 / 1232x568)
 * --------------------------------------------------------------------------- */
static inline void put_px(uint16_t *fb, uint16_t w, uint16_t h, int x, int y, uint16_t col) {
    if (x >= 0 && x < (int)w && y >= 0 && y < (int)h) {
        fb[(size_t)y * w + (size_t)x] = col;
    }
}

static void fill_rect(uint16_t *fb, uint16_t w, uint16_t h, int x, int y, int rw, int rh, uint16_t col) {
    if (rw <= 0 || rh <= 0) return;
    int x0 = (x < 0) ? 0 : x;
    int y0 = (y < 0) ? 0 : y;
    int x1 = (x + rw > (int)w) ? (int)w : (x + rw);
    int y1 = (y + rh > (int)h) ? (int)h : (y + rh);
    for (int py = y0; py < y1; py++) {
        uint16_t *row = &fb[(size_t)py * w];
        for (int px = x0; px < x1; px++) {
            row[px] = col;
        }
    }
}

static void draw_rect(uint16_t *fb, uint16_t w, uint16_t h, int x, int y, int rw, int rh, uint16_t col) {
    fill_rect(fb, w, h, x, y, rw, 1, col);
    fill_rect(fb, w, h, x, y + rh - 1, rw, 1, col);
    fill_rect(fb, w, h, x, y, 1, rh, col);
    fill_rect(fb, w, h, x + rw - 1, y, 1, rh, col);
}

static void draw_line(uint16_t *fb, uint16_t w, uint16_t h, int x0, int y0, int x1, int y1, uint16_t col) {
    int dx = abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
    int dy = -abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;
    for (;;) {
        put_px(fb, w, h, x0, y0, col);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

static void draw_circle(uint16_t *fb, uint16_t w, uint16_t h, int cx, int cy, int r, uint16_t col) {
    int x = r, y = 0, err = 1 - r;
    while (x >= y) {
        put_px(fb, w, h, cx + x, cy + y, col);
        put_px(fb, w, h, cx + y, cy + x, col);
        put_px(fb, w, h, cx - y, cy + x, col);
        put_px(fb, w, h, cx - x, cy + y, col);
        put_px(fb, w, h, cx - x, cy - y, col);
        put_px(fb, w, h, cx - y, cy - x, col);
        put_px(fb, w, h, cx + y, cy - x, col);
        put_px(fb, w, h, cx + x, cy - y, col);
        y++;
        if (err < 0) {
            err += 2 * y + 1;
        } else {
            x--;
            err += 2 * (y - x + 1);
        }
    }
}

static void fill_circle(uint16_t *fb, uint16_t w, uint16_t h, int cx, int cy, int r, uint16_t col) {
    for (int dy = -r; dy <= r; dy++) {
        int dx = (int)sqrtf((float)(r * r - dy * dy));
        fill_rect(fb, w, h, cx - dx, cy + dy, 2 * dx + 1, 1, col);
    }
}

static void draw_text(uint16_t *fb, uint16_t w, uint16_t h,
                      int x, int y, const char *str, uint16_t col, int scale) {
    if (!str || scale < 1) return;
    int cx = x;
    while (*str) {
        uint8_t ch = (uint8_t)(*str++);
        if (ch < 0x20U || ch > 0x7FU) ch = '?';
        const uint8_t *glyph = k_font5x7[ch - 0x20U];
        for (int gx = 0; gx < 5; gx++) {
            uint8_t bits = glyph[gx];
            for (int gy = 0; gy < 7; gy++) {
                if (bits & (1U << gy)) {
                    fill_rect(fb, w, h, cx + gx * scale, y + gy * scale, scale, scale, col);
                }
            }
        }
        cx += 6 * scale;
    }
}

static void draw_button(uint16_t *fb, uint16_t w, uint16_t h,
                        int x, int y, int bw, int bh,
                        const char *label, uint16_t bg_col, uint16_t border_col, uint16_t text_col) {
    fill_rect(fb, w, h, x, y, bw, bh, bg_col);
    draw_rect(fb, w, h, x, y, bw, bh, border_col);
    int len = (int)strlen(label);
    int tx  = x + (bw - len * 12) / 2;
    if (tx < x + 6) tx = x + 6;
    int ty  = y + (bh - 14) / 2;
    draw_text(fb, w, h, tx, ty, label, text_col, 2);
}

static void render_virtual_gimbal(const p4_trx_state_t *st,
                                  uint16_t *fb,
                                  int cx, int cy, int r,
                                  float norm_x, float norm_y,
                                  bool active_touch,
                                  const char *title) {
    uint16_t w = st->screen_w;
    uint16_t h = st->screen_h;

    fill_rect(fb, w, h, cx - r - 12, cy - r - 28, (r + 12) * 2, (r + 12) * 2 + 24, COL_GIMBAL_WELL);
    draw_rect(fb, w, h, cx - r - 12, cy - r - 28, (r + 12) * 2, (r + 12) * 2 + 24,
              active_touch ? COL_ACCENT_CYAN : COL_PANEL_BORDER);

    draw_text(fb, w, h, cx - (int)strlen(title) * 6, cy - r - 20, title, COL_TEXT_MUTED, 2);

    /* Outer & inner rings + crosshair */
    draw_circle(fb, w, h, cx, cy, r, COL_GIMBAL_GRID);
    draw_circle(fb, w, h, cx, cy, r / 2, COL_GIMBAL_GRID);
    fill_rect(fb, w, h, cx - r, cy, r * 2, 1, COL_GIMBAL_GRID);
    fill_rect(fb, w, h, cx, cy - r, 1, r * 2, COL_GIMBAL_GRID);

    /* Thumb stick reticle */
    int sx = cx + (int)lroundf(norm_x * (float)(r - 14));
    int sy = cy - (int)lroundf(norm_y * (float)(r - 14));
    draw_line(fb, w, h, cx, cy, sx, sy, COL_ACCENT_CYAN);
    fill_circle(fb, w, h, sx, sy, 16, active_touch ? COL_ACCENT_GREEN : COL_ACCENT_CYAN);
    draw_circle(fb, w, h, sx, sy, 18, COL_TEXT_WHITE);
}

void p4_trx_render_landscape_rgb565(const p4_trx_state_t *st,
                                    uint16_t *fb) {
    if (!st || !fb) return;
    uint16_t w = st->screen_w;
    uint16_t h = st->screen_h;

    /* 1. Clear background */
    fill_rect(fb, w, h, 0, 0, w, h, COL_BG_DARK);

    /* 2. Top Status Bar (y = 0 .. 44) */
    fill_rect(fb, w, h, 0, 0, w, 44, COL_TOPBAR_BG);
    fill_rect(fb, w, h, 0, 43, w, 1, COL_PANEL_BORDER);

    char buf[96];
    draw_text(fb, w, h, 14, 14, "T-DISPLAY P4 ELRS+RNS TRX", COL_ACCENT_CYAN, 2);

    const elrs_air_rate_config_t *rf = elrs_get_air_rate_by_index(st->elrs_rate_idx);
    const elrs_fhss_domain_info_t *dinfo = elrs_fhss_get_domain_info(st->elrs_domain);
    snprintf(buf, sizeof(buf), "%s %s %ddBm %s",
             rf ? rf->name : "RATE?",
             dinfo ? dinfo->domain_name : "DOM?",
             (int)st->tx_power_dbm,
             (st->antenna_path == P4_ANTENNA_INTERNAL) ? "INT" : "MMCX1");
    draw_text(fb, w, h, 340, 14, buf, COL_TEXT_WHITE, 2);

    snprintf(buf, sizeof(buf), "LQ:%u%% %ddBm", (unsigned)st->uplink_lq, (int)st->uplink_rssi_dbm);
    draw_text(fb, w, h, 680, 14, buf, st->elrs_locked ? COL_ACCENT_GREEN : COL_ACCENT_AMBER, 2);

    snprintf(buf, sizeof(buf), "BAT:%u.%02uV(%u%%)",
             (unsigned)(st->battery.voltage_mv / 1000U),
             (unsigned)((st->battery.voltage_mv % 1000U) / 10U),
             (unsigned)st->battery.soc_pct);
    draw_text(fb, w, h, 860, 14, buf, COL_ACCENT_GREEN, 2);

    /* Top-right DFU button */
    draw_button(fb, w, h, w - 88, 6, 80, 32, "DFU", COL_ACCENT_RED, COL_TEXT_WHITE, COL_TEXT_WHITE);

    /* 3. Left & Right Virtual Gimbals */
    const char *left_label  = (st->gimbal_mode == P4_GIMBAL_MODE_2) ? "L: THR / YAW (M2)" :
                              (st->gimbal_mode == P4_GIMBAL_MODE_1) ? "L: PIT / YAW (M1)" :
                              (st->gimbal_mode == P4_GIMBAL_MODE_3) ? "L: PIT / ROL (M3)" : "L: THR / ROL (M4)";
    const char *right_label = (st->gimbal_mode == P4_GIMBAL_MODE_2) ? "R: PIT / ROL (M2)" :
                              (st->gimbal_mode == P4_GIMBAL_MODE_1) ? "R: THR / ROL (M1)" :
                              (st->gimbal_mode == P4_GIMBAL_MODE_3) ? "R: THR / YAW (M3)" : "R: PIT / YAW (M4)";

    render_virtual_gimbal(st, fb, 142, 268, 118, st->left_x, st->left_y,
                          st->left_touch_id >= 0, left_label);
    render_virtual_gimbal(st, fb, w - 142, 268, 118, st->right_x, st->right_y,
                          st->right_touch_id >= 0, right_label);

    /* 4. Center Console Panel (x = 284 .. w - 284, y = 50 .. h - 70) */
    int cx0 = 284;
    int cw  = (int)w - 568;
    int cy0 = 50;
    int ch  = (int)h - 120;

    fill_rect(fb, w, h, cx0, cy0, cw, ch, COL_PANEL_BG);
    draw_rect(fb, w, h, cx0, cy0, cw, ch, COL_PANEL_BORDER);

    /* 4 Tabs at top of Center Console */
    static const char *const k_tab_names[4] = {"1:HUD", "2:ELRS", "3:SWARM", "4:SPEC"};
    int tab_w = cw / 4;
    for (int t = 0; t < 4; t++) {
        bool sel = ((int)st->active_page == t);
        draw_button(fb, w, h, cx0 + t * tab_w, cy0, tab_w, 38, k_tab_names[t],
                    sel ? COL_ACCENT_CYAN : COL_TOPBAR_BG,
                    COL_PANEL_BORDER,
                    sel ? COL_BG_DARK : COL_TEXT_WHITE);
    }

    int py = cy0 + 48;
    if (st->active_page == P4_PAGE_HUD) {
        /* Artificial Horizon Box */
        int ah_x = cx0 + 14, ah_y = py + 6, ah_w = 210, ah_h = 150;
        fill_rect(fb, w, h, ah_x, ah_y, ah_w, ah_h / 2, rgb565(25, 90, 175));
        fill_rect(fb, w, h, ah_x, ah_y + ah_h / 2, ah_w, ah_h / 2, rgb565(115, 75, 35));
        draw_rect(fb, w, h, ah_x, ah_y, ah_w, ah_h, COL_TEXT_WHITE);

        int pitch_px = (int)lroundf(clampf(st->imu.pitch_deg, -30.0f, 30.0f) * 1.8f);
        int roll_dy  = (int)lroundf(clampf(st->imu.roll_deg, -45.0f, 45.0f) * 0.9f);
        draw_line(fb, w, h, ah_x + 15, ah_y + ah_h / 2 + pitch_px - roll_dy,
                  ah_x + ah_w - 15, ah_y + ah_h / 2 + pitch_px + roll_dy, COL_ACCENT_GREEN);
        snprintf(buf, sizeof(buf), "R:%+.0f P:%+.0f", (double)st->imu.roll_deg, (double)st->imu.pitch_deg);
        draw_text(fb, w, h, ah_x + 12, ah_y + ah_h - 20, buf, COL_TEXT_WHITE, 2);

        /* 8 Primary CRSF Channel Bars */
        int bx = ah_x + ah_w + 16;
        int bw = cw - ah_w - 44;
        for (int c = 0; c < 8; c++) {
            int by = py + 6 + c * 20;
            snprintf(buf, sizeof(buf), "CH%d:%4u", c + 1, (unsigned)st->crsf_channels[c]);
            draw_text(fb, w, h, bx, by, buf, COL_TEXT_WHITE, 1);
            int bar_x = bx + 60;
            int bar_w = bw - 64;
            fill_rect(fb, w, h, bar_x, by, bar_w, 12, COL_GIMBAL_WELL);
            int fill_w = (int)(((int32_t)st->crsf_channels[c] - 172) * bar_w / (1811 - 172));
            if (fill_w < 0) fill_w = 0;
            if (fill_w > bar_w) fill_w = bar_w;
            fill_rect(fb, w, h, bar_x, by, fill_w, 12, (c == 4 && st->sw_armed) ? COL_ACCENT_RED : COL_ACCENT_CYAN);
        }

        /* Drone MAVLink Telemetry Summary */
        int ty = py + 174;
        fill_rect(fb, w, h, cx0 + 14, ty, cw - 28, 128, COL_GIMBAL_WELL);
        draw_rect(fb, w, h, cx0 + 14, ty, cw - 28, 128, COL_PANEL_BORDER);
        snprintf(buf, sizeof(buf), "DRONE TLM: %s  VBAT:%u.%02uV  ALT:%ldm",
                 st->drone_tlm.armed ? "ARMED" : "DISARMED",
                 (unsigned)(st->drone_tlm.vbatt_mv / 1000U),
                 (unsigned)((st->drone_tlm.vbatt_mv % 1000U) / 10U),
                 (long)(st->drone_tlm.rel_alt_mm / 1000L));
        draw_text(fb, w, h, cx0 + 24, ty + 12, buf, COL_ACCENT_GREEN, 2);

        snprintf(buf, sizeof(buf), "P4 GPS: %s%ld.%05ld, %s%ld.%05ld (%u SATS)",
                 (st->gnss.lat_deg_e7 < 0) ? "-" : "",
                 (long)labs(st->gnss.lat_deg_e7 / 10000000L),
                 (long)(labs(st->gnss.lat_deg_e7 % 10000000L) / 100L),
                 (st->gnss.lon_deg_e7 < 0) ? "-" : "",
                 (long)labs(st->gnss.lon_deg_e7 / 10000000L),
                 (long)(labs(st->gnss.lon_deg_e7 % 10000000L) / 100L),
                 (unsigned)st->gnss.satellites);
        draw_text(fb, w, h, cx0 + 24, ty + 44, buf, COL_TEXT_WHITE, 2);

        snprintf(buf, sizeof(buf), "ELRS TX:%lu SYNC:%lu  RNS PEERS:%u",
                 (unsigned long)st->tx_pkt_count,
                 (unsigned long)st->sync_pkt_count,
                 (unsigned)st->rns_peers.count);
        draw_text(fb, w, h, cx0 + 24, ty + 76, buf, COL_ACCENT_CYAN, 2);

        snprintf(buf, sizeof(buf), "SWARM: %s", st->last_swarm_action);
        draw_text(fb, w, h, cx0 + 24, ty + 102, buf, COL_ACCENT_AMBER, 2);

    } else if (st->active_page == P4_PAGE_ELRS) {
        int bw = (cw - 36) / 2;
        draw_button(fb, w, h, cx0 + 12, 110, bw, 50, "RATE -", COL_GIMBAL_WELL, COL_ACCENT_CYAN, COL_TEXT_WHITE);
        draw_button(fb, w, h, cx0 + 24 + bw, 110, bw, 50, "RATE +", COL_GIMBAL_WELL, COL_ACCENT_CYAN, COL_TEXT_WHITE);

        snprintf(buf, sizeof(buf), "DOM: %s", dinfo ? dinfo->domain_name : "DOM?");
        draw_button(fb, w, h, cx0 + 12, 175, bw, 50, buf, COL_GIMBAL_WELL, COL_ACCENT_GREEN, COL_TEXT_WHITE);
        snprintf(buf, sizeof(buf), "ANT: %s", (st->antenna_path == P4_ANTENNA_INTERNAL) ? "INTERNAL" : "MMCX1");
        draw_button(fb, w, h, cx0 + 24 + bw, 175, bw, 50, buf, COL_GIMBAL_WELL, COL_ACCENT_AMBER, COL_TEXT_WHITE);

        draw_button(fb, w, h, cx0 + 12, 240, bw, 50, "TX PWR -", COL_GIMBAL_WELL, COL_PANEL_BORDER, COL_TEXT_WHITE);
        draw_button(fb, w, h, cx0 + 24 + bw, 240, bw, 50, "TX PWR +", COL_GIMBAL_WELL, COL_PANEL_BORDER, COL_TEXT_WHITE);

        snprintf(buf, sizeof(buf), "GIMBAL M%u", (unsigned)st->gimbal_mode);
        draw_button(fb, w, h, cx0 + 12, 305, bw, 50, buf, COL_GIMBAL_WELL, COL_ACCENT_PURPLE, COL_TEXT_WHITE);
        snprintf(buf, sizeof(buf), "ROLE: %u", (unsigned)st->app_mode);
        draw_button(fb, w, h, cx0 + 24 + bw, 305, bw, 50, buf, COL_GIMBAL_WELL, COL_ACCENT_PURPLE, COL_TEXT_WHITE);

        snprintf(buf, sizeof(buf), "UID %02X:%02X:%02X:%02X:%02X:%02X  TCXO 3.0V + AT2401C",
                 st->uid[0], st->uid[1], st->uid[2], st->uid[3], st->uid[4], st->uid[5]);
        draw_text(fb, w, h, cx0 + 16, 372, buf, COL_TEXT_MUTED, 2);

    } else if (st->active_page == P4_PAGE_SWARM) {
        int bw = (cw - 36) / 2;
        draw_button(fb, w, h, cx0 + 12, 110, bw, 50, "OTA PAIR (RNSP)", COL_GIMBAL_WELL, COL_ACCENT_CYAN, COL_TEXT_WHITE);
        draw_button(fb, w, h, cx0 + 24 + bw, 110, bw, 50, "4-DRONE RNSM", COL_GIMBAL_WELL, COL_ACCENT_PURPLE, COL_TEXT_WHITE);

        static const char *const k_fnames[4] = {"FORM: HOLD", "FORM: LINE", "FORM: VEE", "FORM: COL"};
        draw_button(fb, w, h, cx0 + 12, 175, bw, 50, k_fnames[st->sw_formation & 3U],
                    COL_GIMBAL_WELL, COL_ACCENT_GREEN, COL_TEXT_WHITE);
        draw_button(fb, w, h, cx0 + 24 + bw, 175, bw, 50,
                    st->sw_follow_me ? "FOLLOW-ME: ON" : "FOLLOW-ME: OFF",
                    st->sw_follow_me ? COL_ACCENT_GREEN : COL_GIMBAL_WELL, COL_ACCENT_GREEN, COL_TEXT_WHITE);

        draw_button(fb, w, h, cx0 + 12, 240, bw, 50, "RNSC: LOITER", COL_GIMBAL_WELL, COL_ACCENT_AMBER, COL_TEXT_WHITE);
        draw_button(fb, w, h, cx0 + 24 + bw, 240, bw, 50, "RNSC: RTL", COL_GIMBAL_WELL, COL_ACCENT_RED, COL_TEXT_WHITE);

        snprintf(buf, sizeof(buf), "P4 RNS DEST: %02x%02x%02x%02x..%02x%02x  PEERS:%u",
                 st->rns_id.dest_hash[0], st->rns_id.dest_hash[1],
                 st->rns_id.dest_hash[2], st->rns_id.dest_hash[3],
                 st->rns_id.dest_hash[14], st->rns_id.dest_hash[15],
                 (unsigned)st->rns_peers.count);
        draw_text(fb, w, h, cx0 + 16, 312, buf, COL_ACCENT_CYAN, 2);
        draw_text(fb, w, h, cx0 + 16, 344, st->last_swarm_action, COL_ACCENT_AMBER, 2);

    } else {
        /* Page 4: RF Spectrum Waterfall/Bars + Hardware Diagnostics */
        int sx0 = cx0 + 16, sy0 = py + 12, sw = cw - 32, sh = 170;
        fill_rect(fb, w, h, sx0, sy0, sw, sh, COL_GIMBAL_WELL);
        draw_rect(fb, w, h, sx0, sy0, sw, sh, COL_PANEL_BORDER);

        int bin_w = (sw - 16) / (int)P4_SPECTRUM_BINS;
        for (uint8_t b = 0; b < P4_SPECTRUM_BINS; b++) {
            int dbm = st->spectrum_dbm[b];
            int bh  = (dbm + 120) * (sh - 28) / 90;
            if (bh < 4) bh = 4;
            if (bh > sh - 24) bh = sh - 24;
            uint16_t bcol = (dbm > -65) ? COL_ACCENT_RED : (dbm > -85) ? COL_ACCENT_AMBER : COL_ACCENT_CYAN;
            fill_rect(fb, w, h, sx0 + 8 + b * bin_w, sy0 + sh - 8 - bh, bin_w - 2, bh, bcol);
        }
        snprintf(buf, sizeof(buf), "LR2021 SPECTRUM %lu.%01lu MHz",
                 (unsigned long)(st->spectrum_center_hz / 1000000UL),
                 (unsigned long)((st->spectrum_center_hz % 1000000UL) / 100000UL));
        draw_text(fb, w, h, sx0 + 10, sy0 + 8, buf, COL_TEXT_WHITE, 2);

        snprintf(buf, sizeof(buf), "IMU(%s): R=%+.1f P=%+.1f  BQ27220:%umV %dmA",
                 st->imu.is_icm20948 ? "ICM20948" : "QMI8658",
                 (double)st->imu.roll_deg, (double)st->imu.pitch_deg,
                 (unsigned)st->battery.voltage_mv, (int)st->battery.current_ma);
        draw_text(fb, w, h, sx0, sy0 + sh + 16, buf, COL_TEXT_WHITE, 2);

        int bw = (cw - 36) / 2;
        draw_button(fb, w, h, cx0 + 12, 335, bw, 48,
                    st->haptics_enabled ? "HAPTIC: ON" : "HAPTIC: OFF",
                    COL_GIMBAL_WELL, COL_ACCENT_GREEN, COL_TEXT_WHITE);
        draw_button(fb, w, h, cx0 + 24 + bw, 335, bw, 48,
                    st->audio_enabled ? "AUDIO: ON" : "AUDIO: OFF",
                    COL_GIMBAL_WELL, COL_ACCENT_CYAN, COL_TEXT_WHITE);
    }

    /* 5. Bottom 8-Switch Quick Bar (y = h - 62 .. h - 4) */
    int sw_w = (int)w / 8;
    int sy   = (int)h - 60;
    for (int i = 0; i < 8; i++) {
        char lbl[20];
        bool active = false;
        switch (i) {
            case 0: snprintf(lbl, sizeof(lbl), st->sw_armed ? "ARMED" : "DISARM"); active = st->sw_armed; break;
            case 1: snprintf(lbl, sizeof(lbl), "FM:%u", (unsigned)st->sw_flight_mode); active = (st->sw_flight_mode > 0U); break;
            case 2: snprintf(lbl, sizeof(lbl), "LOITER"); active = st->sw_loiter; break;
            case 3: snprintf(lbl, sizeof(lbl), "RTL"); active = st->sw_rtl; break;
            case 4: snprintf(lbl, sizeof(lbl), "FORM:%u", (unsigned)st->sw_formation); active = (st->sw_formation > 0U); break;
            case 5: snprintf(lbl, sizeof(lbl), "FOLLOW"); active = st->sw_follow_me; break;
            case 6: snprintf(lbl, sizeof(lbl), "TILT"); active = st->sw_imu_tilt_steer; break;
            default: snprintf(lbl, sizeof(lbl), "PAGE>>"); active = false; break;
        }
        uint16_t bg = active ? ((i == 0 || i == 3) ? COL_ACCENT_RED : COL_ACCENT_GREEN) : COL_TOPBAR_BG;
        uint16_t fg = active ? COL_BG_DARK : COL_TEXT_WHITE;
        draw_button(fb, w, h, i * sw_w + 2, sy, sw_w - 4, 54, lbl, bg, COL_PANEL_BORDER, fg);
    }
}

void p4_trx_blit_landscape_to_portrait_rgb565(const uint16_t *landscape_fb,
                                              uint16_t *portrait_fb,
                                              uint16_t landscape_w,
                                              uint16_t landscape_h) {
    if (!landscape_fb || !portrait_fb) return;
    /* Native MIPI-DSI panel is Portrait (portrait_w = landscape_h, portrait_h = landscape_w).
     * Map Landscape (lx, ly) -> Portrait (px = (landscape_h - 1) - ly, py = lx).
     */
    uint16_t port_w = landscape_h;
    for (uint16_t ly = 0; ly < landscape_h; ly++) {
        uint16_t px = (uint16_t)((landscape_h - 1U) - ly);
        const uint16_t *src_row = &landscape_fb[(size_t)ly * landscape_w];
        for (uint16_t lx = 0; lx < landscape_w; lx++) {
            portrait_fb[(size_t)lx * port_w + px] = src_row[lx];
        }
    }
}
