/* SPDX-License-Identifier: AGPL-3.0-or-later */
/*
 * Bare-metal nRF54L15 + Semtech LR2021 ExpressLRS Active TX / Active RX /
 * Passive OTA Monitor / High-Speed RF Bridge firmware.
 */

#include "../hal/hal_nrf54l15.h"
#include "../radio/lr2021_driver.h"
#include "../elrs/elrs_protocol.h"
#include "../crsf/crsf.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef enum {
    APP_MODE_IDLE    = 0,
    APP_MODE_PONG    = 1,
    APP_MODE_ELRS_TX = 2,
    APP_MODE_ELRS_RX = 3,
    APP_MODE_MONITOR = 4,
    APP_MODE_BRIDGE  = 5,
} app_mode_t;

typedef enum {
    LOCK_DISCONNECTED = 0,
    LOCK_TENTATIVE    = 1,
    LOCK_CONNECTED    = 2,
} lock_state_t;

static app_mode_t          g_app_mode          = APP_MODE_IDLE;
static lock_state_t        g_lock_state        = LOCK_DISCONNECTED;
static elrs_domain_t       g_domain            = ELRS_DOMAIN_FCC915;
static uint8_t             g_rate_idx          = 3U; /* Default: 900_200HZ (LoRa SF6 BW500) */
static int8_t              g_tx_power_dbm      = 10; /* Safe +10 dBm (10 mW) bench default */
static bool                g_monitor_verbose   = true;

static elrs_ota_state_t    g_ota;
static elrs_fhss_state_t   g_fhss;

/* RC channel state (16 channels in 11-bit CRSF ticks 172..1811, center 992 = 1500 us) */
static uint16_t            g_rc_channels[ELRS_NUM_CHANNELS];
static bool                g_rc_is_armed       = false;
static crsf_parser_t       g_crsf_parser;

/* Timing & FHSS lock tracking */
static volatile uint32_t   g_last_tock_us      = 0U;
static volatile uint32_t   g_last_rx_irq_us    = 0U;
static volatile uint32_t   g_last_valid_pkt_ms = 0U;
static volatile uint32_t   g_last_sync_tx_ms   = 0U;
static volatile bool       g_got_pkt_this_slot = false;
static volatile int32_t    g_last_phase_err_us = 0;
static volatile uint8_t    g_tlm_denom         = 64U;
static volatile uint16_t   g_rf_Consec_good    = 0U;

/* Diagnostic & Link Statistics */
static volatile uint32_t   g_crc_ok_count      = 0U;
static volatile uint32_t   g_crc_fail_count    = 0U;
static volatile uint32_t   g_sync_rx_count     = 0U;
static volatile uint32_t   g_rc_rx_count       = 0U;
static volatile uint32_t   g_tlm_rx_count      = 0U;
static volatile uint32_t   g_fhss_hop_count    = 0U;
static volatile int8_t     g_last_rssi_dbm     = -120;
static volatile int8_t     g_last_snr_q2       = 0;
static volatile uint8_t    g_downlink_rssi     = 0U;
static volatile uint8_t    g_downlink_lq       = 0U;
static volatile int8_t     g_downlink_snr      = 0;

/* Deferred monitor log ring (ISR -> main loop) so UART printing never blocks FHSS ISR */
typedef struct {
    uint32_t timestamp_us;
    uint32_t freq_hz;
    uint8_t  nonce;
    uint8_t  hop_idx;
    uint8_t  pkt_type;
    bool     crc_ok;
    int8_t   rssi_dbm;
    int8_t   snr_q2;
    uint16_t ch_us[4];
    uint8_t  raw[13];
    uint8_t  raw_len;
} mon_event_t;

#define MON_RING_SIZE 32U
static mon_event_t          g_mon_ring[MON_RING_SIZE];
static volatile uint32_t    g_mon_head = 0U;
static volatile uint32_t    g_mon_tail = 0U;

/* Ping-pong / Bridge state */
static volatile bool        g_ping_rx_flag = false;
static uint8_t              g_ping_rx_buf[64];
static uint8_t              g_ping_rx_len  = 0U;
static lr2021_pkt_stats_t   g_ping_rx_stats;
static uint32_t             g_bridge_freq_hz = 2440000000UL;
static uint8_t              g_bridge_pkt_len = 32U;

static void push_mon_event(uint32_t freq_hz,
                           uint8_t nonce,
                           uint8_t hop_idx,
                           uint8_t pkt_type,
                           bool crc_ok,
                           const lr2021_pkt_stats_t *stats,
                           const uint8_t *raw,
                           uint8_t raw_len) {
    uint32_t next = (g_mon_head + 1U) & (MON_RING_SIZE - 1U);
    if (next == g_mon_tail) {
        return;
    }
    mon_event_t *ev = &g_mon_ring[g_mon_head];
    ev->timestamp_us = hal_micros();
    ev->freq_hz      = freq_hz;
    ev->nonce        = nonce;
    ev->hop_idx      = hop_idx;
    ev->pkt_type     = pkt_type;
    ev->crc_ok       = crc_ok;
    ev->rssi_dbm     = stats->rssi_dbm;
    ev->snr_q2       = stats->snr_q2_db;
    for (int i = 0; i < 4; i++) {
        ev->ch_us[i] = crsf_ticks_to_us(g_rc_channels[i]);
    }
    ev->raw_len = (raw_len > 13U) ? 13U : raw_len;
    for (uint8_t i = 0; i < ev->raw_len; i++) {
        ev->raw[i] = raw[i];
    }
    g_mon_head = next;
}

static bool is_telemetry_slot(uint8_t nonce, uint8_t tlm_denom) {
    if (tlm_denom <= 1U) {
        return false;
    }
    return (nonce % tlm_denom) == 0U;
}

static void configure_radio_for_current_rate(void) {
    const elrs_air_rate_config_t *rate = elrs_get_air_rate_by_index(g_rate_idx);
    if (rate == NULL) {
        return;
    }

    /* Update domain automatically if rate is 2.4 GHz vs Sub-GHz */
    bool rate_is_2g4 = (rate->phy_type == ELRS_PHY_LORA_2G4 ||
                        rate->phy_type == ELRS_PHY_FLRC_2G4 ||
                        rate->phy_type == ELRS_PHY_FSK_2G4);
    if (rate_is_2g4 && g_domain != ELRS_DOMAIN_ISM2G4 && g_domain != ELRS_DOMAIN_CE2G4) {
        g_domain = ELRS_DOMAIN_ISM2G4;
    } else if (!rate_is_2g4 && (g_domain == ELRS_DOMAIN_ISM2G4 || g_domain == ELRS_DOMAIN_CE2G4)) {
        g_domain = ELRS_DOMAIN_FCC915;
    }

    elrs_ota_set_mode(&g_ota, rate->payload_len, g_ota.switch_mode);
    elrs_fhss_init(&g_fhss, g_domain, g_ota.uid_seed);
    g_tlm_denom = rate->tlm_denom_default;

    uint32_t init_freq = elrs_fhss_get_initial_freq(&g_fhss);
    bool invert_iq     = (g_ota.uid[5] & 0x01U) != 0U;

    if (rate->phy_type == ELRS_PHY_LORA_900 || rate->phy_type == ELRS_PHY_LORA_2G4) {
        lr2021_config_lora(rate->bw,
                           rate->sf,
                           rate->cr,
                           init_freq,
                           rate->preamble_len,
                           invert_iq,
                           rate->payload_len,
                           false, /* implicit header */
                           false  /* OTA CRC in payload */);
    } else if (rate->phy_type == ELRS_PHY_FLRC_2G4) {
        lr2021_config_flrc(rate->bw,
                           rate->cr,
                           rate->sf,
                           init_freq,
                           rate->preamble_len,
                           g_ota.uid_seed,
                           g_ota.crc_init,
                           rate->payload_len);
    } else {
        uint32_t bitrate = (uint32_t)rate->bw * 10000UL;
        uint32_t fdev    = (uint32_t)rate->cr * 1000UL;
        bool use_fec     = (rate->phy_type == ELRS_PHY_FSK_2G4);
        lr2021_config_fsk(bitrate,
                          rate->sf,
                          fdev,
                          init_freq,
                          rate->preamble_len,
                          g_ota.uid[5],
                          g_ota.uid[4],
                          rate->payload_len,
                          use_fec);
    }

    lr2021_set_tx_power_dbm(g_tx_power_dbm, !rate_is_2g4);
    hal_hwtimer_update_interval(rate->interval_us);
}

/* ============================================================================
 * ExpressLRS Hardware Timer Tick / Tock & Radio DIO8 ISR Callbacks
 * ============================================================================ */

static void elrs_hwtimer_tick_cb(void) {
    /* Halfway through the packet period: nothing required here; hop happens at Tock */
}

static void elrs_hwtimer_tock_cb(void) {
    g_last_tock_us = hal_micros();
    const elrs_air_rate_config_t *rate = elrs_get_air_rate_by_index(g_rate_idx);
    if (rate == NULL) {
        return;
    }

    if (g_app_mode == APP_MODE_ELRS_TX) {
        /* Advance nonce & FHSS hop on TX */
        g_ota.ota_nonce++;
        uint32_t curr_freq = elrs_fhss_get_curr_freq(&g_fhss);
        if ((g_ota.ota_nonce % rate->fhss_hop_interval) == 0U) {
            curr_freq = elrs_fhss_advance_and_get_freq(&g_fhss);
            g_fhss_hop_count++;
            lr2021_set_frequency(curr_freq, false);
        }

        /* Check if this slot is a downlink telemetry receive slot */
        if (is_telemetry_slot(g_ota.ota_nonce, g_tlm_denom)) {
            lr2021_rx_nb();
            return;
        }

        uint8_t pkt[ELRS_OTA8_PACKET_SIZE];
        uint32_t now_ms = hal_millis();
        bool on_sync_ch = elrs_fhss_on_sync_channel(&g_fhss);
        uint32_t sync_interval_ms = (g_tlm_rx_count > 0U) ? 1000U : 200U;

        if (on_sync_ch && (now_ms - g_last_sync_tx_ms) >= sync_interval_ms) {
            elrs_sync_data_t sync = {
                .fhss_index      = (uint8_t)g_fhss.hop_ptr,
                .nonce           = g_ota.ota_nonce,
                .rf_rate_enum    = (uint8_t)rate->rate_enum,
                .switch_enc_mode = (uint8_t)g_ota.switch_mode,
                .new_tlm_ratio   = 0U, /* TLM_RATIO_STD */
                .gemini_mode     = 0U,
                .ota_protocol    = 0U,
                .uid4            = g_ota.uid[4],
                .uid5            = g_ota.uid[5],
            };
            elrs_ota_pack_sync(&g_ota, &sync, pkt);
            g_last_sync_tx_ms = now_ms;
        } else {
            elrs_ota_pack_rc(&g_ota, g_rc_channels, g_rc_is_armed, false, pkt);
        }

        lr2021_tx_nb(pkt, rate->payload_len);
        return;
    }

    if (g_app_mode == APP_MODE_ELRS_RX || g_app_mode == APP_MODE_MONITOR) {
        if (g_lock_state == LOCK_DISCONNECTED) {
            /* Stay parked on initial sync channel waiting for SYNC packet */
            return;
        }

        /* Check lock timeout */
        uint32_t now_ms = hal_millis();
        uint32_t timeout_ms = (g_lock_state == LOCK_TENTATIVE) ? 1500U : 3000U;
        if ((now_ms - g_last_valid_pkt_ms) > timeout_ms) {
            g_lock_state    = LOCK_DISCONNECTED;
            g_rf_Consec_good = 0U;
            elrs_fhss_set_index(&g_fhss, 0U);
            uint32_t sync_freq = elrs_fhss_get_initial_freq(&g_fhss);
            lr2021_set_frequency(sync_freq, true);
            hal_led_set(false);
            return;
        }

        /* Advance OtaNonce and hop frequency at exact FHSShopInterval */
        g_ota.ota_nonce++;
        g_got_pkt_this_slot = false;
        uint32_t freq = elrs_fhss_get_curr_freq(&g_fhss);
        if ((g_ota.ota_nonce % rate->fhss_hop_interval) == 0U) {
            freq = elrs_fhss_advance_and_get_freq(&g_fhss);
            g_fhss_hop_count++;
        }

        /* In Active RX mode, transmit LinkStats on telemetry slot; in Monitor mode, stay in RX to sniff both! */
        if (g_app_mode == APP_MODE_ELRS_RX &&
            g_lock_state == LOCK_CONNECTED &&
            is_telemetry_slot(g_ota.ota_nonce, g_tlm_denom)) {
            lr2021_set_frequency(freq, false);
            uint8_t rssi_mag = (g_last_rssi_dbm < 0) ? (uint8_t)(-g_last_rssi_dbm) : 0U;
            elrs_link_stats_data_t ls = {
                .uplink_rssi_1  = rssi_mag,
                .uplink_rssi_2  = rssi_mag,
                .antenna        = 0U,
                .model_match    = 1U,
                .lq             = 100U,
                .true_diversity = 0U,
                .snr_scaled     = g_last_snr_q2,
            };
            uint8_t pkt[ELRS_OTA8_PACKET_SIZE];
            elrs_ota_pack_linkstats(&g_ota, &ls, 0U, false, pkt);
            lr2021_tx_nb(pkt, rate->payload_len);
        } else {
            lr2021_set_frequency(freq, true);
        }
    }
}

static void radio_tx_done_isr(void) {
    if (g_app_mode == APP_MODE_ELRS_RX || g_app_mode == APP_MODE_PONG || g_app_mode == APP_MODE_BRIDGE) {
        lr2021_rx_nb();
    }
}

static void radio_rx_done_isr(const uint8_t *payload, uint8_t len, const lr2021_pkt_stats_t *stats) {
    uint32_t irq_us  = hal_micros();
    g_last_rx_irq_us = irq_us;
    g_last_rssi_dbm  = stats->rssi_dbm;
    g_last_snr_q2    = stats->snr_q2_db;

    if (g_app_mode == APP_MODE_IDLE || g_app_mode == APP_MODE_PONG || g_app_mode == APP_MODE_BRIDGE) {
        if (len > sizeof(g_ping_rx_buf)) {
            len = sizeof(g_ping_rx_buf);
        }
        memcpy(g_ping_rx_buf, payload, len);
        g_ping_rx_len   = len;
        g_ping_rx_stats = *stats;
        g_ping_rx_flag  = true;

        if (g_app_mode == APP_MODE_PONG && len >= 4U && memcmp(payload, "PING", 4U) == 0) {
            uint8_t reply[64];
            memcpy(reply, payload, len);
            memcpy(reply, "PONG", 4U);
            lr2021_tx_nb(reply, len);
        } else if (g_app_mode == APP_MODE_PONG || g_app_mode == APP_MODE_BRIDGE) {
            lr2021_rx_nb();
        }
        return;
    }

    const elrs_air_rate_config_t *rate = elrs_get_air_rate_by_index(g_rate_idx);
    if (rate == NULL) {
        return;
    }

    if (g_app_mode == APP_MODE_ELRS_TX) {
        /* Downlink telemetry received during telemetry slot */
        elrs_link_stats_data_t ls;
        uint8_t pkg_idx = 0U;
        bool ack = false;
        if (elrs_ota_unpack_linkstats(&g_ota, payload, &ls, &pkg_idx, &ack)) {
            g_crc_ok_count++;
            g_tlm_rx_count++;
            g_downlink_rssi = ls.uplink_rssi_1;
            g_downlink_lq   = ls.lq;
            g_downlink_snr  = ls.snr_scaled / 4;
            hal_led_toggle();
        } else {
            g_crc_fail_count++;
        }
        return;
    }

    /* APP_MODE_ELRS_RX or APP_MODE_MONITOR */
    uint8_t pkt_type = payload[0] & 0x03U;
    uint32_t curr_freq = (g_lock_state == LOCK_DISCONNECTED)
                             ? elrs_fhss_get_initial_freq(&g_fhss)
                             : elrs_fhss_get_curr_freq(&g_fhss);

    /* 1. Check SYNC packet first (SYNC CRC uses nonce = 0) */
    if (pkt_type == ELRS_PKT_TYPE_SYNC) {
        elrs_sync_data_t sync;
        if (elrs_ota_unpack_sync(&g_ota, payload, &sync) &&
            sync.uid4 == g_ota.uid[4] &&
            sync.uid5 == g_ota.uid[5]) {
            g_crc_ok_count++;
            g_sync_rx_count++;
            g_last_valid_pkt_ms = hal_millis();
            g_ota.ota_nonce     = sync.nonce;
            elrs_fhss_set_index(&g_fhss, sync.fhss_index);

            if (g_lock_state == LOCK_DISCONNECTED) {
                g_lock_state     = LOCK_TENTATIVE;
                g_rf_Consec_good = 1U;
                /* Start or re-anchor hwTimer so Tock occurs (interval_us / 2) before next expected RX_DONE */
                if (!hal_hwtimer_is_running()) {
                    hal_hwtimer_start();
                }
            } else if (g_lock_state == LOCK_TENTATIVE) {
                g_rf_Consec_good++;
                if (g_rf_Consec_good >= 3U) {
                    g_lock_state = LOCK_CONNECTED;
                    hal_led_set(true);
                }
            }

            /* Phase lock loop: target RX_DONE at (interval_us / 2) after Tock */
            int32_t target_offset_us = (int32_t)(rate->interval_us >> 1U);
            int32_t actual_offset_us = (int32_t)(irq_us - g_last_tock_us);
            int32_t err_us           = actual_offset_us - target_offset_us;
            g_last_phase_err_us      = err_us;
            if (err_us > -2000 && err_us < 2000) {
                hal_hwtimer_phase_shift(err_us / 2);
            }

            if (g_app_mode == APP_MODE_MONITOR && g_monitor_verbose) {
                push_mon_event(curr_freq, g_ota.ota_nonce, (uint8_t)g_fhss.hop_ptr,
                               pkt_type, true, stats, payload, len);
            }
            return;
        }
    }

    /* 2. If locked/tentative, validate with current OtaNonce */
    if (g_lock_state != LOCK_DISCONNECTED) {
        bool valid = false;
        if (is_telemetry_slot(g_ota.ota_nonce, g_tlm_denom) && g_app_mode == APP_MODE_MONITOR) {
            /* Sniffing RX downlink telemetry slot in passive monitor mode! */
            elrs_link_stats_data_t ls;
            uint8_t pkg_idx = 0U;
            bool ack = false;
            if (elrs_ota_unpack_linkstats(&g_ota, payload, &ls, &pkg_idx, &ack)) {
                valid = true;
                g_tlm_rx_count++;
                g_downlink_rssi = ls.uplink_rssi_1;
                g_downlink_lq   = ls.lq;
                g_downlink_snr  = ls.snr_scaled / 4;
            }
        } else if (pkt_type == ELRS_PKT_TYPE_RCDATA) {
            bool armed = false, ack = false;
            if (elrs_ota_unpack_rc(&g_ota, payload, g_rc_channels, &armed, &ack)) {
                valid = true;
                g_rc_is_armed = armed;
                g_rc_rx_count++;
            }
        } else if (elrs_ota_validate_crc(&g_ota, payload, g_ota.ota_nonce)) {
            valid = true;
        }

        if (valid) {
            g_crc_ok_count++;
            g_last_valid_pkt_ms = hal_millis();
            g_got_pkt_this_slot = true;
            if (g_lock_state == LOCK_TENTATIVE) {
                g_rf_Consec_good++;
                if (g_rf_Consec_good >= 5U) {
                    g_lock_state = LOCK_CONNECTED;
                    hal_led_set(true);
                }
            }

            /* Continuous phase tracking on uplink packets */
            if (!is_telemetry_slot(g_ota.ota_nonce, g_tlm_denom)) {
                int32_t target_offset_us = (int32_t)(rate->interval_us >> 1U);
                int32_t actual_offset_us = (int32_t)(irq_us - g_last_tock_us);
                int32_t err_us           = actual_offset_us - target_offset_us;
                g_last_phase_err_us      = err_us;
                if (err_us > -1500 && err_us < 1500) {
                    hal_hwtimer_phase_shift(err_us / 4);
                }
            }

            /* In Active RX mode, forward unpacked RC channels to Flight Controller over 420k CRSF UART */
            if (g_app_mode == APP_MODE_ELRS_RX && pkt_type == ELRS_PKT_TYPE_RCDATA) {
                uint8_t crsf_frame[26];
                crsf_build_rc_channels_frame(g_rc_channels, crsf_frame);
                hal_crsf_uart_write(crsf_frame, sizeof(crsf_frame));
            }

            if (g_app_mode == APP_MODE_MONITOR && g_monitor_verbose) {
                push_mon_event(curr_freq, g_ota.ota_nonce, (uint8_t)g_fhss.hop_ptr,
                               pkt_type, true, stats, payload, len);
            }
            return;
        }
    }

    g_crc_fail_count++;
    if (g_app_mode == APP_MODE_MONITOR && g_monitor_verbose && g_lock_state == LOCK_DISCONNECTED) {
        /* If unlocked in monitor mode and a SYNC-type header arrives, check if its self-contained UID4/UID5 CRC matches! */
        if (pkt_type == ELRS_PKT_TYPE_SYNC && (len == ELRS_OTA4_PACKET_SIZE || len == ELRS_OTA8_PACKET_SIZE)) {
            uint8_t candidate_uid[6] = {0, 0, 0, 0, payload[5], payload[6]};
            elrs_ota_state_t tmp_ota;
            elrs_ota_init(&tmp_ota, candidate_uid, len, ELRS_SW_WIDE_OR_8CH);
            if (elrs_ota_validate_crc(&tmp_ota, payload, 0U)) {
                push_mon_event(curr_freq, payload[2], payload[1], pkt_type, true, stats, payload, len);
            }
        }
    }
}

/* ============================================================================
 * Mode Transitions & Diagnostics
 * ============================================================================ */

static const char *mode_to_str(app_mode_t m) {
    switch (m) {
        case APP_MODE_IDLE:    return "IDLE";
        case APP_MODE_PONG:    return "PONG";
        case APP_MODE_ELRS_TX: return "ELRS_TX";
        case APP_MODE_ELRS_RX: return "ELRS_RX";
        case APP_MODE_MONITOR: return "MONITOR";
        case APP_MODE_BRIDGE:  return "BRIDGE";
        default:               return "UNKNOWN";
    }
}

static const char *lock_to_str(lock_state_t l) {
    switch (l) {
        case LOCK_DISCONNECTED: return "DISCONNECTED";
        case LOCK_TENTATIVE:    return "TENTATIVE";
        case LOCK_CONNECTED:    return "CONNECTED";
        default:                return "UNKNOWN";
    }
}

static void start_mode(app_mode_t new_mode) {
    hal_hwtimer_stop();
    lr2021_set_mode(LR2021_MODE_STDBY_XOSC);
    g_app_mode       = new_mode;
    g_lock_state     = LOCK_DISCONNECTED;
    g_rf_Consec_good = 0U;
    hal_led_set(false);

    if (new_mode == APP_MODE_IDLE) {
        hal_usb_uart_printf("[MODE] Switched to IDLE (STDBY_XOSC)\r\n");
        return;
    }

    if (new_mode == APP_MODE_BRIDGE) {
        lr2021_config_flrc(LR2021_RADIO_FLRC_BR_0_650_BW_0_6,
                           LR2021_RADIO_FLRC_CR_1_2,
                           LR2021_RADIO_FLRC_BT_1,
                           g_bridge_freq_hz,
                           32U,
                           0xD391F753UL,
                           0x4E08U,
                           g_bridge_pkt_len);
        lr2021_set_tx_power_dbm(g_tx_power_dbm, g_bridge_freq_hz < 1000000000UL);
        lr2021_rx_nb();
        hal_usb_uart_printf("[MODE] Switched to BRIDGE (FLRC %lu Hz, %uB payload, RX continuous)\r\n",
                            (unsigned long)g_bridge_freq_hz, g_bridge_pkt_len);
        return;
    }

    configure_radio_for_current_rate();
    const elrs_air_rate_config_t *rate = elrs_get_air_rate_by_index(g_rate_idx);

    if (new_mode == APP_MODE_ELRS_TX) {
        g_lock_state      = LOCK_CONNECTED;
        g_last_sync_tx_ms = 0U;
        hal_hwtimer_init(rate->interval_us, elrs_hwtimer_tick_cb, elrs_hwtimer_tock_cb);
        hal_hwtimer_start();
        hal_led_set(true);
        hal_usb_uart_printf("[MODE] Started ELRS_TX (rate=%s interval=%lu us domain=%s init_freq=%lu Hz power=%d dBm)\r\n",
                            rate->name,
                            (unsigned long)rate->interval_us,
                            g_fhss.info->domain_name,
                            (unsigned long)elrs_fhss_get_initial_freq(&g_fhss),
                            (int)g_tx_power_dbm);
    } else if (new_mode == APP_MODE_ELRS_RX || new_mode == APP_MODE_MONITOR) {
        hal_hwtimer_init(rate->interval_us, elrs_hwtimer_tick_cb, elrs_hwtimer_tock_cb);
        uint32_t sync_freq = elrs_fhss_get_initial_freq(&g_fhss);
        lr2021_set_frequency(sync_freq, true);
        hal_usb_uart_printf("[MODE] Started %s waiting for SYNC on %lu Hz (rate=%s domain=%s UID=%02X:%02X:%02X:%02X:%02X:%02X)\r\n",
                            mode_to_str(new_mode),
                            (unsigned long)sync_freq,
                            rate->name,
                            g_fhss.info->domain_name,
                            g_ota.uid[0], g_ota.uid[1], g_ota.uid[2],
                            g_ota.uid[3], g_ota.uid[4], g_ota.uid[5]);
    }
}

static void print_status(void) {
    const elrs_air_rate_config_t *rate = elrs_get_air_rate_by_index(g_rate_idx);
    uint16_t ver = lr2021_get_version();
    uint16_t err = lr2021_get_errors();

    hal_usb_uart_printf(
        "STATUS mode=%s lock=%s chip=0x%04X err=0x%04X domain=%s rate=%s(%u) "
        "uid=%02X:%02X:%02X:%02X:%02X:%02X pwr=%ddBm "
        "irq=%lu tx=%lu rx=%lu crc_ok=%lu crc_err=%lu sync_rx=%lu rc_rx=%lu tlm_rx=%lu hops=%lu "
        "rssi=%ddBm snr=%d.%02ddB phase_err=%ldus\r\n",
        mode_to_str(g_app_mode),
        lock_to_str(g_lock_state),
        ver,
        err,
        g_fhss.info->domain_name,
        rate ? rate->name : "NONE",
        g_rate_idx,
        g_ota.uid[0], g_ota.uid[1], g_ota.uid[2],
        g_ota.uid[3], g_ota.uid[4], g_ota.uid[5],
        (int)g_tx_power_dbm,
        (unsigned long)lr2021_get_irq_count(),
        (unsigned long)lr2021_get_tx_done_count(),
        (unsigned long)lr2021_get_rx_done_count(),
        (unsigned long)g_crc_ok_count,
        (unsigned long)g_crc_fail_count,
        (unsigned long)g_sync_rx_count,
        (unsigned long)g_rc_rx_count,
        (unsigned long)g_tlm_rx_count,
        (unsigned long)g_fhss_hop_count,
        (int)g_last_rssi_dbm,
        (int)(g_last_snr_q2 / 4),
        (int)(abs(g_last_snr_q2 % 4) * 25),
        (long)g_last_phase_err_us
    );

    hal_usb_uart_printf("RC_US");
    for (int i = 0; i < 8; i++) {
        hal_usb_uart_printf(" ch%d=%u", i + 1, crsf_ticks_to_us(g_rc_channels[i]));
    }
    hal_usb_uart_printf(" armed=%u\r\n", g_rc_is_armed ? 1U : 0U);
}

static void handle_cli_command(char *line) {
    while (*line && isspace((unsigned char)*line)) {
        line++;
    }
    size_t len = strlen(line);
    while (len > 0U && isspace((unsigned char)line[len - 1U])) {
        line[--len] = '\0';
    }
    if (len == 0U) {
        return;
    }

    char *cmd = strtok(line, " ");
    if (cmd == NULL) {
        return;
    }

    if (strcmp(cmd, "help") == 0) {
        hal_usb_uart_puts(
            "Commands:\r\n"
            "  status                         - Show radio, FHSS, counters, RSSI/SNR, and RC channels\r\n"
            "  rates                          - List supported ExpressLRS air rates (Sub-GHz & 2.4 GHz)\r\n"
            "  phrase <binding_phrase>        - Set UID from ExpressLRS binding phrase (MD5)\r\n"
            "  uid <XX:XX:XX:XX:XX:XX>        - Set 6-byte ExpressLRS UID directly\r\n"
            "  domain <fcc915|au915|eu868|ism2g4> - Set regulatory frequency domain\r\n"
            "  rate <idx_or_name>             - Select ExpressLRS air rate\r\n"
            "  power <dbm>                    - Set TX output power in dBm (-20..22)\r\n"
            "  mode <idle|tx|rx|monitor|bridge> - Switch operating mode\r\n"
            "  rc <ch1_us> [ch2_us .. ch16_us]- Set RC channels in us (988..2012)\r\n"
            "  ping <freq_hz> <lora|flrc> [n] - Transmit n ping packets and await PONG\r\n"
            "  pong <freq_hz> <lora|flrc>     - Listen on freq_hz and auto-reply PONG\r\n"
            "  txpkt <hex_bytes>              - Send raw packet in BRIDGE mode\r\n"
            "  scan <900|2400>                - Sweep RSSI across band\r\n"
        );
        return;
    }

    if (strcmp(cmd, "status") == 0) {
        print_status();
        return;
    }

    if (strcmp(cmd, "rates") == 0) {
        uint8_t n = elrs_get_air_rate_count();
        for (uint8_t i = 0U; i < n; i++) {
            const elrs_air_rate_config_t *r = elrs_get_air_rate_by_index(i);
            hal_usb_uart_printf("  [%2u] %-14s enum=%2u interval=%5lu us pkt=%2uB hop=%u tlm=1:%u\r\n",
                                i, r->name, r->rate_enum, (unsigned long)r->interval_us,
                                r->payload_len, r->fhss_hop_interval, r->tlm_denom_default);
        }
        return;
    }

    if (strcmp(cmd, "phrase") == 0) {
        char *rest = strtok(NULL, "");
        if (rest == NULL) {
            hal_usb_uart_puts("ERR: usage: phrase <binding_phrase>\r\n");
            return;
        }
        while (*rest && isspace((unsigned char)*rest)) rest++;
        uint8_t new_uid[6];
        elrs_uid_from_bind_phrase(rest, new_uid);
        const elrs_air_rate_config_t *r = elrs_get_air_rate_by_index(g_rate_idx);
        elrs_ota_init(&g_ota, new_uid, r ? r->payload_len : 8U, g_ota.switch_mode);
        elrs_fhss_init(&g_fhss, g_domain, g_ota.uid_seed);
        hal_usb_uart_printf("OK phrase=\"%s\" -> UID=%02X:%02X:%02X:%02X:%02X:%02X seed=0x%08lX crc_init=0x%04X sync_freq=%lu Hz\r\n",
                            rest,
                            g_ota.uid[0], g_ota.uid[1], g_ota.uid[2],
                            g_ota.uid[3], g_ota.uid[4], g_ota.uid[5],
                            (unsigned long)g_ota.uid_seed,
                            g_ota.crc_init,
                            (unsigned long)elrs_fhss_get_initial_freq(&g_fhss));
        if (g_app_mode != APP_MODE_IDLE) {
            start_mode(g_app_mode);
        }
        return;
    }

    if (strcmp(cmd, "uid") == 0) {
        char *arg = strtok(NULL, " ");
        uint8_t new_uid[6];
        if (arg == NULL || !elrs_uid_parse_string(arg, new_uid)) {
            hal_usb_uart_puts("ERR: usage: uid XX:XX:XX:XX:XX:XX\r\n");
            return;
        }
        const elrs_air_rate_config_t *r = elrs_get_air_rate_by_index(g_rate_idx);
        elrs_ota_init(&g_ota, new_uid, r ? r->payload_len : 8U, g_ota.switch_mode);
        elrs_fhss_init(&g_fhss, g_domain, g_ota.uid_seed);
        hal_usb_uart_printf("OK UID=%02X:%02X:%02X:%02X:%02X:%02X seed=0x%08lX crc_init=0x%04X sync_freq=%lu Hz\r\n",
                            g_ota.uid[0], g_ota.uid[1], g_ota.uid[2],
                            g_ota.uid[3], g_ota.uid[4], g_ota.uid[5],
                            (unsigned long)g_ota.uid_seed,
                            g_ota.crc_init,
                            (unsigned long)elrs_fhss_get_initial_freq(&g_fhss));
        if (g_app_mode != APP_MODE_IDLE) {
            start_mode(g_app_mode);
        }
        return;
    }

    if (strcmp(cmd, "domain") == 0) {
        char *arg = strtok(NULL, " ");
        if (arg == NULL) {
            hal_usb_uart_puts("ERR: usage: domain <fcc915|au915|eu868|ism2g4>\r\n");
            return;
        }
        if (strcasecmp(arg, "fcc915") == 0) g_domain = ELRS_DOMAIN_FCC915;
        else if (strcasecmp(arg, "au915") == 0) g_domain = ELRS_DOMAIN_AU915;
        else if (strcasecmp(arg, "eu868") == 0) g_domain = ELRS_DOMAIN_EU868;
        else if (strcasecmp(arg, "ism2g4") == 0 || strcasecmp(arg, "ism2400") == 0) g_domain = ELRS_DOMAIN_ISM2G4;
        else if (strcasecmp(arg, "ce2g4") == 0) g_domain = ELRS_DOMAIN_CE2G4;
        else {
            hal_usb_uart_printf("ERR: unknown domain %s\r\n", arg);
            return;
        }
        elrs_fhss_init(&g_fhss, g_domain, g_ota.uid_seed);
        hal_usb_uart_printf("OK domain=%s channels=%lu sync_freq=%lu Hz\r\n",
                            g_fhss.info->domain_name,
                            (unsigned long)g_fhss.info->freq_count,
                            (unsigned long)elrs_fhss_get_initial_freq(&g_fhss));
        if (g_app_mode != APP_MODE_IDLE) {
            start_mode(g_app_mode);
        }
        return;
    }

    if (strcmp(cmd, "rate") == 0) {
        char *arg = strtok(NULL, " ");
        if (arg == NULL) {
            hal_usb_uart_puts("ERR: usage: rate <idx_or_name>\r\n");
            return;
        }
        uint8_t n = elrs_get_air_rate_count();
        bool found = false;
        for (uint8_t i = 0U; i < n; i++) {
            const elrs_air_rate_config_t *r = elrs_get_air_rate_by_index(i);
            if (strcasecmp(arg, r->name) == 0) {
                g_rate_idx = i;
                found = true;
                break;
            }
        }
        if (!found && isdigit((unsigned char)arg[0])) {
            uint8_t idx = (uint8_t)strtoul(arg, NULL, 10);
            if (idx < n) {
                g_rate_idx = idx;
                found = true;
            }
        }
        if (!found) {
            hal_usb_uart_printf("ERR: invalid rate %s\r\n", arg);
            return;
        }
        const elrs_air_rate_config_t *r = elrs_get_air_rate_by_index(g_rate_idx);
        hal_usb_uart_printf("OK rate=%s (%u) interval=%lu us pkt=%uB\r\n",
                            r->name, g_rate_idx, (unsigned long)r->interval_us, r->payload_len);
        if (g_app_mode != APP_MODE_IDLE) {
            start_mode(g_app_mode);
        }
        return;
    }

    if (strcmp(cmd, "power") == 0) {
        char *arg = strtok(NULL, " ");
        if (arg == NULL) {
            hal_usb_uart_puts("ERR: usage: power <dbm>\r\n");
            return;
        }
        g_tx_power_dbm = (int8_t)strtol(arg, NULL, 10);
        lr2021_set_tx_power_dbm(g_tx_power_dbm, true);
        lr2021_set_tx_power_dbm(g_tx_power_dbm, false);
        hal_usb_uart_printf("OK power=%d dBm\r\n", (int)g_tx_power_dbm);
        return;
    }

    if (strcmp(cmd, "mode") == 0) {
        char *arg = strtok(NULL, " ");
        if (arg == NULL) {
            hal_usb_uart_puts("ERR: usage: mode <idle|tx|rx|monitor|bridge>\r\n");
            return;
        }
        if (strcasecmp(arg, "idle") == 0) start_mode(APP_MODE_IDLE);
        else if (strcasecmp(arg, "tx") == 0) start_mode(APP_MODE_ELRS_TX);
        else if (strcasecmp(arg, "rx") == 0) start_mode(APP_MODE_ELRS_RX);
        else if (strcasecmp(arg, "monitor") == 0) start_mode(APP_MODE_MONITOR);
        else if (strcasecmp(arg, "bridge") == 0) start_mode(APP_MODE_BRIDGE);
        else hal_usb_uart_printf("ERR: unknown mode %s\r\n", arg);
        return;
    }

    if (strcmp(cmd, "rc") == 0) {
        unsigned ch = 0U;
        char *tok;
        while (ch < ELRS_NUM_CHANNELS && (tok = strtok(NULL, " ")) != NULL) {
            uint16_t us = (uint16_t)strtoul(tok, NULL, 10);
            g_rc_channels[ch] = crsf_us_to_ticks(us);
            if (ch == 4U) {
                g_rc_is_armed = (us >= 1500U);
            }
            ch++;
        }
        hal_usb_uart_printf("OK updated %u RC channels (ch1=%u ch2=%u ch3=%u ch4=%u armed=%u)\r\n",
                            ch,
                            crsf_ticks_to_us(g_rc_channels[0]),
                            crsf_ticks_to_us(g_rc_channels[1]),
                            crsf_ticks_to_us(g_rc_channels[2]),
                            crsf_ticks_to_us(g_rc_channels[3]),
                            g_rc_is_armed ? 1U : 0U);
        return;
    }

    if (strcmp(cmd, "pong") == 0) {
        char *f_arg = strtok(NULL, " ");
        char *m_arg = strtok(NULL, " ");
        uint32_t freq_hz = f_arg ? (uint32_t)strtoul(f_arg, NULL, 10) : 915000000UL;
        bool use_flrc    = (m_arg && strcasecmp(m_arg, "flrc") == 0);

        hal_hwtimer_stop();
        g_app_mode = APP_MODE_PONG;
        if (use_flrc) {
            lr2021_config_flrc(LR2021_RADIO_FLRC_BR_0_650_BW_0_6,
                               LR2021_RADIO_FLRC_CR_1_2,
                               LR2021_RADIO_FLRC_BT_1,
                               freq_hz, 32U, 0x12345678UL, 0xABCDU, 16U);
        } else {
            uint8_t bw = (freq_hz < 1000000000UL) ? LR2021_RADIO_LORA_BW_500 : LR2021_RADIO_LORA_BW_800;
            lr2021_config_lora(bw, LR2021_RADIO_LORA_SF7, LR2021_RADIO_LORA_CR_4_7,
                               freq_hz, 12U, false, 16U, false, true);
        }
        lr2021_set_tx_power_dbm(g_tx_power_dbm, freq_hz < 1000000000UL);
        lr2021_rx_nb();
        hal_usb_uart_printf("OK PONGresponder active on %lu Hz (%s, 16B payload)\r\n",
                            (unsigned long)freq_hz, use_flrc ? "FLRC" : "LoRa");
        return;
    }

    if (strcmp(cmd, "ping") == 0) {
        char *f_arg = strtok(NULL, " ");
        char *m_arg = strtok(NULL, " ");
        char *c_arg = strtok(NULL, " ");
        uint32_t freq_hz = f_arg ? (uint32_t)strtoul(f_arg, NULL, 10) : 915000000UL;
        bool use_flrc    = (m_arg && strcasecmp(m_arg, "flrc") == 0);
        uint32_t count   = c_arg ? (uint32_t)strtoul(c_arg, NULL, 10) : 5U;

        hal_hwtimer_stop();
        g_app_mode = APP_MODE_IDLE;
        if (use_flrc) {
            lr2021_config_flrc(LR2021_RADIO_FLRC_BR_0_650_BW_0_6,
                               LR2021_RADIO_FLRC_CR_1_2,
                               LR2021_RADIO_FLRC_BT_1,
                               freq_hz, 32U, 0x12345678UL, 0xABCDU, 16U);
        } else {
            uint8_t bw = (freq_hz < 1000000000UL) ? LR2021_RADIO_LORA_BW_500 : LR2021_RADIO_LORA_BW_800;
            lr2021_config_lora(bw, LR2021_RADIO_LORA_SF7, LR2021_RADIO_LORA_CR_4_7,
                               freq_hz, 12U, false, 16U, false, true);
        }
        lr2021_set_tx_power_dbm(g_tx_power_dbm, freq_hz < 1000000000UL);

        uint32_t ok_cnt = 0U;
        for (uint32_t seq = 1U; seq <= count; seq++) {
            uint8_t pkt[16] = {0};
            snprintf((char *)pkt, sizeof(pkt), "PING#%04lu", (unsigned long)seq);
            g_ping_rx_flag = false;
            uint32_t t0 = hal_micros();
            uint32_t tx_done_before = lr2021_get_tx_done_count();
            lr2021_tx_nb(pkt, 16U);

            while (lr2021_get_tx_done_count() == tx_done_before && (hal_micros() - t0) < 50000U) {
                __asm volatile("nop");
            }
            lr2021_rx_nb();

            uint32_t wait_start = hal_micros();
            while (!g_ping_rx_flag && (hal_micros() - wait_start) < 60000U) {
                __asm volatile("nop");
            }
            uint32_t rtt_us = hal_micros() - t0;
            if (g_ping_rx_flag && memcmp(g_ping_rx_buf, "PONG", 4U) == 0) {
                ok_cnt++;
                hal_usb_uart_printf("PING %lu/%lu: PONG rx=\"%.15s\" rtt=%lu us rssi=%d dBm snr=%d.%02d dB\r\n",
                                    (unsigned long)seq, (unsigned long)count,
                                    (const char *)g_ping_rx_buf,
                                    (unsigned long)rtt_us,
                                    (int)g_ping_rx_stats.rssi_dbm,
                                    (int)(g_ping_rx_stats.snr_q2_db / 4),
                                    (int)(abs(g_ping_rx_stats.snr_q2_db % 4) * 25));
            } else {
                hal_usb_uart_printf("PING %lu/%lu: TIMEOUT\r\n", (unsigned long)seq, (unsigned long)count);
            }
            hal_delay_ms(20U);
        }
        lr2021_set_mode(LR2021_MODE_STDBY_XOSC);
        hal_usb_uart_printf("PING_SUMMARY sent=%lu received=%lu loss=%lu%%\r\n",
                            (unsigned long)count, (unsigned long)ok_cnt,
                            (unsigned long)(((count - ok_cnt) * 100U) / (count ? count : 1U)));
        return;
    }

    if (strcmp(cmd, "txpkt") == 0) {
        char *hex = strtok(NULL, " ");
        if (hex == NULL || g_app_mode != APP_MODE_BRIDGE) {
            hal_usb_uart_puts("ERR: txpkt requires BRIDGE mode and hex string\r\n");
            return;
        }
        uint8_t buf[32] = {0};
        size_t slen = strlen(hex);
        size_t blen = 0U;
        for (size_t i = 0; i + 1 < slen && blen < sizeof(buf); i += 2) {
            char pair[3] = {hex[i], hex[i + 1], '\0'};
            buf[blen++] = (uint8_t)strtoul(pair, NULL, 16);
        }
        lr2021_tx_nb(buf, g_bridge_pkt_len);
        hal_usb_uart_printf("OK BRIDGE_TX %u bytes\r\n", (unsigned)blen);
        return;
    }

    if (strcmp(cmd, "scan") == 0) {
        char *band = strtok(NULL, " ");
        bool is_2g4 = (band && (strcmp(band, "2400") == 0 || strcasecmp(band, "2g4") == 0));
        hal_hwtimer_stop();
        uint32_t f_start = is_2g4 ? 2400000000UL : 903000000UL;
        uint32_t f_stop  = is_2g4 ? 2480000000UL : 927000000UL;
        uint32_t f_step  = is_2g4 ? 2000000UL    : 600000UL;

        uint8_t bw = is_2g4 ? LR2021_RADIO_LORA_BW_800 : LR2021_RADIO_LORA_BW_500;
        lr2021_config_lora(bw, LR2021_RADIO_LORA_SF6, LR2021_RADIO_LORA_CR_4_7,
                           f_start, 12U, false, 8U, false, false);

        hal_usb_uart_printf("SCAN_START band=%s start=%lu stop=%lu step=%lu\r\n",
                            is_2g4 ? "2.4GHz" : "900MHz",
                            (unsigned long)f_start, (unsigned long)f_stop, (unsigned long)f_step);
        for (uint32_t f = f_start; f <= f_stop; f += f_step) {
            lr2021_set_mode(LR2021_MODE_FS);
            lr2021_set_frequency(f, true);
            hal_delay_us(600U);
            int8_t max_rssi = -127;
            for (int s = 0; s < 8; s++) {
                int8_t r = lr2021_get_rssi_inst_dbm();
                if (r > max_rssi) max_rssi = r;
                hal_delay_us(150U);
            }
            hal_usb_uart_printf("SCAN freq=%lu rssi=%d\r\n", (unsigned long)f, (int)max_rssi);
        }
        lr2021_set_mode(LR2021_MODE_FS);
        lr2021_clear_errors();
        hal_usb_uart_puts("SCAN_END\r\n");
        if (g_app_mode != APP_MODE_IDLE) {
            start_mode(g_app_mode);
        }
        return;
    }

    hal_usb_uart_printf("ERR: unknown command '%s' (type 'help')\r\n", cmd);
}

int main(void) {
    hal_init();

    /* Initialize RC channels to 1500 us (992 CRSF ticks), throttle (ch3 = index 2) to 1000 us (191) */
    for (unsigned i = 0U; i < ELRS_NUM_CHANNELS; i++) {
        g_rc_channels[i] = ELRS_CRSF_CH_MID;
    }
    g_rc_channels[2] = crsf_us_to_ticks(1000U);
    crsf_parser_init(&g_crsf_parser);

    /* Default binding phrase */
    uint8_t default_uid[6];
    elrs_uid_from_bind_phrase("lr2021-elrs", default_uid);
    elrs_ota_init(&g_ota, default_uid, ELRS_OTA4_PACKET_SIZE, ELRS_SW_WIDE_OR_8CH);
    elrs_fhss_init(&g_fhss, g_domain, g_ota.uid_seed);

    lr2021_set_callbacks(radio_rx_done_isr, radio_tx_done_isr);

    uint16_t chip_ver = 0U;
    bool radio_ok = lr2021_begin(915000000UL, 2440000000UL, &chip_ver);

    hal_usb_uart_printf("\r\n========================================================\r\n");
    hal_usb_uart_printf("LR2021-ELRS Bare-Metal Firmware (XIAO nRF54L15 @ 128MHz)\r\n");
    hal_usb_uart_printf("LR2021 Init: %s (Version=0x%04X, Errors=0x%04X)\r\n",
                        radio_ok ? "OK" : "FAIL", chip_ver, lr2021_get_errors());
    hal_usb_uart_printf("Default UID: %02X:%02X:%02X:%02X:%02X:%02X (phrase=\"lr2021-elrs\")\r\n",
                        g_ota.uid[0], g_ota.uid[1], g_ota.uid[2],
                        g_ota.uid[3], g_ota.uid[4], g_ota.uid[5]);
    hal_usb_uart_printf("========================================================\r\n");

    char cli_buf[192];
    size_t cli_pos = 0U;
    uint32_t last_crsf_linkstats_ms = 0U;

    while (1) {
        /* 1. Process incoming USB CDC CLI bytes */
        int b;
        while ((b = hal_usb_uart_read_byte()) >= 0) {
            char c = (char)b;
            if (c == '\r' || c == '\n') {
                if (cli_pos > 0U) {
                    cli_buf[cli_pos] = '\0';
                    handle_cli_command(cli_buf);
                    cli_pos = 0U;
                }
            } else if (cli_pos + 1U < sizeof(cli_buf)) {
                cli_buf[cli_pos++] = c;
            }
        }

        /* 2. Process incoming external Grove CRSF UART bytes (Handset RC input in TX mode) */
        while ((b = hal_crsf_uart_read_byte()) >= 0) {
            if (crsf_parser_feed_byte(&g_crsf_parser, (uint8_t)b)) {
                uint8_t ftype = g_crsf_parser.buf[2];
                uint8_t flen  = (uint8_t)(g_crsf_parser.expected_len + 2U);
                if (ftype == CRSF_FRAMETYPE_RC_CHANNELS) {
                    (void)crsf_decode_rc_channels_frame(g_crsf_parser.buf, flen, g_rc_channels);
                    g_rc_is_armed = (g_rc_channels[4] > ELRS_CRSF_CH_MID);
                }
            }
        }

        /* 3. In Active RX mode, send periodic CRSF_FRAMETYPE_LINK_STATISTICS (0x14) to Flight Controller */
        uint32_t now_ms = hal_millis();
        if (g_app_mode == APP_MODE_ELRS_RX && g_lock_state == LOCK_CONNECTED &&
            (now_ms - last_crsf_linkstats_ms) >= 100U) {
            last_crsf_linkstats_ms = now_ms;
            uint8_t rssi_mag = (g_last_rssi_dbm < 0) ? (uint8_t)(-g_last_rssi_dbm) : 0U;
            crsf_link_stats_t ls = {
                .uplink_rssi_1   = rssi_mag,
                .uplink_rssi_2   = rssi_mag,
                .uplink_lq       = 100U,
                .uplink_snr      = (int8_t)(g_last_snr_q2 / 4),
                .active_antenna  = 0U,
                .rf_mode         = g_rate_idx,
                .uplink_tx_power = 3U,
                .downlink_rssi   = rssi_mag,
                .downlink_lq     = 100U,
                .downlink_snr    = (int8_t)(g_last_snr_q2 / 4),
            };
            uint8_t frame[14];
            crsf_build_link_stats_frame(&ls, frame);
            hal_crsf_uart_write(frame, sizeof(frame));
        }

        /* 4. Drain deferred MONITOR events or BRIDGE packets to USB console */
        while (g_mon_tail != g_mon_head) {
            mon_event_t ev = g_mon_ring[g_mon_tail];
            g_mon_tail = (g_mon_tail + 1U) & (MON_RING_SIZE - 1U);
            const char *tstr = (ev.pkt_type == ELRS_PKT_TYPE_SYNC) ? "SYNC" :
                               (ev.pkt_type == ELRS_PKT_TYPE_RCDATA) ? "RC" : "DATA";
            hal_usb_uart_printf(
                "MON t=%lu freq=%lu nonce=%3u hop=%3u type=%-4s rssi=%4d snr=%2d ch=[%4u,%4u,%4u,%4u] raw=%02X%02X%02X%02X%02X%02X%02X%02X\r\n",
                (unsigned long)ev.timestamp_us,
                (unsigned long)ev.freq_hz,
                ev.nonce,
                ev.hop_idx,
                tstr,
                (int)ev.rssi_dbm,
                (int)(ev.snr_q2 / 4),
                ev.ch_us[0], ev.ch_us[1], ev.ch_us[2], ev.ch_us[3],
                ev.raw[0], ev.raw[1], ev.raw[2], ev.raw[3],
                ev.raw[4], ev.raw[5], ev.raw[6], ev.raw[7]
            );
        }

        if (g_app_mode == APP_MODE_BRIDGE && g_ping_rx_flag) {
            g_ping_rx_flag = false;
            hal_usb_uart_printf("BRIDGE_RX rssi=%d dBm len=%u hex=",
                                (int)g_ping_rx_stats.rssi_dbm, g_ping_rx_len);
            for (uint8_t i = 0; i < g_ping_rx_len; i++) {
                hal_usb_uart_printf("%02X", g_ping_rx_buf[i]);
            }
            hal_usb_uart_puts("\r\n");
        }
    }
}
