/* SPDX-License-Identifier: AGPL-3.0-or-later */
/*
 * Bare-metal nRF54L15 + Semtech LR2021 ExpressLRS Active TX / Active RX /
 * Passive OTA Monitor / High-Speed RF Bridge firmware.
 */

#include "../hal/hal_nrf54l15.h"
#include "../radio/lr2021_driver.h"
#include "../elrs/elrs_protocol.h"
#include "../crsf/crsf.h"
#include "../rnode/rns_node.h"

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
    APP_MODE_RNODE   = 6,
    APP_MODE_RELAY   = 7,
} app_mode_t;

typedef enum {
    LOCK_DISCONNECTED = 0,
    LOCK_TENTATIVE    = 1,
    LOCK_CONNECTED    = 2,
} lock_state_t;

static app_mode_t          g_app_mode          = APP_MODE_IDLE;
static lock_state_t        g_lock_state        = LOCK_DISCONNECTED;
static elrs_domain_t       g_domain            = ELRS_DOMAIN_EU868;
static elrs_domain_t       g_subghz_domain     = ELRS_DOMAIN_EU868;
static elrs_domain_t       g_2g4_domain        = ELRS_DOMAIN_CE2G4;
static uint8_t             g_rate_idx          = 3U; /* Default: 900_200HZ (LoRa SF6 BW500) */
static int8_t              g_tx_power_dbm      = 10; /* Safe +10 dBm (10 mW) bench default */
static bool                g_monitor_verbose   = true;

static elrs_ota_state_t    g_ota;
static elrs_fhss_state_t   g_fhss;

/* RC channel state (16 channels in 11-bit CRSF ticks 172..1811, center 992 = 1500 us) */
static uint16_t            g_rc_channels[ELRS_NUM_CHANNELS];
static bool                g_rc_is_armed       = false;
static volatile bool       g_crsf_rc_pending   = false;
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
static volatile uint8_t    g_tentative_slots   = 0U;
static volatile uint8_t    g_tentative_hops    = 0U;
static volatile bool       g_tentative_from_pair = false;
static uint16_t            g_chip_version      = 0x0118U;
static uint32_t            g_prev_sync_ch_us   = 0U;
static uint8_t             g_prev_sync_ch_pkt[ELRS_OTA8_PACKET_SIZE];

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

/* Reticulum / RNode / Airborne Relay / FC MAVLink Bridge / ELRS DATA Tunnel state */
typedef struct {
    uint32_t frequency_hz;
    uint32_t bandwidth_hz;
    uint8_t  sf;
    uint8_t  cr;
    int8_t   txpower_dbm;
    uint16_t st_alock;
    uint16_t lt_alock;
    bool     radio_online;
    uint32_t rx_bytes;
    uint32_t tx_bytes;
} rnode_radio_cfg_t;

static rnode_kiss_parser_t  g_kiss_parser;
static rnode_radio_cfg_t    g_rnode_cfg = {
    .frequency_hz = 868000000UL,
    .bandwidth_hz = 125000UL,
    .sf           = 7U,
    .cr           = 5U,
    .txpower_dbm  = 14,
    .st_alock     = 0U,
    .lt_alock     = 0U,
    .radio_online = false,
    .rx_bytes     = 0U,
    .tx_bytes     = 0U,
};
static rns_relay_table_t    g_rns_relay;
static rns_mav_parser_t     g_mav_parser;
static rns_fc_telemetry_t   g_fc_tlm;
static elrs_data_tunnel_t   g_elrs_tunnel;
static uint8_t              g_mav_tx_seq = 0U;

static volatile bool        g_rns_rx_flag = false;
static uint8_t              g_rns_rx_buf[RNODE_LORA_MAX_PKT];
static volatile uint8_t     g_rns_rx_len  = 0U;
static lr2021_pkt_stats_t   g_rns_rx_stats;
static volatile bool        g_rnode_tx_busy = false;
static volatile bool        g_rnode_tx_done_notify = false;

/* Airborne relay delayed rebroadcast queue */
static bool                 g_relay_pending = false;
static uint8_t              g_relay_tx_buf[RNODE_LORA_MAX_PKT];
static uint8_t              g_relay_tx_len  = 0U;
static uint32_t             g_relay_due_ms  = 0U;
static uint32_t             g_beacon_interval_ms = 0U;
static uint32_t             g_last_beacon_ms     = 0U;

/* Reassembled Reticulum packet from ELRS DATA tunnel */
static volatile bool        g_tunnel_rx_ready = false;
static uint8_t              g_tunnel_rx_buf[ELRS_TUNNEL_MAX_MTU];
static volatile uint16_t    g_tunnel_rx_len   = 0U;

static void configure_radio_for_rnode(void) {
    uint8_t bw_code = lr2021_lora_bw_from_hz(g_rnode_cfg.bandwidth_hz);
    uint8_t sf      = (g_rnode_cfg.sf >= 5U && g_rnode_cfg.sf <= 12U) ? g_rnode_cfg.sf : 7U;
    uint8_t cr_code = (g_rnode_cfg.cr >= 5U && g_rnode_cfg.cr <= 8U)
                          ? (uint8_t)(g_rnode_cfg.cr - 4U)
                          : (uint8_t)LR2021_RADIO_LORA_CR_4_5;
    bool is_subghz  = (g_rnode_cfg.frequency_hz < 1000000000UL);

    lr2021_config_lora(bw_code,
                       sf,
                       cr_code,
                       g_rnode_cfg.frequency_hz,
                       18U,   /* RNode 18-symbol preamble */
                       false, /* Normal IQ */
                       RNODE_LORA_MAX_PKT,
                       true,  /* Explicit variable-length LoRa header */
                       true   /* Hardware CRC16 enabled */);
    lr2021_set_lora_syncword(0x12U); /* Private LoRa syncword (0x12 / 0x1424) */
    lr2021_set_tx_power_dbm(g_rnode_cfg.txpower_dbm, is_subghz);
    lr2021_clear_irq();
    hal_radio_irq_enable(true);
    lr2021_rx_nb();
}

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
        g_domain = g_2g4_domain;
    } else if (!rate_is_2g4 && (g_domain == ELRS_DOMAIN_ISM2G4 || g_domain == ELRS_DOMAIN_CE2G4)) {
        g_domain = g_subghz_domain;
    }

    elrs_ota_set_mode(&g_ota, rate->payload_len, g_ota.switch_mode);
    elrs_fhss_init(&g_fhss, g_domain, g_ota.uid_seed);
    g_tlm_denom = (g_app_mode == APP_MODE_ELRS_TX || g_app_mode == APP_MODE_ELRS_RX)
                      ? 4U
                      : rate->tlm_denom_default;

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
        bool use_fec     = (rate->payload_len == ELRS_OTA4_PACKET_SIZE);
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
            uint8_t tlm_ratio_field = 0U;
            for (uint8_t r = 1U; r <= 7U; r++) {
                if ((uint8_t)(1U << (8U - r)) == g_tlm_denom) {
                    tlm_ratio_field = r;
                    break;
                }
            }
            elrs_sync_data_t sync = {
                .fhss_index      = (uint8_t)g_fhss.hop_ptr,
                .nonce           = g_ota.ota_nonce,
                .rf_rate_enum    = (uint8_t)rate->rate_enum,
                .switch_enc_mode = (uint8_t)g_ota.switch_mode,
                .new_tlm_ratio   = tlm_ratio_field,
                .gemini_mode     = 0U,
                .ota_protocol    = 0U,
                .uid4            = g_ota.uid[4],
                .uid5            = g_ota.uid[5],
            };
            elrs_ota_pack_sync(&g_ota, &sync, pkt);
            g_last_sync_tx_ms = now_ms;
        } else if ((g_ota.ota_nonce & 1U) != 0U && elrs_tunnel_has_pending_tx(&g_elrs_tunnel)) {
            (void)elrs_tunnel_next_tx_frame(&g_elrs_tunnel, &g_ota, pkt);
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

        /* Check lock timeout: in LOCK_TENTATIVE, require lock within 12 slots (12 ms) */
        uint32_t now_ms = hal_millis();
        bool drop_lock = false;
        if (g_lock_state == LOCK_TENTATIVE) {
            g_tentative_slots++;
            if (g_tentative_slots > 12U) {
                drop_lock = true;
            }
        } else if ((now_ms - g_last_valid_pkt_ms) > 3000U) {
            drop_lock = true;
        }
        if (drop_lock) {
            g_lock_state          = LOCK_DISCONNECTED;
            g_rf_Consec_good      = 0U;
            g_tentative_slots     = 0U;
            g_tentative_hops      = 0U;
            g_tentative_from_pair = false;
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
        bool did_hop = ((g_ota.ota_nonce % rate->fhss_hop_interval) == 0U);
        if (did_hop) {
            g_tentative_hops++;
            if (g_lock_state == LOCK_TENTATIVE && g_tentative_from_pair && g_rf_Consec_good == 1U) {
                /* Step through each sync_channel sub-sequence (0, 80, 160) on consecutive hops */
                uint8_t num_sync = (uint8_t)(g_fhss.sequence_count / g_fhss.info->freq_count);
                uint8_t cand = (uint8_t)(g_tentative_hops - 1U);
                if (cand < num_sync) {
                    uint8_t try_idx = (uint8_t)(cand * g_fhss.info->freq_count + g_tentative_hops);
                    elrs_fhss_set_index(&g_fhss, try_idx);
                    freq = elrs_fhss_get_curr_freq(&g_fhss);
                } else {
                    g_lock_state          = LOCK_DISCONNECTED;
                    g_rf_Consec_good      = 0U;
                    g_tentative_slots     = 0U;
                    g_tentative_hops      = 0U;
                    g_tentative_from_pair = false;
                    elrs_fhss_set_index(&g_fhss, 0U);
                    lr2021_set_frequency(elrs_fhss_get_initial_freq(&g_fhss), true);
                    return;
                }
            } else {
                freq = elrs_fhss_advance_and_get_freq(&g_fhss);
            }
            g_fhss_hop_count++;
        }

        /* In Active RX mode, transmit DATA tunnel frame or LinkStats on telemetry slot */
        if (g_app_mode == APP_MODE_ELRS_RX &&
            g_lock_state == LOCK_CONNECTED &&
            is_telemetry_slot(g_ota.ota_nonce, g_tlm_denom)) {
            if (did_hop) {
                lr2021_set_frequency(freq, false);
            }
            uint8_t pkt[ELRS_OTA8_PACKET_SIZE];
            if (elrs_tunnel_has_pending_tx(&g_elrs_tunnel)) {
                (void)elrs_tunnel_next_tx_frame(&g_elrs_tunnel, &g_ota, pkt);
            } else {
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
                elrs_ota_pack_linkstats(&g_ota, &ls, 0U, false, pkt);
            }
            lr2021_tx_nb(pkt, rate->payload_len);
        } else if (did_hop) {
            lr2021_set_frequency(freq, true);
        }
    }
}

static void radio_tx_done_isr(void) {
    if (g_app_mode == APP_MODE_RNODE) {
        g_rnode_tx_busy        = false;
        g_rnode_tx_done_notify = true;
        if (g_rnode_cfg.radio_online) {
            lr2021_rx_nb();
        }
        return;
    }
    if (g_app_mode == APP_MODE_ELRS_RX || g_app_mode == APP_MODE_PONG ||
        g_app_mode == APP_MODE_BRIDGE  || g_app_mode == APP_MODE_RELAY) {
        lr2021_rx_nb();
    }
}

static void radio_rx_done_isr(const uint8_t *payload, uint8_t len, const lr2021_pkt_stats_t *stats) {
    uint32_t irq_us  = hal_micros();
    g_last_rx_irq_us = irq_us;
    g_last_rssi_dbm  = stats->rssi_dbm;
    g_last_snr_q2    = stats->snr_q2_db;

    if (g_app_mode == APP_MODE_RNODE || g_app_mode == APP_MODE_RELAY) {
        if (len > 0U && !g_rns_rx_flag) {
            memcpy(g_rns_rx_buf, payload, len);
            g_rns_rx_len   = len;
            g_rns_rx_stats = *stats;
            g_rns_rx_flag  = true;
            g_rnode_cfg.rx_bytes += len;
        }
        lr2021_rx_nb();
        return;
    }

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
        /* Downlink telemetry or DATA tunnel frame received during telemetry slot */
        uint8_t dl_type = payload[0] & 0x03U;
        elrs_link_stats_data_t ls;
        uint8_t pkg_idx = 0U;
        bool ack = false;
        if (dl_type == ELRS_PKT_TYPE_LINKSTATS &&
            elrs_ota_unpack_linkstats(&g_ota, payload, &ls, &pkg_idx, &ack)) {
            g_crc_ok_count++;
            g_tlm_rx_count++;
            g_downlink_rssi = ls.uplink_rssi_1;
            g_downlink_lq   = ls.lq;
            g_downlink_snr  = ls.snr_scaled / 4;
            hal_led_toggle();
        } else if (dl_type == ELRS_PKT_TYPE_DATA &&
                   elrs_ota_validate_crc(&g_ota, payload, g_ota.ota_nonce)) {
            g_crc_ok_count++;
            g_tlm_rx_count++;
            uint16_t rx_msg_len = 0U;
            if (elrs_tunnel_process_rx_frame(&g_elrs_tunnel, payload, g_ota.is_full_res,
                                             g_tunnel_rx_buf, &rx_msg_len)) {
                g_tunnel_rx_len   = rx_msg_len;
                g_tunnel_rx_ready = true;
            }
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
            (sync.uid5 & ~0x3FU) == (g_ota.uid[5] & ~0x3FU)) {
            g_crc_ok_count++;
            g_sync_rx_count++;
            g_last_valid_pkt_ms = hal_millis();

            bool need_reanchor = (g_lock_state == LOCK_DISCONNECTED) ||
                                 (g_ota.ota_nonce != sync.nonce) ||
                                 (g_fhss.hop_ptr != sync.fhss_index);

            g_ota.ota_nonce   = sync.nonce;
            g_ota.switch_mode = (elrs_switch_mode_t)sync.switch_enc_mode;
            g_tlm_denom       = (sync.new_tlm_ratio == 0U)
                                    ? 1U
                                    : (uint8_t)(1U << (8U - sync.new_tlm_ratio));
            elrs_fhss_set_index(&g_fhss, sync.fhss_index);

            if (g_lock_state == LOCK_DISCONNECTED) {
                g_lock_state          = LOCK_TENTATIVE;
                g_rf_Consec_good      = 1U;
                g_tentative_slots     = 0U;
                g_tentative_hops      = 0U;
                g_tentative_from_pair = false;
            } else if (g_lock_state == LOCK_TENTATIVE) {
                g_rf_Consec_good++;
                if (g_rf_Consec_good >= 3U) {
                    g_lock_state = LOCK_CONNECTED;
                    hal_led_set(true);
                }
            }

            if (need_reanchor) {
                /* Schedule TOCK 150 us after SYNC RX_DONE so OtaNonce++ and FHSS hop
                 * occur right after SYNC reception, before the next TX packet starts.
                 */
                hal_hwtimer_anchor_tock_in_us(150U);
                g_last_phase_err_us = 0;
            } else {
                int32_t target_offset_us = (int32_t)rate->interval_us - 150;
                int32_t actual_offset_us = (int32_t)(irq_us - g_last_tock_us);
                int32_t err_us           = actual_offset_us - target_offset_us;
                g_last_phase_err_us      = err_us;
                if (err_us > -400 && err_us < 400) {
                    hal_hwtimer_phase_shift(err_us / 4);
                }
            }

            if (g_app_mode == APP_MODE_MONITOR && g_monitor_verbose) {
                push_mon_event(curr_freq, g_ota.ota_nonce, (uint8_t)g_fhss.hop_ptr,
                               pkt_type, true, stats, payload, len);
            }
            return;
        }
    }

    /* 2. Fast sync-channel consecutive-nonce pair lock when TX is already connected */
    if (g_lock_state == LOCK_DISCONNECTED && rate->fhss_hop_interval >= 2U) {
        uint32_t dt_us = irq_us - g_prev_sync_ch_us;
        uint32_t target_dt = rate->interval_us;
        if (g_prev_sync_ch_us != 0U &&
            dt_us >= (target_dt - 250U) &&
            dt_us <= (target_dt + 250U)) {
            for (uint16_t n = 0U; n < 256U; n++) {
                uint8_t prev_n = (uint8_t)(n - 1U);
                if ((prev_n % rate->fhss_hop_interval) == 0U &&
                    elrs_ota_validate_crc(&g_ota, payload, (uint8_t)n) &&
                    elrs_ota_validate_crc(&g_ota, g_prev_sync_ch_pkt, prev_n)) {
                    g_ota.ota_nonce       = (uint8_t)n;
                    g_ota.switch_mode     = ELRS_SW_HYBRID_OR_16CH;
                    if (g_app_mode == APP_MODE_MONITOR &&
                        ((g_prev_sync_ch_pkt[0] & 0x03U) == ELRS_PKT_TYPE_DATA ||
                         ((g_prev_sync_ch_pkt[1] & 0x7FU) == 0x00U &&
                          g_prev_sync_ch_pkt[5] == 0x00U &&
                          g_prev_sync_ch_pkt[6] == 0x00U))) {
                        g_tlm_denom = 2U;
                    }
                    elrs_fhss_set_index(&g_fhss, 0U);
                    g_lock_state          = LOCK_TENTATIVE;
                    g_rf_Consec_good      = 1U;
                    g_tentative_slots     = 0U;
                    g_tentative_hops      = 0U;
                    g_tentative_from_pair = true;
                    g_last_valid_pkt_ms   = hal_millis();
                    g_crc_ok_count++;
                    hal_hwtimer_anchor_tock_in_us(150U);
                    g_prev_sync_ch_us     = 0U;
                    return;
                }
            }
        }
        g_prev_sync_ch_us = irq_us;
        if (len <= sizeof(g_prev_sync_ch_pkt)) {
            memcpy(g_prev_sync_ch_pkt, payload, len);
        }
    }

    /* 3. If locked/tentative, validate with current OtaNonce */
    if (g_lock_state != LOCK_DISCONNECTED) {
        bool valid = false;
        if (g_app_mode == APP_MODE_MONITOR && !g_ota.is_full_res && (g_ota.ota_nonce & 1U) == 0U &&
            pkt_type == ELRS_PKT_TYPE_LINKSTATS &&
            (payload[1] & 0x7FU) == 0x00U && payload[5] == 0x00U && payload[6] == 0x00U &&
            elrs_ota_validate_crc(&g_ota, payload, g_ota.ota_nonce)) {
            g_tlm_denom = 2U;
        }
        bool is_tlm = is_telemetry_slot(g_ota.ota_nonce, g_tlm_denom);
        if (is_tlm) {
            /* Sniffing RX downlink telemetry slot in passive monitor or tentative RX mode */
            elrs_link_stats_data_t ls;
            uint8_t pkg_idx = 0U;
            bool ack = false;
            if (pkt_type == ELRS_PKT_TYPE_LINKSTATS &&
                elrs_ota_unpack_linkstats(&g_ota, payload, &ls, &pkg_idx, &ack)) {
                valid = true;
                g_tlm_rx_count++;
                g_downlink_rssi = ls.uplink_rssi_1;
                g_downlink_lq   = ls.lq;
                g_downlink_snr  = ls.snr_scaled / 4;
            } else if (pkt_type == ELRS_PKT_TYPE_DATA &&
                       elrs_ota_validate_crc(&g_ota, payload, g_ota.ota_nonce)) {
                valid = true;
                g_tlm_rx_count++;
                uint16_t rx_msg_len = 0U;
                if (elrs_tunnel_process_rx_frame(&g_elrs_tunnel, payload, g_ota.is_full_res,
                                                 g_tunnel_rx_buf, &rx_msg_len)) {
                    g_tunnel_rx_len   = rx_msg_len;
                    g_tunnel_rx_ready = true;
                }
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
            if (pkt_type == ELRS_PKT_TYPE_DATA) {
                uint16_t rx_msg_len = 0U;
                if (elrs_tunnel_process_rx_frame(&g_elrs_tunnel, payload, g_ota.is_full_res,
                                                 g_tunnel_rx_buf, &rx_msg_len)) {
                    g_tunnel_rx_len   = rx_msg_len;
                    g_tunnel_rx_ready = true;
                }
            }
        }

        if (valid) {
            g_crc_ok_count++;
            g_last_valid_pkt_ms = hal_millis();
            g_got_pkt_this_slot = true;
            if (g_lock_state == LOCK_TENTATIVE) {
                g_rf_Consec_good++;
                if (g_rf_Consec_good >= 4U) {
                    g_lock_state = LOCK_CONNECTED;
                    hal_led_set(true);
                }
            }

            /* Continuous phase tracking on uplink packets (keep Tock 150 us after RX_DONE) */
            if (!is_tlm) {
                int32_t target_offset_us = (int32_t)rate->interval_us - 150;
                int32_t actual_offset_us = (int32_t)(irq_us - g_last_tock_us);
                int32_t err_us           = actual_offset_us - target_offset_us;
                g_last_phase_err_us      = err_us;
                if (err_us > -400 && err_us < 400) {
                    hal_hwtimer_phase_shift(err_us / 8);
                }
            }

            /* In Active RX mode, schedule CRSF RC frame output on main loop so ISR never blocks */
            if (g_app_mode == APP_MODE_ELRS_RX && !is_tlm && pkt_type == ELRS_PKT_TYPE_RCDATA) {
                g_crsf_rc_pending = true;
            }

            if (g_app_mode == APP_MODE_MONITOR && g_monitor_verbose) {
                static uint32_t last_rc_mon_ms  = 0U;
                static uint32_t last_tlm_mon_ms = 0U;
                uint32_t now_ms = hal_millis();
                bool do_log = false;
                if (is_tlm) {
                    if ((now_ms - last_tlm_mon_ms) >= 100U) {
                        last_tlm_mon_ms = now_ms;
                        do_log = true;
                    }
                } else if (pkt_type == ELRS_PKT_TYPE_RCDATA) {
                    if ((now_ms - last_rc_mon_ms) >= 100U) {
                        last_rc_mon_ms = now_ms;
                        do_log = true;
                    }
                } else {
                    do_log = true;
                }
                if (do_log) {
                    uint8_t log_type = is_tlm ? 0xFEU : pkt_type;
                    push_mon_event(curr_freq, g_ota.ota_nonce, (uint8_t)g_fhss.hop_ptr,
                                   log_type, true, stats, payload, len);
                }
            }
            return;
        }
    }

    g_crc_fail_count++;
    if (g_app_mode == APP_MODE_MONITOR && g_monitor_verbose && g_lock_state == LOCK_DISCONNECTED) {
        if ((rate->phy_type != ELRS_PHY_FSK_2G4 && rate->phy_type != ELRS_PHY_FSK_900) ||
            stats->rssi_dbm > -75) {
            push_mon_event(curr_freq, payload[2], payload[1], pkt_type, false, stats, payload, len);
        }
    }
    if (g_lock_state == LOCK_DISCONNECTED) {
        lr2021_rx_nb();
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
        case APP_MODE_RNODE:   return "RNODE";
        case APP_MODE_RELAY:   return "RELAY";
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

static void rnode_send_frame(uint8_t cmd, const uint8_t *data, size_t len) {
    uint8_t out[RNODE_KISS_BUF_SIZE];
    size_t n = rnode_kiss_build_frame(out, sizeof(out), cmd, data, len);
    if (n > 0U) {
        hal_usb_uart_write(out, n);
    }
}

static void rnode_send_u8(uint8_t cmd, uint8_t val) {
    rnode_send_frame(cmd, &val, 1U);
}

static void rnode_send_u32(uint8_t cmd, uint32_t val) {
    uint8_t be[4] = {
        (uint8_t)(val >> 24U),
        (uint8_t)(val >> 16U),
        (uint8_t)(val >> 8U),
        (uint8_t)(val & 0xFFU),
    };
    rnode_send_frame(cmd, be, 4U);
}

static void rnode_send_phyprm(void) {
    uint8_t phy[12];
    uint16_t slot_ms = 0U, difs_ms = 0U;
    rnode_build_phyprm(g_rnode_cfg.bandwidth_hz, g_rnode_cfg.sf, 18U, &slot_ms, &difs_ms, phy);
    rnode_send_frame(RNODE_CMD_STAT_PHYPRM, phy, sizeof(phy));
}

static void start_mode(app_mode_t new_mode);

static void on_rnode_kiss_frame(uint8_t cmd, const uint8_t *data, size_t len, void *user_ctx) {
    (void)user_ctx;
    switch (cmd) {
        case RNODE_CMD_DETECT:
            if (len >= 1U && data[0] == RNODE_DETECT_REQ) {
                rnode_send_u8(RNODE_CMD_DETECT, RNODE_DETECT_RESP);
            }
            break;

        case RNODE_CMD_FW_VERSION: {
            uint8_t ver[2] = {RNODE_FW_MAJ, RNODE_FW_MIN};
            rnode_send_frame(RNODE_CMD_FW_VERSION, ver, 2U);
            break;
        }

        case RNODE_CMD_PLATFORM:
            rnode_send_u8(RNODE_CMD_PLATFORM, RNODE_PLATFORM_HMBRW);
            break;

        case RNODE_CMD_MCU:
            rnode_send_u8(RNODE_CMD_MCU, RNODE_MCU_NRF54);
            break;

        case RNODE_CMD_BOARD:
            rnode_send_u8(RNODE_CMD_BOARD, RNODE_BOARD_LR2021);
            break;

        case RNODE_CMD_FREQUENCY:
            if (len >= 4U) {
                g_rnode_cfg.frequency_hz = ((uint32_t)data[0] << 24U) |
                                           ((uint32_t)data[1] << 16U) |
                                           ((uint32_t)data[2] << 8U)  |
                                           (uint32_t)data[3];
                if (g_rnode_cfg.radio_online) {
                    configure_radio_for_rnode();
                }
                rnode_send_u32(RNODE_CMD_FREQUENCY, g_rnode_cfg.frequency_hz);
            }
            break;

        case RNODE_CMD_BANDWIDTH:
            if (len >= 4U) {
                g_rnode_cfg.bandwidth_hz = ((uint32_t)data[0] << 24U) |
                                           ((uint32_t)data[1] << 16U) |
                                           ((uint32_t)data[2] << 8U)  |
                                           (uint32_t)data[3];
                if (g_rnode_cfg.radio_online) {
                    configure_radio_for_rnode();
                }
                rnode_send_u32(RNODE_CMD_BANDWIDTH, g_rnode_cfg.bandwidth_hz);
            }
            break;

        case RNODE_CMD_TXPOWER:
            if (len >= 1U) {
                g_rnode_cfg.txpower_dbm = (int8_t)data[0];
                if (g_rnode_cfg.radio_online) {
                    lr2021_set_tx_power_dbm(g_rnode_cfg.txpower_dbm, g_rnode_cfg.frequency_hz < 1000000000UL);
                }
                rnode_send_u8(RNODE_CMD_TXPOWER, (uint8_t)g_rnode_cfg.txpower_dbm);
            }
            break;

        case RNODE_CMD_SF:
            if (len >= 1U) {
                g_rnode_cfg.sf = data[0];
                if (g_rnode_cfg.radio_online) {
                    configure_radio_for_rnode();
                }
                rnode_send_u8(RNODE_CMD_SF, g_rnode_cfg.sf);
            }
            break;

        case RNODE_CMD_CR:
            if (len >= 1U) {
                g_rnode_cfg.cr = data[0];
                if (g_rnode_cfg.radio_online) {
                    configure_radio_for_rnode();
                }
                rnode_send_u8(RNODE_CMD_CR, g_rnode_cfg.cr);
            }
            break;

        case RNODE_CMD_ST_ALOCK:
            if (len >= 2U) {
                g_rnode_cfg.st_alock = (uint16_t)(((uint16_t)data[0] << 8U) | data[1]);
                rnode_send_frame(RNODE_CMD_ST_ALOCK, data, 2U);
            }
            break;

        case RNODE_CMD_LT_ALOCK:
            if (len >= 2U) {
                g_rnode_cfg.lt_alock = (uint16_t)(((uint16_t)data[0] << 8U) | data[1]);
                rnode_send_frame(RNODE_CMD_LT_ALOCK, data, 2U);
            }
            break;

        case RNODE_CMD_RADIO_STATE:
            if (len >= 1U) {
                if (data[0] == 0xFFU) {
                    rnode_send_u8(RNODE_CMD_RADIO_STATE, g_rnode_cfg.radio_online ? 1U : 0U);
                } else {
                    g_rnode_cfg.radio_online = (data[0] == RNODE_RADIO_STATE_ON);
                    if (g_rnode_cfg.radio_online) {
                        configure_radio_for_rnode();
                        rnode_send_phyprm();
                        hal_led_set(true);
                    } else {
                        lr2021_set_mode(LR2021_MODE_STDBY_XOSC);
                        hal_led_set(false);
                    }
                    rnode_send_u8(RNODE_CMD_RADIO_STATE, g_rnode_cfg.radio_online ? 1U : 0U);
                }
            }
            break;

        case RNODE_CMD_STAT_RX:
            rnode_send_u32(RNODE_CMD_STAT_RX, g_rnode_cfg.rx_bytes);
            break;

        case RNODE_CMD_STAT_TX:
            rnode_send_u32(RNODE_CMD_STAT_TX, g_rnode_cfg.tx_bytes);
            break;

        case RNODE_CMD_STAT_PHYPRM:
            rnode_send_phyprm();
            break;

        case RNODE_CMD_STAT_CHTM: {
            uint8_t chtm[11] = {0};
            int rssi_val = (int)g_last_rssi_dbm + RNODE_RSSI_OFFSET;
            if (rssi_val < 0) rssi_val = 0;
            if (rssi_val > 255) rssi_val = 255;
            chtm[8]  = (uint8_t)rssi_val;
            chtm[9]  = (uint8_t)(157 - 115); /* -115 dBm noise floor */
            chtm[10] = 0xFFU;
            rnode_send_frame(RNODE_CMD_STAT_CHTM, chtm, sizeof(chtm));
            break;
        }

        case RNODE_CMD_RANDOM:
            rnode_send_u8(RNODE_CMD_RANDOM, (uint8_t)((hal_micros() ^ (uint32_t)g_last_rssi_dbm) & 0xFFU));
            break;

        case RNODE_CMD_BLINK:
            hal_led_toggle();
            break;

        case RNODE_CMD_DATA:
            if (len > 0U && len <= RNODE_LORA_MAX_PKT && g_rnode_cfg.radio_online) {
                g_rnode_tx_busy = true;
                g_rnode_cfg.tx_bytes += (uint32_t)len;
                lr2021_tx_nb(data, (uint8_t)len);
            } else if (len > RNODE_LORA_MAX_PKT) {
                rnode_send_u8(RNODE_CMD_ERROR, RNODE_ERR_TXFAILED);
            }
            break;

        case RNODE_CMD_LEAVE:
            g_rnode_cfg.radio_online = false;
            start_mode(APP_MODE_IDLE);
            break;

        default:
            break;
    }
}

static void start_mode(app_mode_t new_mode) {
    hal_hwtimer_stop();
    hal_radio_irq_enable(false);
    lr2021_set_mode(LR2021_MODE_STDBY_XOSC);
    lr2021_clear_irq();
    g_app_mode            = new_mode;
    g_lock_state          = LOCK_DISCONNECTED;
    g_rf_Consec_good      = 0U;
    g_tentative_slots     = 0U;
    g_tentative_hops      = 0U;
    g_tentative_from_pair = false;
    g_prev_sync_ch_us     = 0U;
    g_rns_rx_flag         = false;
    g_relay_pending       = false;
    hal_led_set(false);

    if (new_mode == APP_MODE_IDLE) {
        hal_usb_uart_printf("[MODE] Switched to IDLE (STDBY_XOSC)\r\n");
        return;
    }

    lr2021_reset_counters();
    g_crc_ok_count      = 0U;
    g_crc_fail_count    = 0U;
    g_sync_rx_count     = 0U;
    g_rc_rx_count       = 0U;
    g_tlm_rx_count      = 0U;
    g_fhss_hop_count    = 0U;
    g_last_phase_err_us = 0;

    if (new_mode == APP_MODE_RNODE) {
        /* In RNODE mode, configure radio immediately with current g_rnode_cfg so it's ready */
        g_rnode_cfg.radio_online = true;
        configure_radio_for_rnode();
        hal_led_set(true);
        return;
    }

    if (new_mode == APP_MODE_RELAY) {
        g_rnode_cfg.radio_online = true;
        configure_radio_for_rnode();
        hal_led_set(true);
        hal_usb_uart_printf("[MODE] Started RELAY (Reticulum 868/2.4G repeater + FC MAVLink bridge on %lu Hz BW=%lu SF=%u CR=4/%u)\r\n",
                            (unsigned long)g_rnode_cfg.frequency_hz,
                            (unsigned long)g_rnode_cfg.bandwidth_hz,
                            g_rnode_cfg.sf,
                            g_rnode_cfg.cr);
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
        lr2021_clear_irq();
        hal_radio_irq_enable(true);
        lr2021_rx_nb();
        hal_usb_uart_printf("[MODE] Switched to BRIDGE (FLRC %lu Hz, %uB payload, RX continuous)\r\n",
                            (unsigned long)g_bridge_freq_hz, g_bridge_pkt_len);
        return;
    }

    configure_radio_for_current_rate();
    elrs_tunnel_init(&g_elrs_tunnel, 2U);
    const elrs_air_rate_config_t *rate = elrs_get_air_rate_by_index(g_rate_idx);

    if (new_mode == APP_MODE_ELRS_TX) {
        g_lock_state      = LOCK_CONNECTED;
        g_last_sync_tx_ms = 0U;
        lr2021_clear_irq();
        hal_radio_irq_enable(true);
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
        lr2021_clear_irq();
        hal_radio_irq_enable(true);
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
    uint16_t ver = g_chip_version;
    uint16_t err = 0U;
    if (g_app_mode == APP_MODE_IDLE) {
        ver = lr2021_get_version();
        err = lr2021_get_errors();
    }

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
    hal_usb_uart_printf("TLM_LINK uplink_rssi=-%udBm lq=%u%% snr=%ddB\r\n",
                        (unsigned)g_downlink_rssi,
                        (unsigned)g_downlink_lq,
                        (int)g_downlink_snr);
    hal_usb_uart_printf(
        "RNS rnode_freq=%lu bw=%lu sf=%u cr=4/%u online=%u relay_fwd=%lu relay_drop=%lu "
        "tun_tx=%lu tun_rx=%lu fc_pkts=%lu fc_drop=%lu\r\n",
        (unsigned long)g_rnode_cfg.frequency_hz,
        (unsigned long)g_rnode_cfg.bandwidth_hz,
        g_rnode_cfg.sf,
        g_rnode_cfg.cr,
        g_rnode_cfg.radio_online ? 1U : 0U,
        (unsigned long)g_rns_relay.relayed_count,
        (unsigned long)g_rns_relay.dropped_dup_count,
        (unsigned long)g_elrs_tunnel.tx_msgs_sent,
        (unsigned long)g_elrs_tunnel.rx_msgs_completed,
        (unsigned long)g_fc_tlm.rx_mav_pkts,
        (unsigned long)g_fc_tlm.rx_mav_Drop
    );
}

static bool send_or_tunnel_rns_packet(const uint8_t *pkt, uint16_t len) {
    if (pkt == NULL || len == 0U) {
        return false;
    }
    if (g_app_mode == APP_MODE_ELRS_TX || g_app_mode == APP_MODE_ELRS_RX) {
        return elrs_tunnel_enqueue_tx(&g_elrs_tunnel, pkt, len, g_ota.is_full_res);
    }
    if (g_app_mode == APP_MODE_RELAY || g_app_mode == APP_MODE_RNODE) {
        if (len > RNODE_LORA_MAX_PKT) {
            return false;
        }
        /* Record packet hash in relay table so we never re-relay our own transmission */
        rns_relay_record_sent(&g_rns_relay, pkt, len, hal_millis());
        g_rnode_cfg.tx_bytes += len;
        lr2021_tx_nb(pkt, (uint8_t)len);
        return true;
    }
    return false;
}

static void inspect_and_act_on_rns_packet(const uint8_t *pkt, uint16_t len, const char *src_tag) {
    if (pkt == NULL || len <= RNS_MIN_PACKET_LEN) {
        return;
    }
    bool ifac = (pkt[0] & 0x80U) != 0U;
    uint8_t hops = pkt[1];

    /* 1. Check if addressed to "rnlog.beacon" */
    if (!ifac && memcmp(&pkt[2], RNS_DEST_RNLOG_BEACON, 16U) == 0) {
        uint16_t pay_len = (uint16_t)(len - RNS_MIN_PACKET_LEN);
        hal_usb_uart_printf("RNS_BCN_RX src=%s hops=%u len=%u json=%.*s\r\n",
                            src_tag, hops, len, (int)pay_len, (const char *)&pkt[RNS_MIN_PACKET_LEN]);
        return;
    }

    /* 2. Check if addressed to "elrs.drone.cmd" */
    rns_drone_cmd_t dcmd;
    if (!ifac && rns_parse_drone_command_pkt(pkt, len, &dcmd)) {
        uint16_t pay_len = (uint16_t)(len - RNS_MIN_PACKET_LEN);
        if (dcmd.type == RNS_DRONE_CMD_RC_OVERRIDE) {
            for (int i = 0; i < 4; i++) {
                g_rc_channels[i] = crsf_us_to_ticks(dcmd.rc_us[i]);
            }
        }
        uint8_t mav_frame[64];
        size_t mav_len = rns_build_mavlink_command(&dcmd, g_mav_tx_seq++, mav_frame, sizeof(mav_frame));
        if (mav_len > 0U) {
            hal_crsf_uart_write(mav_frame, mav_len);
        }
        hal_usb_uart_printf("RNS_CMD_RX src=%s hops=%u cmd=\"%.*s\" type=%u mav_len=%u mav_hex=",
                            src_tag, hops, (int)pay_len, (const char *)&pkt[RNS_MIN_PACKET_LEN],
                            (unsigned)dcmd.type, (unsigned)mav_len);
        for (size_t i = 0U; i < mav_len; i++) {
            hal_usb_uart_printf("%02X", mav_frame[i]);
        }
        hal_usb_uart_puts("\r\n");
    }
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
            "  status                         - Show radio, FHSS, RNode/Relay, and RC/MAVLink status\r\n"
            "  rates                          - List supported ExpressLRS air rates (Sub-GHz & 2.4 GHz)\r\n"
            "  phrase <binding_phrase>        - Set UID from ExpressLRS binding phrase (MD5)\r\n"
            "  uid <XX:XX:XX:XX:XX:XX>        - Set 6-byte ExpressLRS UID directly\r\n"
            "  domain <fcc915|au915|eu868|ism2g4> - Set regulatory frequency domain\r\n"
            "  rate <idx_or_name>             - Select ExpressLRS air rate\r\n"
            "  power <dbm>                    - Set TX output power in dBm (-20..22)\r\n"
            "  mode <idle|tx|rx|monitor|bridge|rnode|relay> - Switch operating mode\r\n"
            "  rnode_cfg [freq] [bw] [sf] [cr]- Set RNode/Relay LoRa config (e.g. 868000000 125000 7 5)\r\n"
            "  bcn [now | <interval_ms>]      - Send Reticulum rnlog.beacon now or set periodic ms\r\n"
            "  rns_cmd <GOTO lat lon alt|RTL|LAND|ARM|DISARM|MODE n|RC c1 c2 c3 c4> - Send RNS drone cmd\r\n"
            "  rns_tx <hex_bytes>             - Send raw Reticulum packet over Relay LoRa or ELRS tunnel\r\n"
            "  fc_sim <lat_e7> <lon_e7> <alt_m> <vbatt_mv> - Set simulated FC telemetry for beacons\r\n"
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
        if (strcasecmp(arg, "fcc915") == 0) { g_domain = ELRS_DOMAIN_FCC915; g_subghz_domain = g_domain; }
        else if (strcasecmp(arg, "au915") == 0) { g_domain = ELRS_DOMAIN_AU915; g_subghz_domain = g_domain; }
        else if (strcasecmp(arg, "eu868") == 0) { g_domain = ELRS_DOMAIN_EU868; g_subghz_domain = g_domain; }
        else if (strcasecmp(arg, "ism2g4") == 0 || strcasecmp(arg, "ism2400") == 0) { g_domain = ELRS_DOMAIN_ISM2G4; g_2g4_domain = g_domain; }
        else if (strcasecmp(arg, "ce2g4") == 0 || strcasecmp(arg, "ce_lbt") == 0) { g_domain = ELRS_DOMAIN_CE2G4; g_2g4_domain = g_domain; }
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
            hal_usb_uart_puts("ERR: usage: mode <idle|tx|rx|monitor|bridge|rnode|relay>\r\n");
            return;
        }
        if (strcasecmp(arg, "idle") == 0) start_mode(APP_MODE_IDLE);
        else if (strcasecmp(arg, "tx") == 0) start_mode(APP_MODE_ELRS_TX);
        else if (strcasecmp(arg, "rx") == 0) start_mode(APP_MODE_ELRS_RX);
        else if (strcasecmp(arg, "monitor") == 0) start_mode(APP_MODE_MONITOR);
        else if (strcasecmp(arg, "bridge") == 0) start_mode(APP_MODE_BRIDGE);
        else if (strcasecmp(arg, "rnode") == 0) start_mode(APP_MODE_RNODE);
        else if (strcasecmp(arg, "relay") == 0) start_mode(APP_MODE_RELAY);
        else hal_usb_uart_printf("ERR: unknown mode %s\r\n", arg);
        return;
    }

    if (strcmp(cmd, "rnode_cfg") == 0) {
        char *f_arg  = strtok(NULL, " ");
        char *bw_arg = strtok(NULL, " ");
        char *sf_arg = strtok(NULL, " ");
        char *cr_arg = strtok(NULL, " ");
        if (f_arg)  g_rnode_cfg.frequency_hz = (uint32_t)strtoul(f_arg, NULL, 10);
        if (bw_arg) g_rnode_cfg.bandwidth_hz = (uint32_t)strtoul(bw_arg, NULL, 10);
        if (sf_arg) g_rnode_cfg.sf           = (uint8_t)strtoul(sf_arg, NULL, 10);
        if (cr_arg) g_rnode_cfg.cr           = (uint8_t)strtoul(cr_arg, NULL, 10);
        if (g_app_mode == APP_MODE_RNODE || g_app_mode == APP_MODE_RELAY) {
            configure_radio_for_rnode();
        }
        hal_usb_uart_printf("OK rnode_cfg freq=%lu bw=%lu sf=%u cr=4/%u pwr=%ddBm\r\n",
                            (unsigned long)g_rnode_cfg.frequency_hz,
                            (unsigned long)g_rnode_cfg.bandwidth_hz,
                            g_rnode_cfg.sf,
                            g_rnode_cfg.cr,
                            (int)g_rnode_cfg.txpower_dbm);
        return;
    }

    if (strcmp(cmd, "fc_sim") == 0) {
        char *lat_s = strtok(NULL, " ");
        char *lon_s = strtok(NULL, " ");
        char *alt_s = strtok(NULL, " ");
        char *v_s   = strtok(NULL, " ");
        if (!lat_s || !lon_s || !alt_s || !v_s) {
            hal_usb_uart_puts("ERR: usage: fc_sim <lat_e7> <lon_e7> <alt_m> <vbatt_mv>\r\n");
            return;
        }
        g_fc_tlm.lat_deg_e7 = (int32_t)strtol(lat_s, NULL, 10);
        g_fc_tlm.lon_deg_e7 = (int32_t)strtol(lon_s, NULL, 10);
        g_fc_tlm.rel_alt_mm = (int32_t)strtol(alt_s, NULL, 10) * 1000L;
        g_fc_tlm.vbatt_mv   = (uint16_t)strtoul(v_s, NULL, 10);
        g_fc_tlm.armed      = g_rc_is_armed;
        hal_usb_uart_printf("OK fc_sim lat=%ld lon=%ld alt_mm=%ld vbatt=%umV\r\n",
                            (long)g_fc_tlm.lat_deg_e7, (long)g_fc_tlm.lon_deg_e7,
                            (long)g_fc_tlm.rel_alt_mm, (unsigned)g_fc_tlm.vbatt_mv);
        return;
    }

    if (strcmp(cmd, "bcn") == 0) {
        char *arg = strtok(NULL, " ");
        if (arg == NULL || strcasecmp(arg, "now") == 0) {
            uint8_t pkt[RNODE_LORA_MAX_PKT];
            uint8_t lq = (g_app_mode == APP_MODE_ELRS_TX || g_app_mode == APP_MODE_ELRS_RX) ? 100U : 100U;
            size_t blen = rns_build_telemetry_beacon(&g_fc_tlm, lq, g_last_rssi_dbm, pkt, sizeof(pkt));
            bool ok = send_or_tunnel_rns_packet(pkt, (uint16_t)blen);
            hal_usb_uart_printf("OK RNS_BCN_TX sent=%u len=%u json=%.*s\r\n",
                                ok ? 1U : 0U, (unsigned)blen,
                                (int)(blen - RNS_MIN_PACKET_LEN),
                                (const char *)&pkt[RNS_MIN_PACKET_LEN]);
            return;
        }
        g_beacon_interval_ms = (uint32_t)strtoul(arg, NULL, 10);
        g_last_beacon_ms     = hal_millis();
        hal_usb_uart_printf("OK bcn interval=%lu ms\r\n", (unsigned long)g_beacon_interval_ms);
        return;
    }

    if (strcmp(cmd, "rns_cmd") == 0) {
        char *rest = strtok(NULL, "");
        if (rest == NULL) {
            hal_usb_uart_puts("ERR: usage: rns_cmd <GOTO lat lon alt | RTL | LAND | ARM | DISARM | MODE n | RC c1 c2 c3 c4>\r\n");
            return;
        }
        while (*rest && isspace((unsigned char)*rest)) rest++;
        uint8_t pkt[RNODE_LORA_MAX_PKT];
        size_t clen = rns_build_drone_command_pkt(rest, pkt, sizeof(pkt));
        bool ok = send_or_tunnel_rns_packet(pkt, (uint16_t)clen);
        hal_usb_uart_printf("OK RNS_CMD_TX sent=%u len=%u cmd=\"%s\"\r\n",
                            ok ? 1U : 0U, (unsigned)clen, rest);
        return;
    }

    if (strcmp(cmd, "rns_tx") == 0) {
        char *hex = strtok(NULL, " ");
        if (hex == NULL) {
            hal_usb_uart_puts("ERR: usage: rns_tx <hex_bytes>\r\n");
            return;
        }
        uint8_t pkt[RNODE_LORA_MAX_PKT] = {0};
        size_t slen = strlen(hex);
        uint16_t blen = 0U;
        for (size_t i = 0U; i + 1U < slen && blen < sizeof(pkt); i += 2U) {
            char pair[3] = {hex[i], hex[i + 1U], '\0'};
            pkt[blen++] = (uint8_t)strtoul(pair, NULL, 16);
        }
        bool ok = send_or_tunnel_rns_packet(pkt, blen);
        hal_usb_uart_printf("OK RNS_RAW_TX sent=%u len=%u\r\n", ok ? 1U : 0U, (unsigned)blen);
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
        uint32_t freq_hz = f_arg ? (uint32_t)strtoul(f_arg, NULL, 10) : 868000000UL;
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
        uint32_t freq_hz = f_arg ? (uint32_t)strtoul(f_arg, NULL, 10) : 868000000UL;
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
        char *arg1 = strtok(NULL, " ");
        char *arg2 = strtok(NULL, " ");
        char *arg3 = strtok(NULL, " ");
        hal_hwtimer_stop();
        uint32_t f_start = 863000000UL;
        uint32_t f_stop  = 870000000UL;
        uint32_t f_step  = 250000UL;
        bool is_2g4      = false;
        bool is_900      = false;

        if (arg1 && arg2 && arg3) {
            f_start = (uint32_t)strtoul(arg1, NULL, 10);
            f_stop  = (uint32_t)strtoul(arg2, NULL, 10);
            f_step  = (uint32_t)strtoul(arg3, NULL, 10);
            if (f_step == 0U) f_step = 250000UL;
            is_2g4 = (f_start >= 1000000000UL);
            is_900 = (!is_2g4 && f_start >= 900000000UL);
        } else {
            is_2g4 = (arg1 && (strcmp(arg1, "2400") == 0 || strcasecmp(arg1, "2g4") == 0));
            is_900 = (arg1 && strcmp(arg1, "900") == 0);
            f_start = is_2g4 ? 2400000000UL : (is_900 ? 903000000UL : 863000000UL);
            f_stop  = is_2g4 ? 2480000000UL : (is_900 ? 927000000UL : 870000000UL);
            f_step  = is_2g4 ? 2000000UL    : (is_900 ? 600000UL    : 250000UL);
        }

        uint8_t bw = is_2g4 ? LR2021_RADIO_LORA_BW_800 : LR2021_RADIO_LORA_BW_500;
        lr2021_config_lora(bw, LR2021_RADIO_LORA_SF6, LR2021_RADIO_LORA_CR_4_7,
                           f_start, 12U, false, 8U, false, false);

        hal_usb_uart_printf("SCAN_START band=%s start=%lu stop=%lu step=%lu\r\n",
                            is_2g4 ? "2.4GHz" : (is_900 ? "900MHz" : "868MHz"),
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
    rnode_kiss_init(&g_kiss_parser, on_rnode_kiss_frame, NULL);
    rns_relay_init(&g_rns_relay);
    rns_mav_parser_init(&g_mav_parser);
    memset(&g_fc_tlm, 0, sizeof(g_fc_tlm));
    g_fc_tlm.vbatt_mv = 24800U; /* Default 6S nominal until MAVLink SYS_STATUS arrives */
    elrs_tunnel_init(&g_elrs_tunnel, 2U);

    /* Default binding phrase */
    uint8_t default_uid[6];
    elrs_uid_from_bind_phrase("WinniePico", default_uid);
    elrs_ota_init(&g_ota, default_uid, ELRS_OTA4_PACKET_SIZE, ELRS_SW_WIDE_OR_8CH);
    elrs_fhss_init(&g_fhss, g_domain, g_ota.uid_seed);

    lr2021_set_callbacks(radio_rx_done_isr, radio_tx_done_isr);

    uint16_t chip_ver = 0U;
    bool radio_ok = lr2021_begin(868000000UL, 2440000000UL, &chip_ver);
    g_chip_version = chip_ver;

    hal_usb_uart_printf("\r\n========================================================\r\n");
    hal_usb_uart_printf("LR2021-ELRS + Reticulum RNode/Relay Firmware (nRF54L15)\r\n");
    hal_usb_uart_printf("LR2021 Init: %s (Version=0x%04X, Errors=0x%04X)\r\n",
                        radio_ok ? "OK" : "FAIL", chip_ver, lr2021_get_errors());
    hal_usb_uart_printf("Default UID: %02X:%02X:%02X:%02X:%02X:%02X (phrase=\"WinniePico\")\r\n",
                        g_ota.uid[0], g_ota.uid[1], g_ota.uid[2],
                        g_ota.uid[3], g_ota.uid[4], g_ota.uid[5]);
    hal_usb_uart_printf("========================================================\r\n");

    char cli_buf[192];
    size_t cli_pos = 0U;
    uint32_t last_crsf_linkstats_ms = 0U;

    while (1) {
        /* 1. Process incoming USB CDC CLI or RNode KISS bytes */
        int b;
        while ((b = hal_usb_uart_read_byte()) >= 0) {
            uint8_t ub = (uint8_t)b;
            /* Auto-switch into RNODE KISS mode if a KISS FEND (0xC0) byte arrives */
            if (g_app_mode != APP_MODE_RNODE && ub == RNODE_FEND) {
                hal_hwtimer_stop();
                g_app_mode = APP_MODE_RNODE;
                cli_pos    = 0U;
            }
            if (g_app_mode == APP_MODE_RNODE) {
                rnode_kiss_feed_byte(&g_kiss_parser, ub);
                continue;
            }
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

        /* 2. Process incoming external Grove UART bytes (CRSF RC input OR FC MAVLink telemetry) */
        while ((b = hal_crsf_uart_read_byte()) >= 0) {
            uint8_t ub = (uint8_t)b;
            if (crsf_parser_feed_byte(&g_crsf_parser, ub)) {
                uint8_t ftype = g_crsf_parser.buf[2];
                uint8_t flen  = (uint8_t)(g_crsf_parser.expected_len + 2U);
                if (ftype == CRSF_FRAMETYPE_RC_CHANNELS) {
                    (void)crsf_decode_rc_channels_frame(g_crsf_parser.buf, flen, g_rc_channels);
                    g_rc_is_armed = (g_rc_channels[4] > ELRS_CRSF_CH_MID);
                }
            }
            (void)rns_mav_parse_byte(&g_mav_parser, ub, &g_fc_tlm);
        }

        /* 3. Handle RNode TX_DONE notification & incoming LoRa packets in RNODE / RELAY mode */
        if (g_app_mode == APP_MODE_RNODE && g_rnode_tx_done_notify) {
            g_rnode_tx_done_notify = false;
            rnode_send_u8(RNODE_CMD_READY, 0x01U);
        }

        uint32_t now_ms = hal_millis();
        if (g_rns_rx_flag) {
            uint8_t rx_copy[RNODE_LORA_MAX_PKT];
            uint8_t rx_len = g_rns_rx_len;
            lr2021_pkt_stats_t rx_st = g_rns_rx_stats;
            memcpy(rx_copy, g_rns_rx_buf, rx_len);
            g_rns_rx_flag = false;

            if (g_app_mode == APP_MODE_RNODE) {
                int rssi_enc = (int)rx_st.rssi_dbm + RNODE_RSSI_OFFSET;
                if (rssi_enc < 0) rssi_enc = 0;
                if (rssi_enc > 255) rssi_enc = 255;
                rnode_send_u8(RNODE_CMD_STAT_RSSI, (uint8_t)rssi_enc);
                rnode_send_u8(RNODE_CMD_STAT_SNR, (uint8_t)rx_st.snr_q2_db);
                rnode_send_frame(RNODE_CMD_DATA, rx_copy, rx_len);
                hal_led_toggle();
            } else if (g_app_mode == APP_MODE_RELAY) {
                hal_led_toggle();
                uint8_t orig_hops = (rx_len > 1U) ? rx_copy[1] : 0U;
                inspect_and_act_on_rns_packet(rx_copy, rx_len, "LORA");
                if (!g_relay_pending &&
                    rns_relay_should_forward(&g_rns_relay, rx_copy, rx_len, now_ms)) {
                    memcpy(g_relay_tx_buf, rx_copy, rx_len);
                    g_relay_tx_len  = rx_len;
                    g_relay_due_ms  = now_ms + 120U + (uint32_t)(hal_micros() & 0x7FU);
                    g_relay_pending = true;
                }
                hal_usb_uart_printf("RNS_RELAY_RX len=%u rssi=%ddBm snr=%ddB hops=%u ifac=%u fwd=%u\r\n",
                                    rx_len,
                                    (int)rx_st.rssi_dbm,
                                    (int)(rx_st.snr_q2_db / 4),
                                    orig_hops,
                                    ((rx_copy[0] & 0x80U) != 0U) ? 1U : 0U,
                                    g_relay_pending ? 1U : 0U);
            }
        }

        if (g_app_mode == APP_MODE_RELAY && g_relay_pending &&
            (int32_t)(now_ms - g_relay_due_ms) >= 0) {
            g_relay_pending = false;
            g_rnode_cfg.tx_bytes += g_relay_tx_len;
            lr2021_tx_nb(g_relay_tx_buf, g_relay_tx_len);
            hal_usb_uart_printf("RNS_RELAY_FWD len=%u hops=%u\r\n",
                                g_relay_tx_len,
                                (g_relay_tx_len > 1U) ? g_relay_tx_buf[1] : 0U);
        }

        /* 4. Handle reassembled Reticulum packet from ExpressLRS DATA tunnel */
        if (g_tunnel_rx_ready) {
            uint8_t tun_pkt[ELRS_TUNNEL_MAX_MTU];
            uint16_t tun_len = g_tunnel_rx_len;
            memcpy(tun_pkt, g_tunnel_rx_buf, tun_len);
            g_tunnel_rx_ready = false;

            hal_usb_uart_printf("ELRS_RNS_RX len=%u hops=%u hex=",
                                tun_len, (tun_len > 1U) ? tun_pkt[1] : 0U);
            uint16_t dump_max = (tun_len < 32U) ? tun_len : 32U;
            for (uint16_t i = 0U; i < dump_max; i++) {
                hal_usb_uart_printf("%02X", tun_pkt[i]);
            }
            if (tun_len > dump_max) hal_usb_uart_puts("...");
            hal_usb_uart_puts("\r\n");

            inspect_and_act_on_rns_packet(tun_pkt, tun_len, "ELRS_TUNNEL");
        }

        /* 5. Periodic Reticulum Telemetry Beacon (rnlog.beacon) */
        if (g_beacon_interval_ms > 0U && (now_ms - g_last_beacon_ms) >= g_beacon_interval_ms) {
            g_last_beacon_ms = now_ms;
            uint8_t pkt[RNODE_LORA_MAX_PKT];
            size_t blen = rns_build_telemetry_beacon(&g_fc_tlm, 100U, g_last_rssi_dbm, pkt, sizeof(pkt));
            (void)send_or_tunnel_rns_packet(pkt, (uint16_t)blen);
        }

        /* 6. In Active RX mode, forward CRSF RC channels & periodic LINK_STATISTICS (0x14) to FC */
        if (g_app_mode == APP_MODE_ELRS_RX && g_crsf_rc_pending) {
            g_crsf_rc_pending = false;
            uint8_t crsf_frame[26];
            crsf_build_rc_channels_frame(g_rc_channels, crsf_frame);
            hal_crsf_uart_write(crsf_frame, sizeof(crsf_frame));
        }
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

        /* 7. Drain at most 1 deferred MONITOR event per loop iteration so CLI never starves */
        if (g_mon_tail != g_mon_head) {
            mon_event_t ev = g_mon_ring[g_mon_tail];
            g_mon_tail = (g_mon_tail + 1U) & (MON_RING_SIZE - 1U);
            const char *tstr = (ev.pkt_type == ELRS_PKT_TYPE_SYNC) ? "SYNC" :
                               (ev.pkt_type == 0xFEU) ? "TLM" :
                               (ev.pkt_type == ELRS_PKT_TYPE_RCDATA) ? "RC" : "DATA";
            hal_usb_uart_printf(
                "MON t=%lu freq=%lu nonce=%3u hop=%3u type=%-4s crc=%u rssi=%4d snr=%2d ch=[%4u,%4u,%4u,%4u] raw=",
                (unsigned long)ev.timestamp_us,
                (unsigned long)ev.freq_hz,
                ev.nonce,
                ev.hop_idx,
                tstr,
                ev.crc_ok ? 1U : 0U,
                (int)ev.rssi_dbm,
                (int)(ev.snr_q2 / 4),
                ev.ch_us[0], ev.ch_us[1], ev.ch_us[2], ev.ch_us[3]
            );
            for (uint8_t i = 0U; i < ev.raw_len; i++) {
                hal_usb_uart_printf("%02X", ev.raw[i]);
            }
            hal_usb_uart_puts("\r\n");
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
