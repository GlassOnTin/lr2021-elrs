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
static crsf_parser_t       g_usb_crsf_parser;
static uint8_t             g_last_aux7_bin     = 0xFFU;
static uint8_t             g_last_aux8_bin     = 0xFFU;

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
static rns_node_identity_t  g_rns_id;
static rns_peer_table_t     g_rns_peers;
static uint8_t              g_rns_cmd_seq = 1U;
static uint8_t              g_swarm_slot_id      = 0U;   /* 0 = Leader, 1..7 = Wingman/Follower slots */
static uint8_t              g_form_seq           = 1U;
static bool                 g_mesh_ctrl_active   = false;
static uint32_t             g_last_mesh_ctrl_ms  = 0U;
#define RNS_MESH_WATCHDOG_MS 3000U /* 3.0s dead-man brake matching Anisoptera GUID_TIMEOUT */

static void format_hex_bytes(const uint8_t *in, size_t len, char *out) {
    static const char hex[] = "0123456789abcdef";
    for (size_t i = 0U; i < len; i++) {
        out[i * 2U]      = hex[(in[i] >> 4U) & 0x0FU];
        out[i * 2U + 1U] = hex[in[i] & 0x0FU];
    }
    out[len * 2U] = '\0';
}

static bool parse_hex_bytes(const char *str, uint8_t *out, size_t expected_bytes) {
    if (str == NULL || out == NULL) {
        return false;
    }
    size_t slen = strlen(str);
    if (slen != expected_bytes * 2U) {
        return false;
    }
    for (size_t i = 0U; i < expected_bytes; i++) {
        char pair[3] = {str[i * 2U], str[i * 2U + 1U], '\0'};
        char *endp = NULL;
        unsigned long val = strtoul(pair, &endp, 16);
        if (endp == NULL || *endp != '\0') {
            return false;
        }
        out[i] = (uint8_t)val;
    }
    return true;
}

static void refresh_rns_identity(void) {
    uint8_t hw_id[8];
    hal_get_device_id(hw_id);
    bool prev_ifac = g_rns_id.ifac_enabled;
    char prev_pass[16];
    memcpy(prev_pass, g_rns_id.ifac_pass, sizeof(prev_pass));

    uint8_t role     = RNS_NODE_ROLE_RX;
    const char *name = "LR2021-RX";
    if (g_app_mode == APP_MODE_ELRS_TX) {
        role = RNS_NODE_ROLE_TX;
        name = "LR2021-TX";
    } else if (g_app_mode == APP_MODE_RELAY) {
        role = RNS_NODE_ROLE_RELAY;
        name = "LR2021-RLY";
    } else if (g_app_mode == APP_MODE_RNODE) {
        role = RNS_NODE_ROLE_RNODE;
        name = "LR2021-RND";
    }
    rns_identity_init(&g_rns_id, hw_id, g_ota.uid, role, name);
    rns_identity_set_ifac(&g_rns_id, prev_ifac, prev_pass);
}

static volatile bool        g_rns_rx_flag = false;
static uint8_t              g_rns_rx_buf[RNODE_LORA_MAX_PKT];
static volatile uint8_t     g_rns_rx_len  = 0U;
static lr2021_pkt_stats_t   g_rns_rx_stats;
static volatile bool        g_rnode_tx_busy = false;
static volatile bool        g_rnode_tx_done_notify = false;

/* Multi-frame fragmentation/reassembly for RNode HW_MTU (up to 508B over <=255B LoRa PHY) */
static uint8_t              g_rnode_tx_full_buf[RNODE_HW_MTU];
static uint16_t             g_rnode_tx_full_len      = 0U;
static uint8_t              g_rnode_tx_frag_seq      = 0U;
static volatile uint8_t     g_rnode_tx_frag_idx      = 0U;
static volatile uint8_t     g_rnode_tx_frag_total    = 0U;
static volatile uint32_t    g_rnode_next_frag_due_ms = 0U;

static uint8_t              g_rnode_rx_reasm_buf[RNODE_HW_MTU];
static uint16_t             g_rnode_rx_reasm_len     = 0U;
static uint8_t              g_rnode_rx_reasm_seq     = 0U;
static uint8_t              g_rnode_rx_reasm_next    = 0U;

static void rnode_send_next_tx_frag(void) {
    uint16_t offset = (uint16_t)g_rnode_tx_frag_idx * 250U;
    if (offset >= g_rnode_tx_full_len) {
        return;
    }
    uint16_t rem = g_rnode_tx_full_len - offset;
    uint8_t chunk = (rem > 250U) ? 250U : (uint8_t)rem;
    uint8_t frag[254];
    frag[0] = 0xFEU;
    frag[1] = 0xCAU;
    frag[2] = g_rnode_tx_frag_seq;
    frag[3] = (uint8_t)((g_rnode_tx_frag_total << 4U) | (g_rnode_tx_frag_idx & 0x0FU));
    memcpy(&frag[4], &g_rnode_tx_full_buf[offset], chunk);
    g_rnode_tx_frag_idx++;
    lr2021_tx_nb(frag, (uint8_t)(4U + chunk));
}

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
        if (g_rnode_tx_frag_idx < g_rnode_tx_frag_total) {
            g_rnode_next_frag_due_ms = hal_millis() + 5U;
            if (g_rnode_next_frag_due_ms == 0U) {
                g_rnode_next_frag_due_ms = 1U;
            }
            return;
        }
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
                g_rnode_cfg.radio_online = false;
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
            if (len > 0U && len <= RNODE_HW_MTU && g_rnode_cfg.radio_online) {
                g_rnode_tx_busy = true;
                g_rnode_cfg.tx_bytes += (uint32_t)len;
                if (len <= 250U) {
                    g_rnode_tx_frag_idx   = 0U;
                    g_rnode_tx_frag_total = 0U;
                    lr2021_tx_nb(data, (uint8_t)len);
                } else {
                    memcpy(g_rnode_tx_full_buf, data, len);
                    g_rnode_tx_full_len      = (uint16_t)len;
                    g_rnode_tx_frag_seq++;
                    g_rnode_tx_frag_total    = (uint8_t)((len + 249U) / 250U);
                    g_rnode_tx_frag_idx      = 0U;
                    g_rnode_next_frag_due_ms = 0U;
                    rnode_send_next_tx_frag();
                }
            } else if (len > RNODE_HW_MTU) {
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
    refresh_rns_identity();

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
    char dest_hex[33], id_hex[33];
    format_hex_bytes(g_rns_id.dest_hash, 16U, dest_hex);
    format_hex_bytes(g_rns_id.identity_hash, 16U, id_hex);
    hal_usb_uart_printf("RNS_ID dest=%s id=%s role=%u name=\"%s\" ifac=%u peers=%u sel=%d rtt_ms=%u\r\n",
                        dest_hex, id_hex, (unsigned)g_rns_id.role, g_rns_id.name,
                        g_rns_id.ifac_enabled ? 1U : 0U,
                        (unsigned)g_rns_peers.count,
                        (int)g_rns_peers.selected_idx,
                        (unsigned)g_rns_peers.last_ping_rtt_ms);
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

static bool send_rns_node_announce(void) {
    uint8_t pkt[RNODE_LORA_MAX_PKT];
    size_t alen = rns_build_node_announce(&g_rns_id,
                                          g_rnode_cfg.frequency_hz,
                                          g_rnode_cfg.bandwidth_hz,
                                          g_rnode_cfg.sf,
                                          g_rnode_cfg.cr,
                                          &g_fc_tlm,
                                          pkt,
                                          sizeof(pkt));
    if (alen == 0U) {
        return false;
    }
    return send_or_tunnel_rns_packet(pkt, (uint16_t)alen);
}

static bool send_rns_pairing_sync(void) {
    rns_pairing_cfg_t pcfg;
    memset(&pcfg, 0, sizeof(pcfg));
    memcpy(pcfg.tx_dest, g_rns_id.dest_hash, 16U);
    pcfg.freq_hz      = g_rnode_cfg.frequency_hz;
    pcfg.bw_hz        = g_rnode_cfg.bandwidth_hz;
    pcfg.sf           = g_rnode_cfg.sf;
    pcfg.cr           = g_rnode_cfg.cr;
    pcfg.txpower_dbm  = g_rnode_cfg.txpower_dbm;
    pcfg.ifac_enabled = g_rns_id.ifac_enabled;
    memcpy(pcfg.ifac_pass, g_rns_id.ifac_pass, sizeof(pcfg.ifac_pass));

    uint8_t pkt[RNODE_LORA_MAX_PKT];
    size_t plen = rns_build_pairing_pkt(&pcfg, pkt, sizeof(pkt));
    if (plen == 0U) {
        return false;
    }
    return send_or_tunnel_rns_packet(pkt, (uint16_t)plen);
}

static bool send_rns_targeted_cmd(const uint8_t target_dest[16], const char *cmd_str, uint8_t seq) {
    uint8_t pkt[RNODE_LORA_MAX_PKT];
    size_t clen = rns_build_targeted_cmd_pkt(target_dest, g_rns_id.dest_hash, seq, cmd_str, pkt, sizeof(pkt));
    if (clen == 0U) {
        return false;
    }
    if (strcmp(cmd_str, "PING") == 0 || strncmp(cmd_str, "ACK:", 4) != 0) {
        g_rns_peers.last_ping_seq   = seq;
        g_rns_peers.last_ping_tx_ms = hal_millis();
    }
    return send_or_tunnel_rns_packet(pkt, (uint16_t)clen);
}

static void inspect_and_act_on_rns_packet(const uint8_t *pkt, uint16_t len, const char *src_tag) {
    if (pkt == NULL || len <= RNS_MIN_PACKET_LEN) {
        return;
    }
    bool ifac = (pkt[0] & 0x80U) != 0U;
    uint8_t hops = pkt[1];

    /* 0a. Check if Reticulum Node Announce (RNSA / 0x01) */
    if (rns_parse_node_announce(pkt, len, g_last_rssi_dbm, (int8_t)(g_last_snr_q2 / 4),
                                hal_millis(), &g_rns_peers)) {
        int peer_idx = 0;
        for (uint8_t i = 0U; i < g_rns_peers.count; i++) {
            if (memcmp(g_rns_peers.peers[i].dest_hash, &pkt[2], 16U) == 0) {
                peer_idx = (int)i;
                break;
            }
        }
        const rns_peer_entry_t *pe = &g_rns_peers.peers[peer_idx];
        char dhex[33];
        format_hex_bytes(pe->dest_hash, 16U, dhex);
        hal_usb_uart_printf("RNS_PEER_ANNOUNCE src=%s idx=%d dest=%s role=%u hops=%u name=\"%s\" vbatt=%umV mode=%u\r\n",
                            src_tag, peer_idx, dhex, (unsigned)pe->role, (unsigned)pe->hops,
                            pe->name, (unsigned)pe->vbatt_mv, (unsigned)pe->custom_mode);
        return;
    }

    /* 0b. Check if Over-the-Air Reticulum Pairing & Config Sync (RNSP) */
    rns_pairing_cfg_t pcfg;
    if (rns_parse_pairing_pkt(pkt, len, g_rns_id.dest_hash, &pcfg)) {
        g_rnode_cfg.frequency_hz = pcfg.freq_hz;
        g_rnode_cfg.bandwidth_hz = pcfg.bw_hz;
        g_rnode_cfg.sf           = pcfg.sf;
        g_rnode_cfg.cr           = pcfg.cr;
        g_rnode_cfg.txpower_dbm  = pcfg.txpower_dbm;
        rns_identity_set_ifac(&g_rns_id, pcfg.ifac_enabled, pcfg.ifac_pass);
        if (g_app_mode == APP_MODE_RNODE || g_app_mode == APP_MODE_RELAY) {
            configure_radio_for_rnode();
        }
        /* Record TX peer in peer table and immediately reply with our node announce */
        uint8_t fake_ann[68];
        rns_node_identity_t tx_id;
        memset(&tx_id, 0, sizeof(tx_id));
        memcpy(tx_id.dest_hash, pcfg.tx_dest, 16U);
        tx_id.role = RNS_NODE_ROLE_TX;
        strncpy(tx_id.name, "LR2021-TX", sizeof(tx_id.name) - 1U);
        rns_fc_telemetry_t zero_tlm;
        memset(&zero_tlm, 0, sizeof(zero_tlm));
        size_t alen = rns_build_node_announce(&tx_id,
                                              pcfg.freq_hz,
                                              pcfg.bw_hz,
                                              pcfg.sf,
                                              pcfg.cr,
                                              &zero_tlm,
                                              fake_ann,
                                              sizeof(fake_ann));
        if (alen > 0U) {
            fake_ann[1] = hops;
            (void)rns_parse_node_announce(fake_ann, alen, g_last_rssi_dbm, (int8_t)(g_last_snr_q2 / 4),
                                          hal_millis(), &g_rns_peers);
        }
        char tx_hex[33], my_hex[33];
        format_hex_bytes(pcfg.tx_dest, 16U, tx_hex);
        format_hex_bytes(g_rns_id.dest_hash, 16U, my_hex);
        bool ann_sent = send_rns_node_announce();
        hal_usb_uart_printf("RNS_PAIRED src=%s tx_dest=%s rx_dest=%s freq=%lu bw=%lu sf=%u cr=4/%u pwr=%ddBm ifac=%u reply_ann=%u\r\n",
                            src_tag, tx_hex, my_hex,
                            (unsigned long)g_rnode_cfg.frequency_hz,
                            (unsigned long)g_rnode_cfg.bandwidth_hz,
                            (unsigned)g_rnode_cfg.sf,
                            (unsigned)g_rnode_cfg.cr,
                            (int)g_rnode_cfg.txpower_dbm,
                            g_rns_id.ifac_enabled ? 1U : 0U,
                            ann_sent ? 1U : 0U);
        return;
    }

    /* 0c. Check if Targeted Reticulum Command / Ping / Pong (RNSC context 0xA6) */
    uint8_t sender_dest[16];
    uint8_t cmd_seq = 0U;
    char cmd_str[64];
    if (rns_parse_targeted_cmd_pkt(pkt, len, g_rns_id.dest_hash, sender_dest, &cmd_seq, cmd_str, sizeof(cmd_str))) {
        char sender_hex[33];
        format_hex_bytes(sender_dest, 16U, sender_hex);
        if (strcmp(cmd_str, "PING") == 0) {
            bool pong_ok = send_rns_targeted_cmd(sender_dest, "PONG", cmd_seq);
            hal_usb_uart_printf("RNS_PING_RX src=%s from=%s hops=%u seq=%u pong_sent=%u\r\n",
                                src_tag, sender_hex, hops, cmd_seq, pong_ok ? 1U : 0U);
            return;
        }
        if (strcmp(cmd_str, "PONG") == 0) {
            uint32_t dt = hal_millis() - g_rns_peers.last_ping_tx_ms;
            g_rns_peers.last_ping_rtt_ms = (dt == 0U) ? 1U : (dt > 65535U ? 65535U : (uint16_t)dt);
            hal_usb_uart_printf("RNS_PONG_RX src=%s from=%s hops=%u seq=%u rtt_ms=%u\r\n",
                                src_tag, sender_hex, hops, cmd_seq, (unsigned)g_rns_peers.last_ping_rtt_ms);
            return;
        }
        if (strncmp(cmd_str, "ACK:", 4) == 0) {
            uint32_t dt = hal_millis() - g_rns_peers.last_ping_tx_ms;
            g_rns_peers.last_ping_rtt_ms = (dt == 0U) ? 1U : (dt > 65535U ? 65535U : (uint16_t)dt);
            hal_usb_uart_printf("RNS_CMD_ACK_RX src=%s from=%s hops=%u seq=%u rtt_ms=%u ack=\"%s\"\r\n",
                                src_tag, sender_hex, hops, cmd_seq, (unsigned)g_rns_peers.last_ping_rtt_ms, cmd_str);
            return;
        }

        /* Targeted drone command addressed specifically to our 16-byte dest_hash */
        uint8_t tmp_pkt[128];
        size_t tlen = rns_build_drone_command_pkt(cmd_str, tmp_pkt, sizeof(tmp_pkt));
        rns_drone_cmd_t tdcmd;
        if (tlen > 0U && rns_parse_drone_command_pkt(tmp_pkt, tlen, &tdcmd)) {
            if (tdcmd.type == RNS_DRONE_CMD_RC_OVERRIDE) {
                for (int i = 0; i < 4; i++) {
                    if (tdcmd.rc_us[i] >= 800U) {
                        g_rc_channels[i] = crsf_us_to_ticks(tdcmd.rc_us[i]);
                    }
                }
            }
            uint8_t mav_frame[64];
            size_t mav_len = rns_build_mavlink_command(&tdcmd, g_mav_tx_seq++, mav_frame, sizeof(mav_frame));
            if (mav_len > 0U) {
                hal_crsf_uart_write(mav_frame, mav_len);
            }
            char ack_msg[48];
            snprintf(ack_msg, sizeof(ack_msg), "ACK:%s", cmd_str);
            bool ack_sent = send_rns_targeted_cmd(sender_dest, ack_msg, cmd_seq);
            hal_usb_uart_printf("RNS_TARGETED_CMD_RX src=%s from=%s hops=%u seq=%u cmd=\"%s\" type=%u mav_len=%u ack=%u\r\n",
                                src_tag, sender_hex, hops, cmd_seq, cmd_str,
                                (unsigned)tdcmd.type, (unsigned)mav_len, ack_sent ? 1U : 0U);
            return;
        }
    }

        /* 0d. Check if Multi-Drone Formation State / Setpoint Packet (RNSF context 0xA7) */
    rns_formation_pkt_t fp;
    if (rns_parse_formation_pkt(pkt, len, &fp)) {
        rns_drone_cmd_t fcmd;
        if (rns_compute_formation_slot_cmd(&fp, g_swarm_slot_id, &fcmd)) {
            uint8_t mav_frame[72];
            size_t mav_len = rns_build_mavlink_command(&fcmd, g_mav_tx_seq++, mav_frame, sizeof(mav_frame));
            if (mav_len > 0U) {
                hal_crsf_uart_write(mav_frame, mav_len);
            }
            g_last_mesh_ctrl_ms = hal_millis();
            g_mesh_ctrl_active  = (fp.shape != RNS_FORM_HOLD_BRAKE);
            hal_usb_uart_printf("RNS_FORM_RX src=%s hops=%u seq=%u shape=%u slot=%u lat=%ld lon=%ld alt_mm=%ld vx=%d vy=%d mav_len=%u\r\n",
                                src_tag, hops, (unsigned)fp.seq, (unsigned)fp.shape, (unsigned)g_swarm_slot_id,
                                (long)fcmd.lat_deg_e7, (long)fcmd.lon_deg_e7, (long)fp.leader_alt_mm,
                                (int)fp.vx_cms, (int)fp.vy_cms, (unsigned)mav_len);
            return;
        }
    }

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
                if (dcmd.rc_us[i] >= 800U) {
                    g_rc_channels[i] = crsf_us_to_ticks(dcmd.rc_us[i]);
                }
            }
        }
        uint8_t mav_frame[72];
        size_t mav_len = rns_build_mavlink_command(&dcmd, g_mav_tx_seq++, mav_frame, sizeof(mav_frame));
        if (mav_len > 0U) {
            hal_crsf_uart_write(mav_frame, mav_len);
        }
        if (dcmd.type == RNS_DRONE_CMD_VEL_NED || dcmd.type == RNS_DRONE_CMD_GLOBAL_POS_VEL) {
            g_last_mesh_ctrl_ms = hal_millis();
            g_mesh_ctrl_active  = true;
        } else if (dcmd.type == RNS_DRONE_CMD_LOITER || dcmd.type == RNS_DRONE_CMD_RTL || dcmd.type == RNS_DRONE_CMD_LAND) {
            g_mesh_ctrl_active  = false;
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

static size_t process_crsf_rns_frame(const uint8_t *frame, uint8_t frame_len, uint8_t out_resp[64]) {
    if (frame == NULL || frame_len < 4U || out_resp == NULL) {
        return 0U;
    }
    uint8_t ftype = frame[2];
    if (ftype == CRSF_FRAMETYPE_DEVICE_PING) {
        uint8_t hw_id[8];
        hal_get_device_id(hw_id);
        uint32_t serial = ((uint32_t)hw_id[0] << 24U) |
                          ((uint32_t)hw_id[1] << 16U) |
                          ((uint32_t)hw_id[2] << 8U)  |
                          (uint32_t)hw_id[3];
        return crsf_build_device_info_frame("LR2021 RNS-ELRS", serial, out_resp);
    }

    if (ftype != CRSF_FRAMETYPE_RNS_CMD || frame_len < 7U) {
        return 0U;
    }

    uint8_t subcmd = frame[5];
    switch (subcmd) {
        case CRSF_RNS_SUBCMD_GET_STATUS: {
            crsf_rns_status_t st;
            memset(&st, 0, sizeof(st));
            st.app_mode         = (uint8_t)g_app_mode;
            st.lock_state       = (uint8_t)g_lock_state;
            st.elrs_rate_idx    = g_rate_idx;
            st.rnode_txpower    = g_rnode_cfg.txpower_dbm;
            st.rnode_freq_hz    = g_rnode_cfg.frequency_hz;
            st.rnode_bw_khz     = (uint16_t)(g_rnode_cfg.bandwidth_hz / 1000UL);
            st.rnode_sf         = g_rnode_cfg.sf;
            st.rnode_cr         = g_rnode_cfg.cr;
            st.ifac_enabled     = g_rns_id.ifac_enabled;
            st.peer_count       = g_rns_peers.count;
            st.selected_peer    = (uint8_t)g_rns_peers.selected_idx;
            st.last_ping_rtt_ms = g_rns_peers.last_ping_rtt_ms;
            st.tx_msgs          = (uint16_t)(g_elrs_tunnel.tx_msgs_sent & 0xFFFFU);
            st.rx_msgs          = (uint16_t)(g_elrs_tunnel.rx_msgs_completed & 0xFFFFU);
            st.relayed_msgs     = (uint16_t)(g_rns_relay.relayed_count & 0xFFFFU);
            memcpy(st.tx_dest, g_rns_id.dest_hash, 16U);
            if (g_rns_peers.selected_idx >= 0 && g_rns_peers.selected_idx < (int8_t)g_rns_peers.count) {
                memcpy(st.rx_dest, g_rns_peers.peers[g_rns_peers.selected_idx].dest_hash, 16U);
            }
            return crsf_build_rns_status_frame(&st, out_resp);
        }

        case CRSF_RNS_SUBCMD_GET_PEER: {
            uint8_t idx = (frame_len >= 8U) ? frame[6] : 0U;
            crsf_rns_peer_info_t pi;
            memset(&pi, 0, sizeof(pi));
            pi.peer_idx    = idx;
            pi.total_peers = g_rns_peers.count;
            pi.is_selected = (g_rns_peers.selected_idx == (int8_t)idx);
            if (idx < g_rns_peers.count && g_rns_peers.peers[idx].valid) {
                const rns_peer_entry_t *pe = &g_rns_peers.peers[idx];
                pi.role        = pe->role;
                pi.hops        = pe->hops;
                pi.rssi_dbm    = pe->rssi_dbm;
                pi.snr_db      = pe->snr_db;
                pi.armed       = pe->armed;
                pi.custom_mode = pe->custom_mode;
                pi.vbatt_mv    = pe->vbatt_mv;
                uint32_t dt_s  = (hal_millis() - pe->last_seen_ms) / 1000U;
                pi.age_s       = (dt_s > 65535U) ? 65535U : (uint16_t)dt_s;
                memcpy(pi.dest_hash, pe->dest_hash, 16U);
                memcpy(pi.name, pe->name, 12U);
            }
            return crsf_build_rns_peer_frame(&pi, out_resp);
        }

        case CRSF_RNS_SUBCMD_SET_CFG: {
            /* Payload: [freq_be(4), bw_be(4), sf(1), cr(1), txpwr(1), ifac_en(1)] */
            crsf_rns_cfg_cmd_t cfg;
            if (crsf_decode_rns_set_cfg_frame(frame, frame_len, &cfg)) {
                g_rnode_cfg.frequency_hz = cfg.freq_hz;
                g_rnode_cfg.bandwidth_hz = cfg.bw_hz;
                g_rnode_cfg.sf           = cfg.sf;
                g_rnode_cfg.cr           = cfg.cr;
                g_rnode_cfg.txpower_dbm  = cfg.txpower_dbm;
                rns_identity_set_ifac(&g_rns_id, cfg.ifac_enabled, g_rns_id.ifac_pass[0] ? g_rns_id.ifac_pass : "elrs-mesh");
                if (g_app_mode == APP_MODE_RNODE || g_app_mode == APP_MODE_RELAY) {
                    configure_radio_for_rnode();
                }
                return crsf_build_rns_ack_frame(subcmd, 0U, out_resp);
            }
            return crsf_build_rns_ack_frame(subcmd, 1U, out_resp);
        }

        case CRSF_RNS_SUBCMD_PAIR_RX: {
            bool ok = send_rns_pairing_sync();
            return crsf_build_rns_ack_frame(subcmd, ok ? 0U : 1U, out_resp);
        }

        case CRSF_RNS_SUBCMD_SELECT_PEER: {
            if (frame_len >= 8U) {
                int8_t sel = (int8_t)frame[6];
                if (sel >= 0 && sel < (int8_t)g_rns_peers.count) {
                    g_rns_peers.selected_idx = sel;
                    return crsf_build_rns_ack_frame(subcmd, 0U, out_resp);
                }
            }
            return crsf_build_rns_ack_frame(subcmd, 1U, out_resp);
        }

        case CRSF_RNS_SUBCMD_ACTION: {
            uint8_t act = (frame_len >= 8U) ? frame[6] : 0U;
            if (act == CRSF_RNS_ACT_ANNOUNCE) {
                bool ok = send_rns_node_announce();
                return crsf_build_rns_ack_frame(subcmd, ok ? 0U : 1U, out_resp);
            }
            const uint8_t *target_dest = RNS_DEST_DRONE_CMD;
            if (g_rns_peers.selected_idx >= 0 && g_rns_peers.selected_idx < (int8_t)g_rns_peers.count) {
                target_dest = g_rns_peers.peers[g_rns_peers.selected_idx].dest_hash;
            }
            const char *cstr = NULL;
            if (act == CRSF_RNS_ACT_PING)         cstr = "PING";
            else if (act == CRSF_RNS_ACT_LOITER)  cstr = "LOITER";
            else if (act == CRSF_RNS_ACT_RTL)     cstr = "RTL";
            else if (act == CRSF_RNS_ACT_LAND)    cstr = "LAND";
            else if (act == CRSF_RNS_ACT_ARM)     cstr = "ARM";
            else if (act == CRSF_RNS_ACT_DISARM)  cstr = "DISARM";
            if (cstr != NULL) {
                bool ok = send_rns_targeted_cmd(target_dest, cstr, g_rns_cmd_seq++);
                return crsf_build_rns_ack_frame(subcmd, ok ? 0U : 1U, out_resp);
            }
            return crsf_build_rns_ack_frame(subcmd, 1U, out_resp);
        }

        default:
            return crsf_build_rns_ack_frame(subcmd, 1U, out_resp);
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
            "  rns_id                         - Show this node's 16-byte Reticulum SINGLE address & ID\r\n"
            "  rns_ann                        - Broadcast signed Reticulum node announce (RNSA)\r\n"
            "  rns_pair [ifac_pass]           - OTA sync Reticulum RF/IFAC config & pair bound RX (RNSP)\r\n"
            "  rns_peers [select <idx>]       - List discovered Reticulum peers or select target RX\r\n"
            "  rns_to <dest|sel> <cmd>        - Send command/PING to specific 16B Reticulum RX address\r\n"
            "  crsf_lua <status|peer|ann|pair|ping|loiter|rtl> - Test EdgeTX Lua CRSF 0x7A/0x7B bridge\r\n"
            "  crsf_ota <status|peer|ann|pair|ping|loiter|rtl> - Tunnel CRSF 0x7A request over the air to RX\r\n"
            "  rns_slot <0..7>                - Set drone formation slot (0=Leader, 1=Left1, 2=Right1, ...)\r\n"
            "  rns_form <vee|line|col|hold> [spacing_m] [vn_ms] [ve_ms] [hdg_deg] - Broadcast formation state\r\n"
            "  rns_cmd <GOTO|VEL|BVL|RTL|LAND|LOITER|ARM|DISARM|MODE n|RC ...>    - Send RNS drone cmd\r\n"
            "  rns_tx <hex_bytes>             - Send raw Reticulum packet over Relay LoRa or ELRS tunnel\r\n"
            "  fc_sim <lat_e7> <lon_e7> <alt_m> <vbatt_mv> - Set simulated FC telemetry for beacons\r\n"
            "  mav_rx <hex_bytes>             - Feed raw MAVLink v1/v2 frame into FC telemetry parser\r\n"
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
        refresh_rns_identity();
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
        refresh_rns_identity();
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

    if (strcmp(cmd, "mav_rx") == 0) {
        char *hex = strtok(NULL, " ");
        if (hex == NULL) {
            hal_usb_uart_puts("ERR: usage: mav_rx <hex_bytes>\r\n");
            return;
        }
        size_t slen = strlen(hex);
        for (size_t i = 0U; i + 1U < slen; i += 2U) {
            char pair[3] = {hex[i], hex[i + 1U], '\0'};
            uint8_t b = (uint8_t)strtoul(pair, NULL, 16);
            (void)rns_mav_parse_byte(&g_mav_parser, b, &g_fc_tlm);
        }
        hal_usb_uart_printf("OK MAV_RX pkts=%lu mode=%u armed=%u vbatt=%umV lat=%ld lon=%ld alt_mm=%ld\r\n",
                            (unsigned long)g_fc_tlm.rx_mav_pkts,
                            (unsigned)g_fc_tlm.custom_mode,
                            g_fc_tlm.armed ? 1U : 0U,
                            (unsigned)g_fc_tlm.vbatt_mv,
                            (long)g_fc_tlm.lat_deg_e7,
                            (long)g_fc_tlm.lon_deg_e7,
                            (long)g_fc_tlm.rel_alt_mm);
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

    if (strcmp(cmd, "rns_id") == 0) {
        uint8_t hw_id[8];
        hal_get_device_id(hw_id);
        char hw_hex[17], dest_hex[33], id_hex[33], sel_hex[33] = "none";
        format_hex_bytes(hw_id, 8U, hw_hex);
        format_hex_bytes(g_rns_id.dest_hash, 16U, dest_hex);
        format_hex_bytes(g_rns_id.identity_hash, 16U, id_hex);
        if (g_rns_peers.selected_idx >= 0 && g_rns_peers.selected_idx < (int8_t)g_rns_peers.count) {
            format_hex_bytes(g_rns_peers.peers[g_rns_peers.selected_idx].dest_hash, 16U, sel_hex);
        }
        hal_usb_uart_printf("OK RNS_IDENTITY hw_id=%s dest=%s id=%s role=%u name=\"%s\" ifac=%u peers=%u sel_rx=%s\r\n",
                            hw_hex, dest_hex, id_hex, (unsigned)g_rns_id.role, g_rns_id.name,
                            g_rns_id.ifac_enabled ? 1U : 0U,
                            (unsigned)g_rns_peers.count, sel_hex);
        return;
    }

    if (strcmp(cmd, "rns_ann") == 0) {
        bool ok = send_rns_node_announce();
        char dest_hex[33];
        format_hex_bytes(g_rns_id.dest_hash, 16U, dest_hex);
        hal_usb_uart_printf("OK RNS_ANNOUNCE_TX sent=%u dest=%s role=%u name=\"%s\"\r\n",
                            ok ? 1U : 0U, dest_hex, (unsigned)g_rns_id.role, g_rns_id.name);
        return;
    }

    if (strcmp(cmd, "rns_pair") == 0) {
        char *pass = strtok(NULL, " ");
        if (pass != NULL) {
            if (strcasecmp(pass, "off") == 0 || strcasecmp(pass, "none") == 0) {
                rns_identity_set_ifac(&g_rns_id, false, NULL);
            } else {
                rns_identity_set_ifac(&g_rns_id, true, pass);
            }
        }
        bool ok = send_rns_pairing_sync();
        char tx_hex[33];
        format_hex_bytes(g_rns_id.dest_hash, 16U, tx_hex);
        hal_usb_uart_printf("OK RNS_PAIR_TX sent=%u tx_dest=%s freq=%lu bw=%lu sf=%u cr=4/%u pwr=%ddBm ifac=%u\r\n",
                            ok ? 1U : 0U, tx_hex,
                            (unsigned long)g_rnode_cfg.frequency_hz,
                            (unsigned long)g_rnode_cfg.bandwidth_hz,
                            (unsigned)g_rnode_cfg.sf,
                            (unsigned)g_rnode_cfg.cr,
                            (int)g_rnode_cfg.txpower_dbm,
                            g_rns_id.ifac_enabled ? 1U : 0U);
        return;
    }

    if (strcmp(cmd, "rns_peers") == 0) {
        char *sub = strtok(NULL, " ");
        if (sub != NULL && (strcasecmp(sub, "select") == 0 || strcasecmp(sub, "sel") == 0)) {
            char *idx_s = strtok(NULL, " ");
            if (idx_s == NULL) {
                hal_usb_uart_puts("ERR: usage: rns_peers select <idx>\r\n");
                return;
            }
            int idx = (int)strtol(idx_s, NULL, 10);
            if (idx >= 0 && idx < (int)g_rns_peers.count && g_rns_peers.peers[idx].valid) {
                g_rns_peers.selected_idx = (int8_t)idx;
                char dhex[33];
                format_hex_bytes(g_rns_peers.peers[idx].dest_hash, 16U, dhex);
                hal_usb_uart_printf("OK RNS_PEER_SEL idx=%d dest=%s name=\"%s\"\r\n",
                                    idx, dhex, g_rns_peers.peers[idx].name);
            } else {
                hal_usb_uart_printf("ERR: invalid peer idx %d (count=%u)\r\n", idx, (unsigned)g_rns_peers.count);
            }
            return;
        }
        hal_usb_uart_printf("RNS_PEERS count=%u selected=%d last_rtt_ms=%u\r\n",
                            (unsigned)g_rns_peers.count,
                            (int)g_rns_peers.selected_idx,
                            (unsigned)g_rns_peers.last_ping_rtt_ms);
        uint32_t now_ms = hal_millis();
        for (uint8_t i = 0U; i < g_rns_peers.count; i++) {
            const rns_peer_entry_t *pe = &g_rns_peers.peers[i];
            if (!pe->valid) continue;
            char dhex[33];
            format_hex_bytes(pe->dest_hash, 16U, dhex);
            uint32_t age_ms = now_ms - pe->last_seen_ms;
            hal_usb_uart_printf("  [%u]%s dest=%s role=%u hops=%u rssi=%ddBm snr=%ddB vbatt=%umV mode=%u age=%lums name=\"%s\"\r\n",
                                (unsigned)i,
                                (g_rns_peers.selected_idx == (int8_t)i) ? "*" : " ",
                                dhex, (unsigned)pe->role, (unsigned)pe->hops,
                                (int)pe->rssi_dbm, (int)pe->snr_db,
                                (unsigned)pe->vbatt_mv, (unsigned)pe->custom_mode,
                                (unsigned long)age_ms, pe->name);
        }
        return;
    }

    if (strcmp(cmd, "rns_to") == 0) {
        char *dest_arg = strtok(NULL, " ");
        char *rest     = strtok(NULL, "");
        if (dest_arg == NULL || rest == NULL) {
            hal_usb_uart_puts("ERR: usage: rns_to <32_hex_dest | sel | bcast> <PING | LOITER | RTL | LAND | ARM | DISARM | MODE n | GOTO ...>\r\n");
            return;
        }
        while (*rest && isspace((unsigned char)*rest)) rest++;
        uint8_t target_dest[16];
        if (strcasecmp(dest_arg, "sel") == 0) {
            if (g_rns_peers.selected_idx < 0 || g_rns_peers.selected_idx >= (int8_t)g_rns_peers.count) {
                hal_usb_uart_puts("ERR: no selected peer in table\r\n");
                return;
            }
            memcpy(target_dest, g_rns_peers.peers[g_rns_peers.selected_idx].dest_hash, 16U);
        } else if (strcasecmp(dest_arg, "bcast") == 0) {
            memcpy(target_dest, RNS_DEST_DRONE_CMD, 16U);
        } else if (!parse_hex_bytes(dest_arg, target_dest, 16U)) {
            hal_usb_uart_puts("ERR: destination must be 32 hex chars (16 bytes), 'sel', or 'bcast'\r\n");
            return;
        }
        uint8_t seq = g_rns_cmd_seq++;
        bool ok = send_rns_targeted_cmd(target_dest, rest, seq);
        char dhex[33];
        format_hex_bytes(target_dest, 16U, dhex);
        hal_usb_uart_printf("OK RNS_TO_TX sent=%u to=%s seq=%u cmd=\"%s\"\r\n",
                            ok ? 1U : 0U, dhex, (unsigned)seq, rest);
        return;
    }

    if (strcmp(cmd, "crsf_lua") == 0) {
        char *sub = strtok(NULL, " ");
        if (sub == NULL) {
            hal_usb_uart_puts("ERR: usage: crsf_lua <status|peer [idx]|ann|pair|ping|loiter|rtl>\r\n");
            return;
        }
        uint8_t req[32];
        size_t req_len = 0U;
        if (strcasecmp(sub, "status") == 0) {
            req_len = crsf_build_rns_cmd_frame(CRSF_RNS_SUBCMD_GET_STATUS, NULL, 0U, req);
        } else if (strcasecmp(sub, "peer") == 0) {
            char *idx_s = strtok(NULL, " ");
            uint8_t idx = idx_s ? (uint8_t)strtoul(idx_s, NULL, 10) : 0U;
            req_len = crsf_build_rns_cmd_frame(CRSF_RNS_SUBCMD_GET_PEER, &idx, 1U, req);
        } else if (strcasecmp(sub, "pair") == 0) {
            req_len = crsf_build_rns_cmd_frame(CRSF_RNS_SUBCMD_PAIR_RX, NULL, 0U, req);
        } else {
            uint8_t act = 0U;
            if (strcasecmp(sub, "ann") == 0)         act = CRSF_RNS_ACT_ANNOUNCE;
            else if (strcasecmp(sub, "ping") == 0)   act = CRSF_RNS_ACT_PING;
            else if (strcasecmp(sub, "loiter") == 0) act = CRSF_RNS_ACT_LOITER;
            else if (strcasecmp(sub, "rtl") == 0)    act = CRSF_RNS_ACT_RTL;
            else if (strcasecmp(sub, "land") == 0)   act = CRSF_RNS_ACT_LAND;
            else if (strcasecmp(sub, "arm") == 0)    act = CRSF_RNS_ACT_ARM;
            else if (strcasecmp(sub, "disarm") == 0) act = CRSF_RNS_ACT_DISARM;
            if (act != 0U) {
                req_len = crsf_build_rns_cmd_frame(CRSF_RNS_SUBCMD_ACTION, &act, 1U, req);
            }
        }
        uint8_t resp[64];
        size_t resp_len = process_crsf_rns_frame(req, (uint8_t)req_len, resp);
        if (resp_len == 0U) {
            hal_usb_uart_puts("ERR: crsf_lua failed\r\n");
            return;
        }
        if (resp[5] == CRSF_RNS_RESP_STATUS) {
            crsf_rns_status_t st;
            if (crsf_decode_rns_status_frame(resp, resp_len, &st)) {
                char txh[33], rxh[33];
                format_hex_bytes(st.tx_dest, 16U, txh);
                format_hex_bytes(st.rx_dest, 16U, rxh);
                hal_usb_uart_printf("OK CRSF_LUA_STATUS len=%u mode=%u lock=%u freq=%lu bw_khz=%u sf=%u cr=4/%u peers=%u rtt=%ums tx_dest=%s rx_dest=%s\r\n",
                                    (unsigned)resp_len, st.app_mode, st.lock_state,
                                    (unsigned long)st.rnode_freq_hz, (unsigned)st.rnode_bw_khz,
                                    st.rnode_sf, st.rnode_cr, st.peer_count, st.last_ping_rtt_ms,
                                    txh, rxh);
            }
        } else if (resp[5] == CRSF_RNS_RESP_PEER) {
            crsf_rns_peer_info_t pi;
            if (crsf_decode_rns_peer_frame(resp, resp_len, &pi)) {
                char dh[33];
                format_hex_bytes(pi.dest_hash, 16U, dh);
                hal_usb_uart_printf("OK CRSF_LUA_PEER len=%u idx=%u/%u role=%u hops=%u rssi=%ddBm snr=%ddB vbatt=%umV dest=%s name=\"%s\"\r\n",
                                    (unsigned)resp_len, pi.peer_idx, pi.total_peers, pi.role, pi.hops,
                                    (int)pi.rssi_dbm, (int)pi.snr_db, (unsigned)pi.vbatt_mv, dh, pi.name);
            }
        } else if (resp[5] == CRSF_RNS_RESP_ACK) {
            hal_usb_uart_printf("OK CRSF_LUA_ACK len=%u subcmd=0x%02X status=%u\r\n",
                                (unsigned)resp_len, resp[6], resp[7]);
        }
        return;
    }

    if (strcmp(cmd, "crsf_ota") == 0) {
        char *sub = strtok(NULL, " ");
        if (sub == NULL) {
            hal_usb_uart_puts("ERR: usage: crsf_ota <status|peer [idx]|ann|pair|ping|loiter|rtl>\r\n");
            return;
        }
        uint8_t req[32];
        size_t req_len = 0U;
        if (strcasecmp(sub, "status") == 0) {
            req_len = crsf_build_rns_cmd_frame(CRSF_RNS_SUBCMD_GET_STATUS, NULL, 0U, req);
        } else if (strcasecmp(sub, "peer") == 0) {
            char *idx_s = strtok(NULL, " ");
            uint8_t idx = idx_s ? (uint8_t)strtoul(idx_s, NULL, 10) : 0U;
            req_len = crsf_build_rns_cmd_frame(CRSF_RNS_SUBCMD_GET_PEER, &idx, 1U, req);
        } else if (strcasecmp(sub, "pair") == 0) {
            req_len = crsf_build_rns_cmd_frame(CRSF_RNS_SUBCMD_PAIR_RX, NULL, 0U, req);
        } else {
            uint8_t act = 0U;
            if (strcasecmp(sub, "ann") == 0)         act = CRSF_RNS_ACT_ANNOUNCE;
            else if (strcasecmp(sub, "ping") == 0)   act = CRSF_RNS_ACT_PING;
            else if (strcasecmp(sub, "loiter") == 0) act = CRSF_RNS_ACT_LOITER;
            else if (strcasecmp(sub, "rtl") == 0)    act = CRSF_RNS_ACT_RTL;
            else if (strcasecmp(sub, "land") == 0)   act = CRSF_RNS_ACT_LAND;
            else if (strcasecmp(sub, "arm") == 0)    act = CRSF_RNS_ACT_ARM;
            else if (strcasecmp(sub, "disarm") == 0) act = CRSF_RNS_ACT_DISARM;
            if (act != 0U) {
                req_len = crsf_build_rns_cmd_frame(CRSF_RNS_SUBCMD_ACTION, &act, 1U, req);
            }
        }
        if (req_len == 0U) {
            hal_usb_uart_puts("ERR: crsf_ota invalid subcommand\r\n");
            return;
        }
        /* Address to 0xEC (CRSF_ADDRESS_CRSF_RECEIVER) so it tunnels over the air to the bound RX */
        req[0] = CRSF_ADDRESS_CRSF_RECEIVER;
        req[3] = CRSF_ADDRESS_CRSF_RECEIVER;
        req[req_len - 1U] = crsf_crc8(&req[2], req_len - 3U);
        bool sent = send_or_tunnel_rns_packet(req, (uint16_t)req_len);
        hal_usb_uart_printf("OK CRSF_OTA_TX sent=%u len=%u sub=\"%s\"\r\n",
                            sent ? 1U : 0U, (unsigned)req_len, sub);
        return;
    }

    if (strcmp(cmd, "rns_slot") == 0) {
        char *arg = strtok(NULL, " ");
        if (arg != NULL) {
            uint8_t slot = (uint8_t)strtoul(arg, NULL, 10);
            if (slot <= 7U) {
                g_swarm_slot_id = slot;
            }
        }
        hal_usb_uart_printf("OK RNS_SLOT slot=%u role=%s\r\n",
                            (unsigned)g_swarm_slot_id,
                            (g_swarm_slot_id == 0U) ? "LEADER" : "FOLLOWER");
        return;
    }

    if (strcmp(cmd, "rns_form") == 0) {
        char *shape_s = strtok(NULL, " ");
        if (shape_s == NULL) {
            hal_usb_uart_puts("ERR: usage: rns_form <vee|line|col|hold> [spacing_m] [vn_ms] [ve_ms] [hdg_deg]\r\n");
            return;
        }
        uint8_t shape = RNS_FORM_VEE;
        if (strcasecmp(shape_s, "vee") == 0)       shape = RNS_FORM_VEE;
        else if (strcasecmp(shape_s, "line") == 0) shape = RNS_FORM_LINE_ABREAST;
        else if (strcasecmp(shape_s, "col") == 0)  shape = RNS_FORM_COLUMN;
        else if (strcasecmp(shape_s, "hold") == 0 || strcasecmp(shape_s, "brake") == 0) shape = RNS_FORM_HOLD_BRAKE;

        char *sp_s  = strtok(NULL, " ");
        char *vn_s  = strtok(NULL, " ");
        char *ve_s  = strtok(NULL, " ");
        char *hdg_s = strtok(NULL, " ");

        float spacing_m = sp_s  ? strtof(sp_s, NULL)  : 10.0f;
        float vn_ms     = vn_s  ? strtof(vn_s, NULL)  : 0.0f;
        float ve_ms     = ve_s  ? strtof(ve_s, NULL)  : 0.0f;
        float hdg_deg   = hdg_s ? strtof(hdg_s, NULL) : 0.0f;

        rns_formation_pkt_t fp;
        memset(&fp, 0, sizeof(fp));
        memcpy(fp.leader_dest, g_rns_id.dest_hash, 16U);
        fp.seq           = g_form_seq++;
        fp.shape         = shape;
        fp.spacing_dm    = (uint16_t)(spacing_m * 10.0f);
        fp.leader_lat_e7 = (g_fc_tlm.lat_deg_e7 != 0) ? g_fc_tlm.lat_deg_e7 : -353632620L;
        fp.leader_lon_e7 = (g_fc_tlm.lon_deg_e7 != 0) ? g_fc_tlm.lon_deg_e7 : 1491652370L;
        fp.leader_alt_mm = (g_fc_tlm.rel_alt_mm != 0) ? g_fc_tlm.rel_alt_mm : 25000L;
        fp.vx_cms        = (int16_t)(vn_ms * 100.0f);
        fp.vy_cms        = (int16_t)(ve_ms * 100.0f);
        fp.vz_cms        = 0;
        fp.heading_cdeg  = (uint16_t)(hdg_deg * 100.0f);

        uint8_t pkt[64];
        size_t flen = rns_build_formation_pkt(&fp, pkt, sizeof(pkt));
        bool ok = send_or_tunnel_rns_packet(pkt, (uint16_t)flen);
        hal_usb_uart_printf("OK RNS_FORM_TX sent=%u seq=%u shape=%u spacing_dm=%u lat=%ld lon=%ld vx=%d vy=%d hdg=%u\r\n",
                            ok ? 1U : 0U, (unsigned)fp.seq, (unsigned)fp.shape, (unsigned)fp.spacing_dm,
                            (long)fp.leader_lat_e7, (long)fp.leader_lon_e7,
                            (int)fp.vx_cms, (int)fp.vy_cms, (unsigned)fp.heading_cdeg);
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
    crsf_parser_init(&g_usb_crsf_parser);
    rnode_kiss_init(&g_kiss_parser, on_rnode_kiss_frame, NULL);
    rns_relay_init(&g_rns_relay);
    rns_peer_table_init(&g_rns_peers);
    rns_mav_parser_init(&g_mav_parser);
    memset(&g_fc_tlm, 0, sizeof(g_fc_tlm));
    g_fc_tlm.vbatt_mv = 24800U; /* Default 6S nominal until MAVLink SYS_STATUS arrives */
    elrs_tunnel_init(&g_elrs_tunnel, 2U);

    /* Default binding phrase */
    uint8_t default_uid[6];
    elrs_uid_from_bind_phrase("lr2021-elrs-demo", default_uid);
    elrs_ota_init(&g_ota, default_uid, ELRS_OTA4_PACKET_SIZE, ELRS_SW_WIDE_OR_8CH);
    elrs_fhss_init(&g_fhss, g_domain, g_ota.uid_seed);
    memset(&g_rns_id, 0, sizeof(g_rns_id));
    refresh_rns_identity();

    lr2021_set_callbacks(radio_rx_done_isr, radio_tx_done_isr);

    uint16_t chip_ver = 0U;
    bool radio_ok = lr2021_begin(868000000UL, 2440000000UL, &chip_ver);
    g_chip_version = chip_ver;

    char boot_dest[33];
    format_hex_bytes(g_rns_id.dest_hash, 16U, boot_dest);
    hal_usb_uart_printf("\r\n========================================================\r\n");
    hal_usb_uart_printf("LR2021-ELRS + Reticulum RNode/Relay Firmware (nRF54L15)\r\n");
    hal_usb_uart_printf("LR2021 Init: %s (Version=0x%04X, Errors=0x%04X)\r\n",
                        radio_ok ? "OK" : "FAIL", chip_ver, lr2021_get_errors());
    hal_usb_uart_printf("Default UID: %02X:%02X:%02X:%02X:%02X:%02X (phrase=\"lr2021-elrs-demo\")\r\n",
                        g_ota.uid[0], g_ota.uid[1], g_ota.uid[2],
                        g_ota.uid[3], g_ota.uid[4], g_ota.uid[5]);
    hal_usb_uart_printf("Reticulum SINGLE Dest: %s\r\n", boot_dest);
    hal_usb_uart_printf("========================================================\r\n");

    char cli_buf[192];
    size_t cli_pos = 0U;
    uint32_t last_crsf_linkstats_ms = 0U;

    while (1) {
        /* 1. Process incoming USB CDC CLI, RNode KISS, or Plug-and-Play USB-C CRSF bytes */
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
            /* Plug-and-play USB-C CRSF auto-detection (0xEA/0xEC/0xEE/0xC8 frames) */
            if (g_usb_crsf_parser.pos > 0U ||
                ub == CRSF_SYNC_BYTE || ub == CRSF_ADDRESS_HANDSET ||
                ub == CRSF_ADDRESS_CRSF_RECEIVER || ub == CRSF_ADDRESS_CRSF_TRANSMITTER) {
                if (crsf_parser_feed_byte(&g_usb_crsf_parser, ub)) {
                    uint8_t ftype = g_usb_crsf_parser.buf[2];
                    uint8_t flen  = (uint8_t)(g_usb_crsf_parser.expected_len + 2U);
                    cli_pos = 0U;
                    if (ftype == CRSF_FRAMETYPE_RC_CHANNELS) {
                        (void)crsf_decode_rc_channels_frame(g_usb_crsf_parser.buf, flen, g_rc_channels);
                        g_rc_is_armed = (g_rc_channels[4] > ELRS_CRSF_CH_MID);
                    } else if (ftype == CRSF_FRAMETYPE_DEVICE_PING || ftype == CRSF_FRAMETYPE_RNS_CMD) {
                        if (g_app_mode == APP_MODE_ELRS_TX && g_usb_crsf_parser.buf[3] == CRSF_ADDRESS_CRSF_RECEIVER) {
                            (void)send_or_tunnel_rns_packet(g_usb_crsf_parser.buf, flen);
                        } else {
                            uint8_t resp[64];
                            size_t rlen = process_crsf_rns_frame(g_usb_crsf_parser.buf, flen, resp);
                            if (rlen > 0U) {
                                hal_usb_uart_write(resp, rlen);
                            }
                        }
                    }
                }
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
                } else if (ftype == CRSF_FRAMETYPE_DEVICE_PING || ftype == CRSF_FRAMETYPE_RNS_CMD) {
                    if (g_app_mode == APP_MODE_ELRS_TX && g_crsf_parser.buf[3] == CRSF_ADDRESS_CRSF_RECEIVER) {
                        /* Forward handset CRSF request addressed to 0xEC over the air to the bound RX */
                        (void)send_or_tunnel_rns_packet(g_crsf_parser.buf, flen);
                    } else {
                        uint8_t resp[64];
                        size_t rlen = process_crsf_rns_frame(g_crsf_parser.buf, flen, resp);
                        if (rlen > 0U) {
                            hal_crsf_uart_write(resp, rlen);
                        }
                    }
                }
            }
            (void)rns_mav_parse_byte(&g_mav_parser, ub, &g_fc_tlm);
        }

        /* 3. Handle RNode TX_DONE notification & incoming LoRa packets in RNODE / RELAY mode */
        uint32_t now_ms = hal_millis();
        if (g_app_mode == APP_MODE_RNODE && g_rnode_next_frag_due_ms != 0U &&
            (int32_t)(now_ms - g_rnode_next_frag_due_ms) >= 0) {
            g_rnode_next_frag_due_ms = 0U;
            rnode_send_next_tx_frag();
        }
        if (g_app_mode == APP_MODE_RNODE && g_rnode_tx_done_notify) {
            g_rnode_tx_done_notify = false;
            rnode_send_u8(RNODE_CMD_READY, 0x01U);
        }

        if (g_rns_rx_flag) {
            uint8_t rx_copy[RNODE_LORA_MAX_PKT];
            uint8_t rx_len = g_rns_rx_len;
            lr2021_pkt_stats_t rx_st = g_rns_rx_stats;
            memcpy(rx_copy, g_rns_rx_buf, rx_len);
            g_rns_rx_flag = false;

            if (g_app_mode == APP_MODE_RNODE) {
                const uint8_t *emit_ptr = rx_copy;
                uint16_t emit_len       = rx_len;
                bool emit_ready         = true;

                if (rx_len >= 5U && rx_copy[0] == 0xFEU && rx_copy[1] == 0xCAU) {
                    uint8_t seq   = rx_copy[2];
                    uint8_t total = (uint8_t)(rx_copy[3] >> 4U);
                    uint8_t idx   = (uint8_t)(rx_copy[3] & 0x0FU);
                    uint8_t clen  = (uint8_t)(rx_len - 4U);
                    emit_ready    = false;
                    if (idx == 0U && total > 1U) {
                        memcpy(g_rnode_rx_reasm_buf, &rx_copy[4], clen);
                        g_rnode_rx_reasm_len  = clen;
                        g_rnode_rx_reasm_seq  = seq;
                        g_rnode_rx_reasm_next = 1U;
                    } else if (seq == g_rnode_rx_reasm_seq && idx == g_rnode_rx_reasm_next &&
                               (uint16_t)(g_rnode_rx_reasm_len + clen) <= RNODE_HW_MTU) {
                        memcpy(&g_rnode_rx_reasm_buf[g_rnode_rx_reasm_len], &rx_copy[4], clen);
                        g_rnode_rx_reasm_len = (uint16_t)(g_rnode_rx_reasm_len + clen);
                        g_rnode_rx_reasm_next++;
                        if (g_rnode_rx_reasm_next >= total) {
                            emit_ptr   = g_rnode_rx_reasm_buf;
                            emit_len   = g_rnode_rx_reasm_len;
                            emit_ready = true;
                        }
                    }
                }

                if (emit_ready) {
                    int rssi_enc = (int)rx_st.rssi_dbm + RNODE_RSSI_OFFSET;
                    if (rssi_enc < 0) rssi_enc = 0;
                    if (rssi_enc > 255) rssi_enc = 255;
                    rnode_send_u8(RNODE_CMD_STAT_RSSI, (uint8_t)rssi_enc);
                    rnode_send_u8(RNODE_CMD_STAT_SNR, (uint8_t)rx_st.snr_q2_db);
                    rnode_send_frame(RNODE_CMD_DATA, emit_ptr, emit_len);
                    hal_led_toggle();
                }
            } else if (g_app_mode == APP_MODE_RELAY) {
                hal_led_toggle();
                uint8_t orig_hops = (rx_len > 1U) ? rx_copy[1] : 0U;
                inspect_and_act_on_rns_packet(rx_copy, rx_len, "LORA");
                /* Cross-link bridge: if ELRS tunnel is free, also queue for downlink/uplink tunnel */
                (void)elrs_tunnel_enqueue_tx(&g_elrs_tunnel, rx_copy, rx_len, g_ota.is_full_res);
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

        /* 4. Handle reassembled Reticulum packet or Over-the-Air CRSF frame from ExpressLRS DATA tunnel */
        if (g_tunnel_rx_ready) {
            uint8_t tun_pkt[ELRS_TUNNEL_MAX_MTU];
            uint16_t tun_len = g_tunnel_rx_len;
            memcpy(tun_pkt, g_tunnel_rx_buf, tun_len);
            g_tunnel_rx_ready = false;

            /* Check if the tunneled message is a CRSF frame (0x28/0x29/0x21/0x7A/0x7B) */
            bool is_crsf = false;
            if (tun_len >= 4U && tun_len <= CRSF_MAX_FRAME_LEN &&
                (tun_pkt[0] == CRSF_SYNC_BYTE || tun_pkt[0] == CRSF_ADDRESS_HANDSET ||
                 tun_pkt[0] == CRSF_ADDRESS_CRSF_RECEIVER || tun_pkt[0] == CRSF_ADDRESS_CRSF_TRANSMITTER) &&
                (uint16_t)(tun_pkt[1] + 2U) == tun_len &&
                crsf_crc8(&tun_pkt[2], (size_t)(tun_pkt[1] - 1U)) == tun_pkt[tun_len - 1U]) {
                uint8_t ftype = tun_pkt[2];
                if (ftype == CRSF_FRAMETYPE_DEVICE_PING || ftype == CRSF_FRAMETYPE_RNS_CMD) {
                    is_crsf = true;
                    uint8_t resp[64];
                    size_t rlen = process_crsf_rns_frame(tun_pkt, (uint8_t)tun_len, resp);
                    if (rlen > 0U) {
                        bool sent = send_or_tunnel_rns_packet(resp, (uint16_t)rlen);
                        hal_usb_uart_printf("OTA_CRSF_REQ_RX ftype=0x%02X sub=0x%02X resp_len=%u sent=%u\r\n",
                                            ftype, (tun_len >= 6U) ? tun_pkt[5] : 0U,
                                            (unsigned)rlen, sent ? 1U : 0U);
                    }
                } else if (ftype == CRSF_FRAMETYPE_DEVICE_INFO ||
                           ftype == CRSF_FRAMETYPE_RNS_TLM ||
                           ftype == CRSF_FRAMETYPE_FLIGHT_MODE) {
                    is_crsf = true;
                    hal_crsf_uart_write(tun_pkt, tun_len);
                    if (ftype == CRSF_FRAMETYPE_RNS_TLM && tun_len >= 6U && tun_pkt[5] == CRSF_RNS_RESP_STATUS) {
                        crsf_rns_status_t st;
                        if (crsf_decode_rns_status_frame(tun_pkt, tun_len, &st)) {
                            char rxh[33];
                            format_hex_bytes(st.tx_dest, 16U, rxh);
                            hal_usb_uart_printf("OTA_CRSF_STATUS_RX mode=%u lock=%u freq=%lu bw_khz=%u sf=%u cr=4/%u peers=%u rtt=%ums rx_dest=%s\r\n",
                                                st.app_mode, st.lock_state,
                                                (unsigned long)st.rnode_freq_hz, (unsigned)st.rnode_bw_khz,
                                                st.rnode_sf, st.rnode_cr, st.peer_count, st.last_ping_rtt_ms, rxh);
                        }
                    } else if (ftype == CRSF_FRAMETYPE_RNS_TLM && tun_len >= 6U && tun_pkt[5] == CRSF_RNS_RESP_ACK) {
                        hal_usb_uart_printf("OTA_CRSF_ACK_RX subcmd=0x%02X status=%u\r\n",
                                            tun_pkt[6], tun_pkt[7]);
                    } else if (ftype == CRSF_FRAMETYPE_FLIGHT_MODE) {
                        char fm_str[32];
                        if (crsf_decode_flight_mode_frame(tun_pkt, tun_len, fm_str, sizeof(fm_str))) {
                            hal_usb_uart_printf("OTA_CRSF_FM_RX fm=\"%s\"\r\n", fm_str);
                        }
                    }
                }
            }

            if (!is_crsf) {
                hal_usb_uart_printf("ELRS_RNS_RX len=%u hops=%u hex=",
                                    tun_len, (tun_len > 1U) ? tun_pkt[1] : 0U);
                uint16_t dump_max = (tun_len < 32U) ? tun_len : 32U;
                for (uint16_t i = 0U; i < dump_max; i++) {
                    hal_usb_uart_printf("%02X", tun_pkt[i]);
                }
                if (tun_len > dump_max) hal_usb_uart_puts("...");
                hal_usb_uart_puts("\r\n");

                inspect_and_act_on_rns_packet(tun_pkt, tun_len, "ELRS_TUNNEL");
                /* Record in relay dedup table so if Drone 1 acts as an airborne relay or bridges
                 * between ELRS DATA tunnel and Reticulum mesh, it increments hops and avoids loops */
                uint8_t relay_copy[RNODE_LORA_MAX_PKT];
                if (tun_len <= sizeof(relay_copy)) {
                    memcpy(relay_copy, tun_pkt, tun_len);
                    if (rns_relay_should_forward(&g_rns_relay, relay_copy, tun_len, now_ms) && !g_relay_pending) {
                        memcpy(g_relay_tx_buf, relay_copy, tun_len);
                        g_relay_tx_len  = (uint8_t)tun_len;
                        g_relay_due_ms  = now_ms + 80U + (uint32_t)(hal_micros() & 0x3FU);
                        g_relay_pending = true;
                    }
                }
            }
        }

        /* 4b. Mesh Control Dead-Man Watchdog (3.0s timeout -> automatic zero-velocity brake + LOITER) */
        if (g_mesh_ctrl_active && (int32_t)(hal_millis() - g_last_mesh_ctrl_ms) >= (int32_t)RNS_MESH_WATCHDOG_MS) {
            g_mesh_ctrl_active = false;
            rns_drone_cmd_t stop_cmd;
            memset(&stop_cmd, 0, sizeof(stop_cmd));
            stop_cmd.type = RNS_DRONE_CMD_LOITER;
            uint8_t mav_frame[64];
            size_t mav_len = rns_build_mavlink_command(&stop_cmd, g_mav_tx_seq++, mav_frame, sizeof(mav_frame));
            if (mav_len > 0U) {
                hal_crsf_uart_write(mav_frame, mav_len);
            }
            hal_usb_uart_printf("RNS_MESH_FAILSAFE action=LOITER timeout_ms=%u\r\n", (unsigned)RNS_MESH_WATCHDOG_MS);
        }

        /* 5. Periodic Reticulum Telemetry Beacon (rnlog.beacon) */
        if (g_beacon_interval_ms > 0U && (now_ms - g_last_beacon_ms) >= g_beacon_interval_ms) {
            g_last_beacon_ms = now_ms;
            uint8_t pkt[RNODE_LORA_MAX_PKT];
            size_t blen = rns_build_telemetry_beacon(&g_fc_tlm, 100U, g_last_rssi_dbm, pkt, sizeof(pkt));
            (void)send_or_tunnel_rns_packet(pkt, (uint16_t)blen);
        }

        /* 6. In Active RX mode, forward CRSF RC channels, wire-free AUX switch triggers & LINK_STATISTICS (0x14) */
        if (g_app_mode == APP_MODE_ELRS_RX && g_crsf_rc_pending) {
            g_crsf_rc_pending = false;
            uint8_t crsf_frame[26];
            crsf_build_rc_channels_frame(g_rc_channels, crsf_frame);
            hal_crsf_uart_write(crsf_frame, sizeof(crsf_frame));

            /* Wire-free RC AUX7 (ch11, idx 10) / AUX8 (ch12, idx 11) switch command triggers:
             * Allows a stock handset with internal ELRS TX to trigger Reticulum Announce or
             * Flight Mode (RNS:dest) telemetry broadcast with zero external wiring. */
            uint8_t aux7_bin = (g_rc_channels[10] < 600U) ? 0U : ((g_rc_channels[10] > 1400U) ? 2U : 1U);
            if (g_last_aux7_bin != 0xFFU && aux7_bin != g_last_aux7_bin && aux7_bin == 2U) {
                (void)send_rns_node_announce();
                char fm_buf[28];
                snprintf(fm_buf, sizeof(fm_buf), "RNS:%02x%02x..%02x%02x %up",
                         g_rns_id.dest_hash[0], g_rns_id.dest_hash[1],
                         g_rns_id.dest_hash[14], g_rns_id.dest_hash[15],
                         (unsigned)g_rns_peers.count);
                uint8_t fm_frame[36];
                size_t fm_len = crsf_build_flight_mode_frame(fm_buf, fm_frame);
                if (fm_len > 0U) {
                    hal_crsf_uart_write(fm_frame, fm_len);
                }
                hal_usb_uart_printf("RC_AUX_TRIGGER ch11=HIGH action=ANNOUNCE fm=\"%s\"\r\n", fm_buf);
            }
            g_last_aux7_bin = aux7_bin;
            (void)g_last_aux8_bin;
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
