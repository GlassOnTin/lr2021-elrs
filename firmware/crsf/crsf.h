/* SPDX-License-Identifier: AGPL-3.0-or-later */
/*
 * CRSF (Crossfire) protocol serializer and parser for Flight Controller (RX mode)
 * and Handset / External Module bay (TX mode).
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CRSF_SYNC_BYTE                  0xC8U
#define CRSF_ADDRESS_HANDSET            0xEAU
#define CRSF_ADDRESS_CRSF_RECEIVER      0xECU
#define CRSF_ADDRESS_CRSF_TRANSMITTER   0xEEU
#define CRSF_ADDRESS_FLIGHT_CONTROLLER  0xC8U
#define CRSF_MAX_FRAME_LEN              64U
#define CRSF_NUM_CHANNELS               16U

#define CRSF_FRAMETYPE_GPS              0x02U
#define CRSF_FRAMETYPE_BATTERY_SENSOR   0x08U
#define CRSF_FRAMETYPE_LINK_STATISTICS  0x14U
#define CRSF_FRAMETYPE_RC_CHANNELS      0x16U
#define CRSF_FRAMETYPE_ATTITUDE         0x1EU
#define CRSF_FRAMETYPE_FLIGHT_MODE      0x21U
#define CRSF_FRAMETYPE_DEVICE_PING      0x28U
#define CRSF_FRAMETYPE_DEVICE_INFO      0x29U
#define CRSF_FRAMETYPE_RNS_CMD          0x7AU
#define CRSF_FRAMETYPE_RNS_TLM          0x7BU

/* EdgeTX Lua <-> LR2021 Reticulum Bridge sub-commands (inside 0x7A / 0x7B extended frames) */
#define CRSF_RNS_SUBCMD_GET_STATUS      0x01U
#define CRSF_RNS_SUBCMD_GET_PEER        0x02U
#define CRSF_RNS_SUBCMD_SET_CFG         0x03U
#define CRSF_RNS_SUBCMD_PAIR_RX         0x04U
#define CRSF_RNS_SUBCMD_SELECT_PEER     0x05U
#define CRSF_RNS_SUBCMD_ACTION          0x06U

#define CRSF_RNS_RESP_STATUS            0x81U
#define CRSF_RNS_RESP_PEER              0x82U
#define CRSF_RNS_RESP_ACK               0x83U

#define CRSF_RNS_ACT_ANNOUNCE           0x01U
#define CRSF_RNS_ACT_PING               0x02U
#define CRSF_RNS_ACT_LOITER             0x03U
#define CRSF_RNS_ACT_RTL                0x04U
#define CRSF_RNS_ACT_LAND               0x05U
#define CRSF_RNS_ACT_ARM                0x06U
#define CRSF_RNS_ACT_DISARM             0x07U
#define CRSF_RNS_ACT_FORM_VEE           0x08U
#define CRSF_RNS_ACT_FORM_LINE          0x09U
#define CRSF_RNS_ACT_FORM_COL           0x0AU
#define CRSF_RNS_ACT_FORM_HOLD          0x0BU

typedef struct {
    uint8_t  tx_dest[16];
    uint8_t  rx_dest[16];
    uint8_t  app_mode;
    uint8_t  lock_state;
    uint8_t  elrs_rate_idx;
    bool     ifac_enabled;
    uint32_t rnode_freq_hz;
    uint16_t rnode_bw_khz;
    uint8_t  rnode_sf;
    uint8_t  rnode_cr;
    int8_t   rnode_txpower;
    uint8_t  peer_count;
    uint8_t  selected_peer;
    uint16_t last_ping_rtt_ms;
    uint16_t tx_msgs;
    uint16_t rx_msgs;
    uint16_t relayed_msgs;
} crsf_rns_status_t;

typedef struct {
    uint8_t  peer_idx;
    uint8_t  total_peers;
    bool     is_selected;
    uint8_t  role;
    uint8_t  dest_hash[16];
    char     name[12];
    uint8_t  hops;
    int8_t   rssi_dbm;
    int8_t   snr_db;
    bool     armed;
    uint8_t  custom_mode;
    uint16_t vbatt_mv;
    uint16_t age_s;
} crsf_rns_peer_info_t;

typedef struct {
    uint8_t uplink_rssi_1;
    uint8_t uplink_rssi_2;
    uint8_t uplink_lq;
    int8_t  uplink_snr;
    uint8_t active_antenna;
    uint8_t rf_mode;
    uint8_t uplink_tx_power;
    uint8_t downlink_rssi;
    uint8_t downlink_lq;
    int8_t  downlink_snr;
} crsf_link_stats_t;

typedef struct {
    uint8_t buf[CRSF_MAX_FRAME_LEN];
    uint8_t pos;
    uint8_t expected_len;
} crsf_parser_t;

/* Compute CRSF CRC8 (polynomial 0xD5) over [type .. payload] */
uint8_t crsf_crc8(const uint8_t *data, size_t len);

/* Build 26-byte CRSF_FRAMETYPE_RC_CHANNELS (0x16) frame from 16x 11-bit channels */
size_t crsf_build_rc_channels_frame(const uint16_t ch[CRSF_NUM_CHANNELS], uint8_t out_frame[26]);

/* Decode 16x 11-bit channels from a validated CRSF_FRAMETYPE_RC_CHANNELS frame */
bool crsf_decode_rc_channels_frame(const uint8_t *frame, size_t frame_len, uint16_t out_ch[CRSF_NUM_CHANNELS]);

/* Build 14-byte CRSF_FRAMETYPE_LINK_STATISTICS (0x14) frame */
size_t crsf_build_link_stats_frame(const crsf_link_stats_t *stats, uint8_t out_frame[14]);

/* Decode CRSF_FRAMETYPE_LINK_STATISTICS frame */
bool crsf_decode_link_stats_frame(const uint8_t *frame, size_t frame_len, crsf_link_stats_t *out_stats);

/* Convert microseconds (988..2012 us) <-> 11-bit CRSF (172..1811) */
uint16_t crsf_us_to_ticks(uint16_t us);
uint16_t crsf_ticks_to_us(uint16_t ticks);

/* Stream parser: feed one byte at a time; returns true when a complete valid CRSF frame is in parser->buf (length = parser->expected_len + 2) */
void crsf_parser_init(crsf_parser_t *parser);
bool crsf_parser_feed_byte(crsf_parser_t *parser, uint8_t byte);

/* Build CRSF_FRAMETYPE_DEVICE_INFO (0x29) response for EdgeTX Lua discovery */
size_t crsf_build_device_info_frame(const char *device_name, uint32_t serial_no, uint8_t out_frame[CRSF_MAX_FRAME_LEN]);

/* Build & decode CRSF_FRAMETYPE_FLIGHT_MODE (0x21) standard telemetry string frame (e.g. "RNS:4cef..c149 1p") */
size_t crsf_build_flight_mode_frame(const char *fm_str, uint8_t out_frame[CRSF_MAX_FRAME_LEN]);
bool   crsf_decode_flight_mode_frame(const uint8_t *frame, size_t frame_len, char *out_str, size_t max_len);

/* Build & decode CRSF_FRAMETYPE_RNS_TLM (0x7B) extended frames for EdgeTX Lua script */
size_t crsf_build_rns_status_frame(const crsf_rns_status_t *st, uint8_t out_frame[CRSF_MAX_FRAME_LEN]);
bool   crsf_decode_rns_status_frame(const uint8_t *frame, size_t frame_len, crsf_rns_status_t *out_st);

size_t crsf_build_rns_peer_frame(const crsf_rns_peer_info_t *pi, uint8_t out_frame[CRSF_MAX_FRAME_LEN]);
bool   crsf_decode_rns_peer_frame(const uint8_t *frame, size_t frame_len, crsf_rns_peer_info_t *out_pi);

typedef struct {
    uint32_t freq_hz;
    uint32_t bw_hz;
    uint8_t  sf;
    uint8_t  cr;
    int8_t   txpower_dbm;
    bool     ifac_enabled;
} crsf_rns_cfg_cmd_t;

size_t crsf_build_rns_ack_frame(uint8_t subcmd, uint8_t status_code, uint8_t out_frame[CRSF_MAX_FRAME_LEN]);
bool   crsf_decode_rns_ack_frame(const uint8_t *frame, size_t frame_len, uint8_t *out_subcmd, uint8_t *out_status_code);

/* Build & decode CRSF_FRAMETYPE_RNS_CMD (0x7A) extended command frame (Handset -> TX Module / OTA RX) */
size_t crsf_build_rns_cmd_frame(uint8_t subcmd, const uint8_t *payload, size_t pay_len, uint8_t out_frame[CRSF_MAX_FRAME_LEN]);
bool   crsf_decode_rns_cmd_frame(const uint8_t *frame,
                                 size_t frame_len,
                                 uint8_t *out_dest_addr,
                                 uint8_t *out_orig_addr,
                                 uint8_t *out_subcmd,
                                 const uint8_t **out_payload,
                                 size_t *out_pay_len);
bool   crsf_decode_rns_set_cfg_frame(const uint8_t *frame, size_t frame_len, crsf_rns_cfg_cmd_t *out_cfg);

#ifdef __cplusplus
}
#endif
