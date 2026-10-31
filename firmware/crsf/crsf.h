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

#ifdef __cplusplus
}
#endif
