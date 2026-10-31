/* SPDX-License-Identifier: AGPL-3.0-or-later */

#include "crsf.h"
#include <string.h>

#define CRSF_CRC_POLY 0xD5U

uint8_t crsf_crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0U;
    for (size_t i = 0U; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0U; b < 8U; b++) {
            if ((crc & 0x80U) != 0U) {
                crc = (uint8_t)((crc << 1U) ^ CRSF_CRC_POLY);
            } else {
                crc = (uint8_t)(crc << 1U);
            }
        }
    }
    return crc;
}

size_t crsf_build_rc_channels_frame(const uint16_t ch[CRSF_NUM_CHANNELS], uint8_t out_frame[26]) {
    memset(out_frame, 0, 26U);
    out_frame[0] = CRSF_SYNC_BYTE;
    out_frame[1] = 24U; /* type (1) + payload (22) + crc (1) */
    out_frame[2] = CRSF_FRAMETYPE_RC_CHANNELS;

    uint32_t bit_buf   = 0U;
    uint8_t  bit_count = 0U;
    uint8_t  byte_idx  = 3U;

    for (uint8_t i = 0U; i < CRSF_NUM_CHANNELS; i++) {
        bit_buf |= ((uint32_t)(ch[i] & 0x07FFU)) << bit_count;
        bit_count = (uint8_t)(bit_count + 11U);
        while (bit_count >= 8U) {
            out_frame[byte_idx++] = (uint8_t)(bit_buf & 0xFFU);
            bit_buf >>= 8U;
            bit_count = (uint8_t)(bit_count - 8U);
        }
    }

    out_frame[25] = crsf_crc8(&out_frame[2], 23U);
    return 26U;
}

bool crsf_decode_rc_channels_frame(const uint8_t *frame, size_t frame_len, uint16_t out_ch[CRSF_NUM_CHANNELS]) {
    if (frame_len < 26U || frame[1] != 24U || frame[2] != CRSF_FRAMETYPE_RC_CHANNELS) {
        return false;
    }
    if (crsf_crc8(&frame[2], 23U) != frame[25]) {
        return false;
    }

    uint32_t bit_buf   = 0U;
    uint8_t  bit_count = 0U;
    uint8_t  byte_idx  = 3U;

    for (uint8_t i = 0U; i < CRSF_NUM_CHANNELS; i++) {
        while (bit_count < 11U) {
            bit_buf |= ((uint32_t)frame[byte_idx++]) << bit_count;
            bit_count = (uint8_t)(bit_count + 8U);
        }
        out_ch[i] = (uint16_t)(bit_buf & 0x07FFU);
        bit_buf >>= 11U;
        bit_count = (uint8_t)(bit_count - 11U);
    }
    return true;
}

size_t crsf_build_link_stats_frame(const crsf_link_stats_t *stats, uint8_t out_frame[14]) {
    out_frame[0]  = CRSF_SYNC_BYTE;
    out_frame[1]  = 12U; /* type (1) + payload (10) + crc (1) */
    out_frame[2]  = CRSF_FRAMETYPE_LINK_STATISTICS;
    out_frame[3]  = stats->uplink_rssi_1;
    out_frame[4]  = stats->uplink_rssi_2;
    out_frame[5]  = stats->uplink_lq;
    out_frame[6]  = (uint8_t)stats->uplink_snr;
    out_frame[7]  = stats->active_antenna;
    out_frame[8]  = stats->rf_mode;
    out_frame[9]  = stats->uplink_tx_power;
    out_frame[10] = stats->downlink_rssi;
    out_frame[11] = stats->downlink_lq;
    out_frame[12] = (uint8_t)stats->downlink_snr;
    out_frame[13] = crsf_crc8(&out_frame[2], 11U);
    return 14U;
}

bool crsf_decode_link_stats_frame(const uint8_t *frame, size_t frame_len, crsf_link_stats_t *out_stats) {
    if (frame_len < 14U || frame[1] != 12U || frame[2] != CRSF_FRAMETYPE_LINK_STATISTICS) {
        return false;
    }
    if (crsf_crc8(&frame[2], 11U) != frame[13]) {
        return false;
    }
    out_stats->uplink_rssi_1   = frame[3];
    out_stats->uplink_rssi_2   = frame[4];
    out_stats->uplink_lq       = frame[5];
    out_stats->uplink_snr      = (int8_t)frame[6];
    out_stats->active_antenna  = frame[7];
    out_stats->rf_mode         = frame[8];
    out_stats->uplink_tx_power = frame[9];
    out_stats->downlink_rssi   = frame[10];
    out_stats->downlink_lq     = frame[11];
    out_stats->downlink_snr    = (int8_t)frame[12];
    return true;
}

uint16_t crsf_us_to_ticks(uint16_t us) {
    if (us < 988U) us = 988U;
    if (us > 2012U) us = 2012U;
    int32_t res = (((int32_t)(us - 988U) * (1811 - 172) * 2) / (2012 - 988) + 172 * 2 + 1) / 2;
    return (uint16_t)res;
}

uint16_t crsf_ticks_to_us(uint16_t ticks) {
    if (ticks < 172U) ticks = 172U;
    if (ticks > 1811U) ticks = 1811U;
    int32_t res = (((int32_t)(ticks - 172U) * (2012 - 988) * 2) / (1811 - 172) + 988 * 2 + 1) / 2;
    return (uint16_t)res;
}

void crsf_parser_init(crsf_parser_t *parser) {
    memset(parser, 0, sizeof(*parser));
}

bool crsf_parser_feed_byte(crsf_parser_t *parser, uint8_t byte) {
    if (parser->pos == 0U) {
        if (byte == CRSF_SYNC_BYTE || byte == CRSF_ADDRESS_HANDSET || byte == CRSF_ADDRESS_CRSF_TRANSMITTER) {
            parser->buf[0] = byte;
            parser->pos = 1U;
        }
        return false;
    }
    if (parser->pos == 1U) {
        if (byte < 2U || byte > (CRSF_MAX_FRAME_LEN - 2U)) {
            parser->pos = 0U;
            return false;
        }
        parser->expected_len = byte;
        parser->buf[1] = byte;
        parser->pos = 2U;
        return false;
    }

    parser->buf[parser->pos++] = byte;
    uint8_t total_len = (uint8_t)(parser->expected_len + 2U);
    if (parser->pos >= total_len) {
        parser->pos = 0U;
        uint8_t calc_crc = crsf_crc8(&parser->buf[2], (size_t)(parser->expected_len - 1U));
        uint8_t in_crc   = parser->buf[total_len - 1U];
        return calc_crc == in_crc;
    }
    return false;
}
