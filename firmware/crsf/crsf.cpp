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
        if (byte == CRSF_SYNC_BYTE || byte == CRSF_ADDRESS_HANDSET ||
            byte == CRSF_ADDRESS_CRSF_RECEIVER || byte == CRSF_ADDRESS_CRSF_TRANSMITTER) {
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

static void write_be16(uint8_t *b, uint16_t v) {
    b[0] = (uint8_t)(v >> 8U);
    b[1] = (uint8_t)(v & 0xFFU);
}

static uint16_t read_be16(const uint8_t *b) {
    return (uint16_t)(((uint16_t)b[0] << 8U) | b[1]);
}

static void write_be32(uint8_t *b, uint32_t v) {
    b[0] = (uint8_t)(v >> 24U);
    b[1] = (uint8_t)((v >> 16U) & 0xFFU);
    b[2] = (uint8_t)((v >> 8U) & 0xFFU);
    b[3] = (uint8_t)(v & 0xFFU);
}

static uint32_t read_be32(const uint8_t *b) {
    return ((uint32_t)b[0] << 24U) |
           ((uint32_t)b[1] << 16U) |
           ((uint32_t)b[2] << 8U)  |
           (uint32_t)b[3];
}

size_t crsf_build_device_info_frame(const char *device_name, uint32_t serial_no, uint8_t out_frame[CRSF_MAX_FRAME_LEN]) {
    if (device_name == NULL || out_frame == NULL) return 0U;
    size_t nlen = strlen(device_name);
    if (nlen > 24U) nlen = 24U;

    size_t idx = 0U;
    out_frame[idx++] = CRSF_ADDRESS_HANDSET;
    out_frame[idx++] = 0U; /* length placeholder */
    out_frame[idx++] = CRSF_FRAMETYPE_DEVICE_INFO;
    out_frame[idx++] = CRSF_ADDRESS_HANDSET;
    out_frame[idx++] = CRSF_ADDRESS_CRSF_TRANSMITTER;
    memcpy(&out_frame[idx], device_name, nlen);
    idx += nlen;
    out_frame[idx++] = '\0';
    write_be32(&out_frame[idx], serial_no); idx += 4U;
    write_be32(&out_frame[idx], 0x00011800UL); idx += 4U; /* HW ver: LR2021 0x0118 */
    write_be32(&out_frame[idx], 0x00013F00UL); idx += 4U; /* SW ver: 1.63.0 */
    out_frame[idx++] = 0U; /* parameter count */
    out_frame[idx++] = 0U; /* parameter protocol version */

    out_frame[1] = (uint8_t)(idx - 1U); /* type..payload + crc */
    out_frame[idx] = crsf_crc8(&out_frame[2], idx - 2U);
    return idx + 1U;
}

size_t crsf_build_flight_mode_frame(const char *fm_str, uint8_t out_frame[CRSF_MAX_FRAME_LEN]) {
    if (fm_str == NULL || out_frame == NULL) return 0U;
    size_t slen = strlen(fm_str);
    if (slen > 28U) slen = 28U;

    out_frame[0] = CRSF_SYNC_BYTE;
    out_frame[1] = (uint8_t)(slen + 3U); /* type(1) + string(slen) + NUL(1) + crc(1) */
    out_frame[2] = CRSF_FRAMETYPE_FLIGHT_MODE;
    memcpy(&out_frame[3], fm_str, slen);
    out_frame[3U + slen] = '\0';
    out_frame[4U + slen] = crsf_crc8(&out_frame[2], slen + 2U);
    return slen + 5U;
}

bool crsf_decode_flight_mode_frame(const uint8_t *frame, size_t frame_len, char *out_str, size_t max_len) {
    if (frame == NULL || out_str == NULL || max_len == 0U || frame_len < 5U) return false;
    uint8_t flen = frame[1];
    if ((size_t)(flen + 2U) > frame_len || frame[2] != CRSF_FRAMETYPE_FLIGHT_MODE) {
        return false;
    }
    if (crsf_crc8(&frame[2], (size_t)(flen - 1U)) != frame[flen + 1U]) {
        return false;
    }
    size_t pay_len = (size_t)(flen - 2U);
    size_t copy_len = (pay_len < (max_len - 1U)) ? pay_len : (max_len - 1U);
    memcpy(out_str, &frame[3], copy_len);
    out_str[copy_len] = '\0';
    return true;
}

size_t crsf_build_rns_status_frame(const crsf_rns_status_t *st, uint8_t out_frame[CRSF_MAX_FRAME_LEN]) {
    if (st == NULL || out_frame == NULL) return 0U;
    /* Frame layout (62 bytes total):
     * [0]     0xEA (CRSF_ADDRESS_HANDSET)
     * [1]     60   (type + ext_hdr + subcmd + 55B payload + crc)
     * [2]     0x7B (CRSF_FRAMETYPE_RNS_TLM)
     * [3]     0xEA (dest = Handset)
     * [4]     0xEE (orig = TX Module)
     * [5]     0x81 (CRSF_RNS_RESP_STATUS)
     * [6..21] tx_dest[16]
     * [22..37]rx_dest[16]
     * [38]    app_mode
     * [39]    lock_state
     * [40]    elrs_rate_idx
     * [41]    ifac_enabled
     * [42..45]rnode_freq_hz (BE32)
     * [46..47]rnode_bw_khz  (BE16)
     * [48]    rnode_sf
     * [49]    rnode_cr
     * [50]    rnode_txpower
     * [51]    peer_count
     * [52]    selected_peer
     * [53..54]last_ping_rtt_ms (BE16)
     * [55..56]tx_msgs (BE16)
     * [57..58]rx_msgs (BE16)
     * [59..60]relayed_msgs (BE16)
     * [61]    crc8
     */
    memset(out_frame, 0, 62U);
    out_frame[0] = CRSF_ADDRESS_HANDSET;
    out_frame[1] = 60U;
    out_frame[2] = CRSF_FRAMETYPE_RNS_TLM;
    out_frame[3] = CRSF_ADDRESS_HANDSET;
    out_frame[4] = CRSF_ADDRESS_CRSF_TRANSMITTER;
    out_frame[5] = CRSF_RNS_RESP_STATUS;

    memcpy(&out_frame[6], st->tx_dest, 16U);
    memcpy(&out_frame[22], st->rx_dest, 16U);
    out_frame[38] = st->app_mode;
    out_frame[39] = st->lock_state;
    out_frame[40] = st->elrs_rate_idx;
    out_frame[41] = st->ifac_enabled ? 1U : 0U;
    write_be32(&out_frame[42], st->rnode_freq_hz);
    write_be16(&out_frame[46], st->rnode_bw_khz);
    out_frame[48] = st->rnode_sf;
    out_frame[49] = st->rnode_cr;
    out_frame[50] = (uint8_t)st->rnode_txpower;
    out_frame[51] = st->peer_count;
    out_frame[52] = st->selected_peer;
    write_be16(&out_frame[53], st->last_ping_rtt_ms);
    write_be16(&out_frame[55], st->tx_msgs);
    write_be16(&out_frame[57], st->rx_msgs);
    write_be16(&out_frame[59], st->relayed_msgs);
    out_frame[61] = crsf_crc8(&out_frame[2], 59U);
    return 62U;
}

bool crsf_decode_rns_status_frame(const uint8_t *frame, size_t frame_len, crsf_rns_status_t *out_st) {
    if (frame == NULL || out_st == NULL || frame_len < 62U) return false;
    if (frame[1] != 60U || frame[2] != CRSF_FRAMETYPE_RNS_TLM || frame[5] != CRSF_RNS_RESP_STATUS) {
        return false;
    }
    if (crsf_crc8(&frame[2], 59U) != frame[61]) {
        return false;
    }
    memset(out_st, 0, sizeof(*out_st));
    memcpy(out_st->tx_dest, &frame[6], 16U);
    memcpy(out_st->rx_dest, &frame[22], 16U);
    out_st->app_mode         = frame[38];
    out_st->lock_state       = frame[39];
    out_st->elrs_rate_idx    = frame[40];
    out_st->ifac_enabled     = (frame[41] != 0U);
    out_st->rnode_freq_hz    = read_be32(&frame[42]);
    out_st->rnode_bw_khz     = read_be16(&frame[46]);
    out_st->rnode_sf         = frame[48];
    out_st->rnode_cr         = frame[49];
    out_st->rnode_txpower    = (int8_t)frame[50];
    out_st->peer_count       = frame[51];
    out_st->selected_peer    = frame[52];
    out_st->last_ping_rtt_ms = read_be16(&frame[53]);
    out_st->tx_msgs          = read_be16(&frame[55]);
    out_st->rx_msgs          = read_be16(&frame[57]);
    out_st->relayed_msgs     = read_be16(&frame[59]);
    return true;
}

size_t crsf_build_rns_peer_frame(const crsf_rns_peer_info_t *pi, uint8_t out_frame[CRSF_MAX_FRAME_LEN]) {
    if (pi == NULL || out_frame == NULL) return 0U;
    /* Frame layout (48 bytes total):
     * [0]     0xEA
     * [1]     46
     * [2]     0x7B (CRSF_FRAMETYPE_RNS_TLM)
     * [3]     0xEA
     * [4]     0xEE
     * [5]     0x82 (CRSF_RNS_RESP_PEER)
     * [6]     peer_idx
     * [7]     total_peers
     * [8]     is_selected
     * [9]     role
     * [10..25]dest_hash[16]
     * [26..37]name[12]
     * [38]    hops
     * [39]    rssi_dbm
     * [40]    snr_db
     * [41]    armed
     * [42]    custom_mode
     * [43..44]vbatt_mv (BE16)
     * [45..46]age_s    (BE16)
     * [47]    crc8
     */
    memset(out_frame, 0, 48U);
    out_frame[0] = CRSF_ADDRESS_HANDSET;
    out_frame[1] = 46U;
    out_frame[2] = CRSF_FRAMETYPE_RNS_TLM;
    out_frame[3] = CRSF_ADDRESS_HANDSET;
    out_frame[4] = CRSF_ADDRESS_CRSF_TRANSMITTER;
    out_frame[5] = CRSF_RNS_RESP_PEER;
    out_frame[6] = pi->peer_idx;
    out_frame[7] = pi->total_peers;
    out_frame[8] = pi->is_selected ? 1U : 0U;
    out_frame[9] = pi->role;
    memcpy(&out_frame[10], pi->dest_hash, 16U);
    memcpy(&out_frame[26], pi->name, 12U);
    out_frame[38] = pi->hops;
    out_frame[39] = (uint8_t)pi->rssi_dbm;
    out_frame[40] = (uint8_t)pi->snr_db;
    out_frame[41] = pi->armed ? 1U : 0U;
    out_frame[42] = pi->custom_mode;
    write_be16(&out_frame[43], pi->vbatt_mv);
    write_be16(&out_frame[45], pi->age_s);
    out_frame[47] = crsf_crc8(&out_frame[2], 45U);
    return 48U;
}

bool crsf_decode_rns_peer_frame(const uint8_t *frame, size_t frame_len, crsf_rns_peer_info_t *out_pi) {
    if (frame == NULL || out_pi == NULL || frame_len < 48U) return false;
    if (frame[1] != 46U || frame[2] != CRSF_FRAMETYPE_RNS_TLM || frame[5] != CRSF_RNS_RESP_PEER) {
        return false;
    }
    if (crsf_crc8(&frame[2], 45U) != frame[47]) {
        return false;
    }
    memset(out_pi, 0, sizeof(*out_pi));
    out_pi->peer_idx    = frame[6];
    out_pi->total_peers = frame[7];
    out_pi->is_selected = (frame[8] != 0U);
    out_pi->role        = frame[9];
    memcpy(out_pi->dest_hash, &frame[10], 16U);
    memcpy(out_pi->name, &frame[26], 12U);
    out_pi->hops        = frame[38];
    out_pi->rssi_dbm    = (int8_t)frame[39];
    out_pi->snr_db      = (int8_t)frame[40];
    out_pi->armed       = (frame[41] != 0U);
    out_pi->custom_mode = frame[42];
    out_pi->vbatt_mv    = read_be16(&frame[43]);
    out_pi->age_s       = read_be16(&frame[45]);
    return true;
}

size_t crsf_build_rns_ack_frame(uint8_t subcmd, uint8_t status_code, uint8_t out_frame[CRSF_MAX_FRAME_LEN]) {
    if (out_frame == NULL) return 0U;
    out_frame[0] = CRSF_ADDRESS_HANDSET;
    out_frame[1] = 7U;
    out_frame[2] = CRSF_FRAMETYPE_RNS_TLM;
    out_frame[3] = CRSF_ADDRESS_HANDSET;
    out_frame[4] = CRSF_ADDRESS_CRSF_TRANSMITTER;
    out_frame[5] = CRSF_RNS_RESP_ACK;
    out_frame[6] = subcmd;
    out_frame[7] = status_code;
    out_frame[8] = crsf_crc8(&out_frame[2], 6U);
    return 9U;
}

bool crsf_decode_rns_ack_frame(const uint8_t *frame, size_t frame_len, uint8_t *out_subcmd, uint8_t *out_status_code) {
    if (frame == NULL || frame_len < 9U) return false;
    if (frame[1] != 7U || frame[2] != CRSF_FRAMETYPE_RNS_TLM || frame[5] != CRSF_RNS_RESP_ACK) {
        return false;
    }
    if (crsf_crc8(&frame[2], 6U) != frame[8]) {
        return false;
    }
    if (out_subcmd != NULL) *out_subcmd = frame[6];
    if (out_status_code != NULL) *out_status_code = frame[7];
    return true;
}

size_t crsf_build_rns_cmd_frame(uint8_t subcmd, const uint8_t *payload, size_t pay_len, uint8_t out_frame[CRSF_MAX_FRAME_LEN]) {
    if (out_frame == NULL || (pay_len + 7U) > CRSF_MAX_FRAME_LEN) return 0U;
    out_frame[0] = CRSF_ADDRESS_CRSF_TRANSMITTER;
    out_frame[1] = (uint8_t)(5U + pay_len);
    out_frame[2] = CRSF_FRAMETYPE_RNS_CMD;
    out_frame[3] = CRSF_ADDRESS_CRSF_TRANSMITTER;
    out_frame[4] = CRSF_ADDRESS_HANDSET;
    out_frame[5] = subcmd;
    if (payload != NULL && pay_len > 0U) {
        memcpy(&out_frame[6], payload, pay_len);
    }
    out_frame[6U + pay_len] = crsf_crc8(&out_frame[2], 4U + pay_len);
    return 7U + pay_len;
}

bool crsf_decode_rns_cmd_frame(const uint8_t *frame,
                               size_t frame_len,
                               uint8_t *out_dest_addr,
                               uint8_t *out_orig_addr,
                               uint8_t *out_subcmd,
                               const uint8_t **out_payload,
                               size_t *out_pay_len) {
    if (frame == NULL || frame_len < 7U) return false;
    uint8_t flen = frame[1];
    if ((size_t)(flen + 2U) > frame_len || flen < 5U || frame[2] != CRSF_FRAMETYPE_RNS_CMD) {
        return false;
    }
    if (crsf_crc8(&frame[2], (size_t)(flen - 1U)) != frame[flen + 1U]) {
        return false;
    }
    if (out_dest_addr != NULL) *out_dest_addr = frame[3];
    if (out_orig_addr != NULL) *out_orig_addr = frame[4];
    if (out_subcmd != NULL)    *out_subcmd    = frame[5];
    size_t plen = (size_t)(flen - 5U);
    if (out_payload != NULL)   *out_payload   = (plen > 0U) ? &frame[6] : NULL;
    if (out_pay_len != NULL)   *out_pay_len   = plen;
    return true;
}

bool crsf_decode_rns_set_cfg_frame(const uint8_t *frame, size_t frame_len, crsf_rns_cfg_cmd_t *out_cfg) {
    if (out_cfg == NULL) return false;
    uint8_t subcmd = 0U;
    const uint8_t *pay = NULL;
    size_t pay_len = 0U;
    if (!crsf_decode_rns_cmd_frame(frame, frame_len, NULL, NULL, &subcmd, &pay, &pay_len)) {
        return false;
    }
    if (subcmd != CRSF_RNS_SUBCMD_SET_CFG || pay == NULL || pay_len < 12U) {
        return false;
    }
    memset(out_cfg, 0, sizeof(*out_cfg));
    out_cfg->freq_hz      = read_be32(&pay[0]);
    out_cfg->bw_hz        = read_be32(&pay[4]);
    out_cfg->sf           = pay[8];
    out_cfg->cr           = pay[9];
    out_cfg->txpower_dbm  = (int8_t)pay[10];
    out_cfg->ifac_enabled = (pay[11] != 0U);
    return true;
}
