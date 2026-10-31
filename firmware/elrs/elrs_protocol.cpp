/* SPDX-License-Identifier: AGPL-3.0-or-later */
/*
 * Portable ExpressLRS v3/v4 Over-The-Air (OTA) protocol, UID MD5 derivation,
 * GenerableCRC (14-bit & 16-bit), and FHSS sequence generator for LR2021.
 *
 * Adapted from ExpressLRS (GPL-3.0-or-later, src/lib/OTA, src/lib/FHSS, src/lib/CRC).
 */

#include "elrs_protocol.h"
#include "../radio/LR2021_Regs.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CRSF_CHANNEL_VALUE_STD_MIN  172U
#define CRSF_CHANNEL_VALUE_1000     191U
#define CRSF_CHANNEL_VALUE_MID      992U
#define CRSF_CHANNEL_VALUE_2000     1792U
#define CRSF_CHANNEL_VALUE_STD_MAX  1811U

#define ELRS_CRC14_POLY 0x2E57U
#define ELRS_CRC16_POLY 0x3D65U

/* ============================================================================
 * 1. RFC 1321 MD5 for ExpressLRS Binding Phrase -> 6-byte UID
 *    Input string format: -DMY_BINDING_PHRASE="<phrase>"
 * ============================================================================ */

static const uint32_t k_md5_k[64] = {
    0xd76aa478UL, 0xe8c7b756UL, 0x242070dbUL, 0xc1bdceeeUL,
    0xf57c0fafUL, 0x4787c62aUL, 0xa8304613UL, 0xfd469501UL,
    0x698098d8UL, 0x8b44f7afUL, 0xffff5bb1UL, 0x895cd7beUL,
    0x6b901122UL, 0xfd987193UL, 0xa679438eUL, 0x49b40821UL,
    0xf61e2562UL, 0xc040b340UL, 0x265e5a51UL, 0xe9b6c7aaUL,
    0xd62f105dUL, 0x02441453UL, 0xd8a1e681UL, 0xe7d3fbc8UL,
    0x21e1cde6UL, 0xc33707d6UL, 0xf4d50d87UL, 0x455a14edUL,
    0xa9e3e905UL, 0xfcefa3f8UL, 0x676f02d9UL, 0x8d2a4c8aUL,
    0xfffa3942UL, 0x8771f681UL, 0x6d9d6122UL, 0xfde5380cUL,
    0xa4beea44UL, 0x4bdecfa9UL, 0xf6bb4b60UL, 0xbebfbc70UL,
    0x289b7ec6UL, 0xeaa127faUL, 0xd4ef3085UL, 0x04881d05UL,
    0xd9d4d039UL, 0xe6db99e5UL, 0x1fa27cf8UL, 0xc4ac5665UL,
    0xf4292244UL, 0x432aff97UL, 0xab9423a7UL, 0xfc93a039UL,
    0x655b59c3UL, 0x8f0ccc92UL, 0xffeff47dUL, 0x85845dd1UL,
    0x6fa87e4fUL, 0xfe2ce6e0UL, 0xa3014314UL, 0x4e0811a1UL,
    0xf7537e82UL, 0xbd3af235UL, 0x2ad7d2bbUL, 0xeb86d391UL
};

static const uint8_t k_md5_s[64] = {
    7, 12, 17, 22,  7, 12, 17, 22,  7, 12, 17, 22,  7, 12, 17, 22,
    5,  9, 14, 20,  5,  9, 14, 20,  5,  9, 14, 20,  5,  9, 14, 20,
    4, 11, 16, 23,  4, 11, 16, 23,  4, 11, 16, 23,  4, 11, 16, 23,
    6, 10, 15, 21,  6, 10, 15, 21,  6, 10, 15, 21,  6, 10, 15, 21
};

static inline uint32_t rotl32(uint32_t x, uint8_t n) {
    return (x << n) | (x >> (32U - n));
}

static void md5_compute(const uint8_t *msg, size_t len, uint8_t digest[16]) {
    uint32_t a0 = 0x67452301UL;
    uint32_t b0 = 0xefcdab89UL;
    uint32_t c0 = 0x98badcfeUL;
    uint32_t d0 = 0x10325476UL;

    size_t new_len = len + 1U;
    while ((new_len % 64U) != 56U) {
        new_len++;
    }

    uint8_t block[256];
    if (new_len + 8U > sizeof(block)) {
        memset(digest, 0, 16);
        return;
    }
    memset(block, 0, new_len + 8U);
    memcpy(block, msg, len);
    block[len] = 0x80U;

    uint64_t bits_len = (uint64_t)len * 8ULL;
    for (uint8_t i = 0; i < 8U; i++) {
        block[new_len + i] = (uint8_t)((bits_len >> (8U * i)) & 0xFFU);
    }

    for (size_t offset = 0; offset < new_len + 8U; offset += 64U) {
        uint32_t M[16];
        for (uint8_t i = 0; i < 16U; i++) {
            const uint8_t *p = &block[offset + i * 4U];
            M[i] = (uint32_t)p[0] |
                   ((uint32_t)p[1] << 8U) |
                   ((uint32_t)p[2] << 16U) |
                   ((uint32_t)p[3] << 24U);
        }

        uint32_t A = a0, B = b0, C = c0, D = d0;
        for (uint8_t i = 0; i < 64U; i++) {
            uint32_t F, g;
            if (i < 16U) {
                F = (B & C) | ((~B) & D);
                g = i;
            } else if (i < 32U) {
                F = (D & B) | ((~D) & C);
                g = (5U * i + 1U) % 16U;
            } else if (i < 48U) {
                F = B ^ C ^ D;
                g = (3U * i + 5U) % 16U;
            } else {
                F = C ^ (B | (~D));
                g = (7U * i) % 16U;
            }
            F = F + A + k_md5_k[i] + M[g];
            A = D;
            D = C;
            C = B;
            B = B + rotl32(F, k_md5_s[i]);
        }
        a0 += A;
        b0 += B;
        c0 += C;
        d0 += D;
    }

    uint32_t words[4] = {a0, b0, c0, d0};
    for (uint8_t i = 0; i < 4U; i++) {
        digest[i * 4U + 0U] = (uint8_t)(words[i] & 0xFFU);
        digest[i * 4U + 1U] = (uint8_t)((words[i] >> 8U) & 0xFFU);
        digest[i * 4U + 2U] = (uint8_t)((words[i] >> 16U) & 0xFFU);
        digest[i * 4U + 3U] = (uint8_t)((words[i] >> 24U) & 0xFFU);
    }
}

void elrs_uid_from_bind_phrase(const char *phrase, uint8_t out_uid[ELRS_UID_LEN]) {
    char buf[192];
    int n = snprintf(buf, sizeof(buf), "-DMY_BINDING_PHRASE=\"%s\"", phrase ? phrase : "");
    if (n < 0) {
        memset(out_uid, 0, ELRS_UID_LEN);
        return;
    }
    uint8_t digest[16];
    md5_compute((const uint8_t *)buf, (size_t)n, digest);
    memcpy(out_uid, digest, ELRS_UID_LEN);
}

bool elrs_uid_parse_string(const char *str, uint8_t out_uid[ELRS_UID_LEN]) {
    if (str == NULL) {
        return false;
    }
    /* Check if comma-separated decimal or colon-separated hex or 12 hex digits */
    if (strchr(str, ',') != NULL) {
        unsigned int b[6];
        if (sscanf(str, "%u,%u,%u,%u,%u,%u", &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) == 6) {
            for (int i = 0; i < 6; i++) {
                out_uid[i] = (uint8_t)(b[i] & 0xFFU);
            }
            return true;
        }
        return false;
    }
    if (strchr(str, ':') != NULL) {
        unsigned int b[6];
        if (sscanf(str, "%x:%x:%x:%x:%x:%x", &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) == 6) {
            for (int i = 0; i < 6; i++) {
                out_uid[i] = (uint8_t)(b[i] & 0xFFU);
            }
            return true;
        }
        return false;
    }
    if (strlen(str) == 12U) {
        for (int i = 0; i < 6; i++) {
            char hex[3] = {str[i * 2], str[i * 2 + 1], '\0'};
            if (!isxdigit((unsigned char)hex[0]) || !isxdigit((unsigned char)hex[1])) {
                return false;
            }
            out_uid[i] = (uint8_t)strtoul(hex, NULL, 16);
        }
        return true;
    }
    return false;
}

/* ============================================================================
 * 2. ExpressLRS Crc2Byte (14-bit & 16-bit) & OTA4 / OTA8 Packing
 * ============================================================================ */

static void crc2byte_init(elrs_ota_state_t *st, uint8_t bits, uint16_t poly) {
    st->crc_bits = bits;
    st->crc_poly = poly;
    st->crc_mask = (uint16_t)((1UL << bits) - 1UL);
    uint16_t highbit = (uint16_t)(1UL << (bits - 1U));
    for (uint16_t i = 0; i < 256U; i++) {
        uint16_t crc = (uint16_t)(i << (bits - 8U));
        for (uint8_t j = 0; j < 8U; j++) {
            crc = (uint16_t)((crc << 1U) ^ ((crc & highbit) ? poly : 0U));
        }
        st->crc_table[i] = crc;
    }
}

static uint16_t crc2byte_calc(const elrs_ota_state_t *st, const uint8_t *data, uint8_t len, uint16_t crc) {
    while (len--) {
        uint8_t idx = (uint8_t)(((crc >> (st->crc_bits - 8U)) ^ (uint16_t)(*data++)) & 0xFFU);
        crc = (uint16_t)((crc << 8U) ^ st->crc_table[idx]);
    }
    return (uint16_t)(crc & st->crc_mask);
}

void elrs_ota_set_mode(elrs_ota_state_t *st, uint8_t packet_size, elrs_switch_mode_t switch_mode) {
    st->is_full_res = (packet_size == ELRS_OTA8_PACKET_SIZE);
    st->switch_mode = switch_mode;
    if (st->is_full_res) {
        crc2byte_init(st, 16U, ELRS_CRC16_POLY);
    } else {
        crc2byte_init(st, 14U, ELRS_CRC14_POLY);
    }
}

void elrs_ota_init(elrs_ota_state_t *st,
                   const uint8_t uid[ELRS_UID_LEN],
                   uint8_t packet_size,
                   elrs_switch_mode_t switch_mode) {
    memset(st, 0, sizeof(*st));
    memcpy(st->uid, uid, ELRS_UID_LEN);
    st->crc_init = (uint16_t)((((uint16_t)uid[4] << 8U) | uid[5]) ^ ((uint16_t)ELRS_OTA_VERSION_ID << 8U));
    st->uid_seed = ((uint32_t)uid[2] << 24U) |
                   ((uint32_t)uid[3] << 16U) |
                   ((uint32_t)uid[4] << 8U)  |
                   ((uint32_t)(uid[5] ^ ELRS_OTA_VERSION_ID));
    st->uplink_tx_power_enum = 1U;
    elrs_ota_set_mode(st, packet_size, switch_mode);
}

void elrs_ota_generate_crc(const elrs_ota_state_t *st, uint8_t *pkt, uint8_t ota_nonce) {
    uint8_t pkt_type = pkt[0] & 0x03U;
    uint16_t nonce_val = (pkt_type == ELRS_PKT_TYPE_SYNC) ? 0U : (uint16_t)ota_nonce;
    if (st->is_full_res) {
        uint16_t crc = crc2byte_calc(st, pkt, 11U, (uint16_t)(st->crc_init ^ nonce_val));
        pkt[11] = (uint8_t)(crc & 0xFFU);
        pkt[12] = (uint8_t)(crc >> 8U);
    } else {
        pkt[0] &= 0x03U; /* Clear crcHigh bits before computing */
        uint16_t crc = crc2byte_calc(st, pkt, 7U, (uint16_t)(st->crc_init ^ nonce_val));
        pkt[0] |= (uint8_t)(((crc >> 8U) & 0x3FU) << 2U);
        pkt[7]  = (uint8_t)(crc & 0xFFU);
    }
}

bool elrs_ota_validate_crc(const elrs_ota_state_t *st, const uint8_t *pkt, uint8_t ota_nonce) {
    uint8_t pkt_type = pkt[0] & 0x03U;
    uint16_t nonce_val = (pkt_type == ELRS_PKT_TYPE_SYNC) ? 0U : (uint16_t)ota_nonce;
    if (st->is_full_res) {
        uint16_t in_crc = (uint16_t)pkt[11] | ((uint16_t)pkt[12] << 8U);
        uint16_t calc   = crc2byte_calc(st, pkt, 11U, (uint16_t)(st->crc_init ^ nonce_val));
        return in_crc == calc;
    } else {
        uint8_t tmp[7];
        memcpy(tmp, pkt, 7U);
        uint16_t in_crc = (((uint16_t)(pkt[0] >> 2U) & 0x3FU) << 8U) | pkt[7];
        tmp[0] &= 0x03U;
        uint16_t calc = crc2byte_calc(st, tmp, 7U, (uint16_t)(st->crc_init ^ nonce_val));
        return in_crc == calc;
    }
}

/* Channel scaling helpers matching crsf_protocol.h */
static inline uint16_t elrs_fmap(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) {
    int32_t res = (((int32_t)(x - in_min) * (int32_t)(out_max - out_min) * 2) / (int32_t)(in_max - in_min) +
                   (int32_t)out_min * 2 + 1) / 2;
    if (res < 0) return 0U;
    if (res > 65535) return 65535U;
    return (uint16_t)res;
}

static inline uint16_t crsf_constrain(uint16_t v, uint16_t lo, uint16_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline uint16_t crsf_to_uint10_limit(uint16_t ch11) {
    uint16_t c = crsf_constrain(ch11, CRSF_CHANNEL_VALUE_STD_MIN, CRSF_CHANNEL_VALUE_STD_MAX);
    return elrs_fmap(c, CRSF_CHANNEL_VALUE_STD_MIN, CRSF_CHANNEL_VALUE_STD_MAX, 0U, 1023U);
}

static inline uint16_t uint10_to_crsf(uint16_t val10) {
    return elrs_fmap(val10, 0U, 1023U, CRSF_CHANNEL_VALUE_STD_MIN, CRSF_CHANNEL_VALUE_STD_MAX);
}

static inline uint16_t crsf_to_n(uint16_t val, uint16_t cnt) {
    if (val <= CRSF_CHANNEL_VALUE_1000) return 0U;
    if (val >= CRSF_CHANNEL_VALUE_2000) return (uint16_t)(cnt - 1U);
    return (uint16_t)(((uint32_t)(val - CRSF_CHANNEL_VALUE_1000) * cnt) /
                      (CRSF_CHANNEL_VALUE_2000 - CRSF_CHANNEL_VALUE_1000 + 1U));
}

static inline uint16_t n_to_crsf(uint16_t val, uint16_t max_val) {
    return (uint16_t)(((uint32_t)val * (CRSF_CHANNEL_VALUE_2000 - CRSF_CHANNEL_VALUE_1000)) / max_val +
                      CRSF_CHANNEL_VALUE_1000);
}

static inline uint8_t crsf_to_switch3b(uint16_t ch) {
    const uint16_t bin_cnt  = 6U;
    const uint16_t bin_size = (CRSF_CHANNEL_VALUE_STD_MAX - CRSF_CHANNEL_VALUE_STD_MIN) / bin_cnt;
    if (ch < (CRSF_CHANNEL_VALUE_MID - bin_size / 4U) ||
        ch > (CRSF_CHANNEL_VALUE_MID + bin_size / 4U)) {
        return (uint8_t)crsf_to_n(ch, bin_cnt);
    }
    return 7U;
}

static inline uint16_t switch3b_to_crsf(uint8_t val) {
    switch (val) {
        case 0: return CRSF_CHANNEL_VALUE_1000;
        case 5: return CRSF_CHANNEL_VALUE_2000;
        case 6:
        case 7: return CRSF_CHANNEL_VALUE_MID;
        default: return (uint16_t)((uint16_t)val * 240U + 391U);
    }
}

static void pack_4x10(const uint16_t ch[4], uint8_t dest[5], bool div2_mode) {
    memset(dest, 0, 5U);
    unsigned dest_idx   = 0U;
    unsigned dest_shift = 0U;
    for (unsigned i = 0U; i < 4U; i++) {
        uint16_t v10 = div2_mode ? (uint16_t)((ch[i] >> 1U) & 0x3FFU) : crsf_to_uint10_limit(ch[i]);
        dest[dest_idx++] |= (uint8_t)((v10 << dest_shift) & 0xFFU);
        unsigned src_bits_left = 10U - 8U + dest_shift;
        dest[dest_idx] = (uint8_t)(v10 >> (10U - src_bits_left));
        dest_shift = src_bits_left;
    }
}

static void unpack_4x10(const uint8_t src[5], uint16_t dest[4], bool div2_mode) {
    uint8_t  bits_merged = 0U;
    uint32_t read_val    = 0U;
    unsigned read_idx    = 0U;
    for (unsigned n = 0U; n < 4U; n++) {
        while (bits_merged < 10U) {
            read_val |= ((uint32_t)src[read_idx++]) << bits_merged;
            bits_merged += 8U;
        }
        uint16_t v10 = (uint16_t)(read_val & 0x3FFU);
        dest[n] = div2_mode ? (uint16_t)(v10 << 1U) : uint10_to_crsf(v10);
        read_val >>= 10U;
        bits_merged -= 10U;
    }
}

static inline uint8_t hybrid_wide_nonce_to_switch_idx(uint8_t nonce) {
    return (uint8_t)(((nonce & 0x07U) + ((nonce >> 3U) & 0x01U)) % 8U);
}

void elrs_ota_pack_sync(const elrs_ota_state_t *st,
                        const elrs_sync_data_t *sync,
                        uint8_t *out_pkt) {
    uint8_t pkt_len = st->is_full_res ? ELRS_OTA8_PACKET_SIZE : ELRS_OTA4_PACKET_SIZE;
    memset(out_pkt, 0, pkt_len);
    out_pkt[0] = ELRS_PKT_TYPE_SYNC;
    out_pkt[1] = sync->fhss_index;
    out_pkt[2] = sync->nonce;
    out_pkt[3] = sync->rf_rate_enum;
    out_pkt[4] = (uint8_t)((sync->switch_enc_mode & 0x01U) |
                           ((sync->new_tlm_ratio & 0x07U) << 1U) |
                           ((sync->gemini_mode & 0x01U) << 4U) |
                           ((sync->ota_protocol & 0x03U) << 5U));
    out_pkt[5] = sync->uid4;
    out_pkt[6] = sync->uid5;
    elrs_ota_generate_crc(st, out_pkt, sync->nonce);
}

bool elrs_ota_unpack_sync(const elrs_ota_state_t *st,
                          const uint8_t *pkt,
                          elrs_sync_data_t *out_sync) {
    if ((pkt[0] & 0x03U) != ELRS_PKT_TYPE_SYNC) {
        return false;
    }
    if (!elrs_ota_validate_crc(st, pkt, 0U)) {
        return false;
    }
    out_sync->fhss_index      = pkt[1];
    out_sync->nonce           = pkt[2];
    out_sync->rf_rate_enum    = pkt[3];
    out_sync->switch_enc_mode = pkt[4] & 0x01U;
    out_sync->new_tlm_ratio   = (pkt[4] >> 1U) & 0x07U;
    out_sync->gemini_mode     = (pkt[4] >> 4U) & 0x01U;
    out_sync->ota_protocol    = (pkt[4] >> 5U) & 0x03U;
    out_sync->uid4            = pkt[5];
    out_sync->uid5            = pkt[6];
    return true;
}

void elrs_ota_pack_rc(elrs_ota_state_t *st,
                      const uint16_t ch_crsf[ELRS_NUM_CHANNELS],
                      bool is_armed,
                      bool stubborn_ack,
                      uint8_t *out_pkt) {
    if (!st->is_full_res) {
        memset(out_pkt, 0, ELRS_OTA4_PACKET_SIZE);
        out_pkt[0] = ELRS_PKT_TYPE_RCDATA;
        pack_4x10(&ch_crsf[0], &out_pkt[1], false);

        uint8_t sw_byte = 0U;
        if (st->switch_mode == ELRS_SW_WIDE_OR_8CH) {
            uint8_t sw_idx = hybrid_wide_nonce_to_switch_idx(st->ota_nonce);
            sw_byte = (uint8_t)((stubborn_ack ? 1U : 0U) << 6U);
            if (sw_idx == 7U) {
                sw_byte |= (st->uplink_tx_power_enum & 0x3FU);
            } else {
                uint16_t v6 = crsf_to_n(ch_crsf[4U + sw_idx + 1U], 64U) & 0x3FU;
                sw_byte |= (uint8_t)v6;
            }
        } else {
            uint8_t idx = st->hybrid8_next_sw_idx;
            uint8_t val;
            if (idx == 6U) {
                val = (uint8_t)(crsf_to_n(ch_crsf[11], 16U) & 0x0FU);
            } else {
                val = crsf_to_switch3b(ch_crsf[5U + idx]);
            }
            sw_byte = (uint8_t)(((stubborn_ack ? 1U : 0U) << 6U) | (idx << 3U) | val);
            st->hybrid8_next_sw_idx = (uint8_t)((idx + 1U) % 7U);
        }
        out_pkt[6] = (uint8_t)((sw_byte & 0x7FU) | ((is_armed ? 1U : 0U) << 7U));
        elrs_ota_generate_crc(st, out_pkt, st->ota_nonce);
    } else {
        memset(out_pkt, 0, ELRS_OTA8_PACKET_SIZE);
        bool high_aux = (st->switch_mode == ELRS_SW_WIDE_OR_8CH) ? false : st->fullres_is_high_aux;
        if (st->switch_mode != ELRS_SW_WIDE_OR_8CH) {
            st->fullres_is_high_aux = !st->fullres_is_high_aux;
        }
        uint8_t pwr = st->uplink_tx_power_enum;
        if (pwr < 1U) pwr = 1U;
        if (pwr > 8U) pwr = 8U;
        out_pkt[0] = (uint8_t)(ELRS_PKT_TYPE_RCDATA |
                               ((stubborn_ack ? 1U : 0U) << 2U) |
                               (((pwr - 1U) & 0x07U) << 3U) |
                               ((high_aux ? 1U : 0U) << 6U) |
                               ((is_armed ? 1U : 0U) << 7U));
        uint8_t src_lo = 0U;
        uint8_t src_hi = 4U;
        if (st->switch_mode == ELRS_SW_HYBRID_OR_16CH) {
            if (high_aux) {
                src_lo = 8U;
                src_hi = 12U;
            }
        } else {
            src_hi = high_aux ? 8U : 4U;
        }
        pack_4x10(&ch_crsf[src_lo], &out_pkt[1], true);
        pack_4x10(&ch_crsf[src_hi], &out_pkt[6], true);
        elrs_ota_generate_crc(st, out_pkt, st->ota_nonce);
    }
}

bool elrs_ota_unpack_rc(const elrs_ota_state_t *st,
                        const uint8_t *pkt,
                        uint16_t ch_crsf[ELRS_NUM_CHANNELS],
                        bool *out_is_armed,
                        bool *out_stubborn_ack) {
    if ((pkt[0] & 0x03U) != ELRS_PKT_TYPE_RCDATA) {
        return false;
    }
    if (!elrs_ota_validate_crc(st, pkt, st->ota_nonce)) {
        return false;
    }
    if (!st->is_full_res) {
        unpack_4x10(&pkt[1], &ch_crsf[0], false);
        bool armed = (pkt[6] >> 7U) != 0U;
        uint8_t sw_byte = pkt[6] & 0x7FU;
        ch_crsf[4]  = armed ? CRSF_CHANNEL_VALUE_2000 : CRSF_CHANNEL_VALUE_1000;
        ch_crsf[13] = ch_crsf[4];

        bool ack = ((sw_byte >> 6U) & 0x01U) != 0U;
        if (st->switch_mode == ELRS_SW_WIDE_OR_8CH) {
            uint8_t sw_idx = hybrid_wide_nonce_to_switch_idx(st->ota_nonce);
            if (sw_idx < 7U) {
                ch_crsf[5U + sw_idx] = n_to_crsf(sw_byte & 0x3FU, 63U);
            }
        } else {
            uint8_t sw_idx = (sw_byte >> 3U) & 0x07U;
            if (sw_idx >= 6U) {
                ch_crsf[11] = n_to_crsf(sw_byte & 0x0FU, 15U);
            } else {
                ch_crsf[5U + sw_idx] = switch3b_to_crsf(sw_byte & 0x07U);
            }
        }
        if (out_is_armed != NULL) *out_is_armed = armed;
        if (out_stubborn_ack != NULL) *out_stubborn_ack = ack;
        return true;
    } else {
        bool ack      = ((pkt[0] >> 2U) & 0x01U) != 0U;
        bool high_aux = ((pkt[0] >> 6U) & 0x01U) != 0U;
        bool armed    = ((pkt[0] >> 7U) & 0x01U) != 0U;
        uint8_t dst_lo = 0U;
        uint8_t dst_hi = 4U;
        if (st->switch_mode == ELRS_SW_HYBRID_OR_16CH) {
            if (high_aux) {
                dst_lo = 8U;
                dst_hi = 12U;
            }
        } else {
            dst_hi = high_aux ? 8U : 4U;
            ch_crsf[13] = armed ? CRSF_CHANNEL_VALUE_2000 : CRSF_CHANNEL_VALUE_1000;
        }
        unpack_4x10(&pkt[1], &ch_crsf[dst_lo], true);
        unpack_4x10(&pkt[6], &ch_crsf[dst_hi], true);
        if (out_is_armed != NULL) *out_is_armed = armed;
        if (out_stubborn_ack != NULL) *out_stubborn_ack = ack;
        return true;
    }
}

void elrs_ota_pack_linkstats(const elrs_ota_state_t *st,
                             const elrs_link_stats_data_t *stats,
                             uint8_t pkg_index,
                             bool stubborn_ack,
                             uint8_t *out_pkt) {
    uint8_t pkt_len = st->is_full_res ? ELRS_OTA8_PACKET_SIZE : ELRS_OTA4_PACKET_SIZE;
    memset(out_pkt, 0, pkt_len);
    if (!st->is_full_res) {
        out_pkt[0] = ELRS_PKT_TYPE_LINKSTATS;
        out_pkt[1] = (uint8_t)((pkg_index & 0x7FU) | ((stubborn_ack ? 1U : 0U) << 7U));
    } else {
        out_pkt[0] = (uint8_t)(ELRS_PKT_TYPE_LINKSTATS |
                               ((stubborn_ack ? 1U : 0U) << 2U) |
                               ((pkg_index & 0x1FU) << 3U));
        out_pkt[1] = 0U; /* Unused or first byte of ul_link_stats at offset 1 in OTA8 */
    }
    uint8_t off = st->is_full_res ? 1U : 2U;
    out_pkt[off + 0U] = (uint8_t)((stats->uplink_rssi_1 & 0x7FU) | ((stats->antenna & 0x01U) << 7U));
    out_pkt[off + 1U] = (uint8_t)((stats->uplink_rssi_2 & 0x7FU) | ((stats->model_match & 0x01U) << 7U));
    out_pkt[off + 2U] = (uint8_t)((stats->lq & 0x7FU) | ((stats->true_diversity & 0x01U) << 7U));
    out_pkt[off + 3U] = (uint8_t)stats->snr_scaled;
    elrs_ota_generate_crc(st, out_pkt, st->ota_nonce);
}

bool elrs_ota_unpack_linkstats(const elrs_ota_state_t *st,
                               const uint8_t *pkt,
                               elrs_link_stats_data_t *out_stats,
                               uint8_t *out_pkg_index,
                               bool *out_stubborn_ack) {
    if ((pkt[0] & 0x03U) != ELRS_PKT_TYPE_LINKSTATS) {
        return false;
    }
    if (!elrs_ota_validate_crc(st, pkt, st->ota_nonce)) {
        return false;
    }
    uint8_t off;
    if (!st->is_full_res) {
        if (out_pkg_index != NULL) *out_pkg_index = pkt[1] & 0x7FU;
        if (out_stubborn_ack != NULL) *out_stubborn_ack = (pkt[1] >> 7U) != 0U;
        off = 2U;
    } else {
        if (out_stubborn_ack != NULL) *out_stubborn_ack = ((pkt[0] >> 2U) & 0x01U) != 0U;
        if (out_pkg_index != NULL) *out_pkg_index = (pkt[0] >> 3U) & 0x1FU;
        off = 1U;
    }
    out_stats->uplink_rssi_1  = pkt[off + 0U] & 0x7FU;
    out_stats->antenna        = (pkt[off + 0U] >> 7U) & 0x01U;
    out_stats->uplink_rssi_2  = pkt[off + 1U] & 0x7FU;
    out_stats->model_match    = (pkt[off + 1U] >> 7U) & 0x01U;
    out_stats->lq             = pkt[off + 2U] & 0x7FU;
    out_stats->true_diversity = (pkt[off + 2U] >> 7U) & 0x01U;
    out_stats->snr_scaled     = (int8_t)pkt[off + 3U];
    return true;
}

/* ============================================================================
 * 3. ExpressLRS PRNG & FHSS Sequence Generator
 * ============================================================================ */

static const elrs_fhss_domain_info_t k_fhss_domains[ELRS_DOMAIN_COUNT] = {
    [ELRS_DOMAIN_AU915]  = {"AU915",  915500000UL, 926900000UL, 20U, 921000000UL},
    [ELRS_DOMAIN_FCC915] = {"FCC915", 903500000UL, 926900000UL, 40U, 915000000UL},
    [ELRS_DOMAIN_EU868]  = {"EU868",  863275000UL, 869575000UL, 13U, 868000000UL},
    [ELRS_DOMAIN_IN866]  = {"IN866",  865375000UL, 866950000UL,  4U, 866000000UL},
    [ELRS_DOMAIN_AU433]  = {"AU433",  433420000UL, 434420000UL,  3U, 434000000UL},
    [ELRS_DOMAIN_EU433]  = {"EU433",  433100000UL, 434450000UL,  3U, 434000000UL},
    [ELRS_DOMAIN_US433]  = {"US433",  433250000UL, 438000000UL,  8U, 434000000UL},
    [ELRS_DOMAIN_US433W] = {"US433W", 423500000UL, 438000000UL, 20U, 434000000UL},
    [ELRS_DOMAIN_TH920]  = {"TH920",  920500000UL, 924700000UL,  8U, 922600000UL},
    [ELRS_DOMAIN_ISM2G4] = {"ISM2G4", 2400400000UL, 2479400000UL, 80U, 2440000000UL},
    [ELRS_DOMAIN_CE2G4]  = {"CE_LBT", 2400400000UL, 2479400000UL, 80U, 2440000000UL},
};

static uint16_t elrs_rng_next(uint32_t *rng_seed) {
    const uint32_t m = 2147483648UL;
    const uint32_t a = 214013UL;
    const uint32_t c = 2531011UL;
    *rng_seed = (a * (*rng_seed) + c) % m;
    return (uint16_t)(*rng_seed >> 16U);
}

const elrs_fhss_domain_info_t *elrs_fhss_get_domain_info(elrs_domain_t domain) {
    if ((unsigned)domain >= (unsigned)ELRS_DOMAIN_COUNT) {
        domain = ELRS_DOMAIN_FCC915;
    }
    return &k_fhss_domains[domain];
}

void elrs_fhss_init(elrs_fhss_state_t *fhss, elrs_domain_t domain, uint32_t uid_seed) {
    const elrs_fhss_domain_info_t *info = elrs_fhss_get_domain_info(domain);
    fhss->domain         = domain;
    fhss->info           = info;
    fhss->sync_channel   = (uint8_t)(info->freq_count / 2U);
    fhss->freq_spread_hz = (info->freq_stop_hz - info->freq_start_hz) / (info->freq_count - 1U);
    fhss->hop_ptr        = 0U;

    const uint16_t seq_cnt = (uint16_t)((ELRS_FHSS_SEQUENCE_LEN / info->freq_count) * info->freq_count);
    fhss->sequence_count   = seq_cnt;

    uint32_t rng_state = uid_seed;
    for (uint16_t i = 0U; i < seq_cnt; i++) {
        uint16_t rem = i % info->freq_count;
        if (rem == 0U) {
            fhss->sequence[i] = fhss->sync_channel;
        } else if (rem == fhss->sync_channel) {
            fhss->sequence[i] = 0U;
        } else {
            fhss->sequence[i] = (uint8_t)rem;
        }
    }

    for (uint16_t i = 0U; i < seq_cnt; i++) {
        if ((i % info->freq_count) != 0U) {
            uint8_t offset = (uint8_t)((i / info->freq_count) * info->freq_count);
            uint8_t r      = (uint8_t)((elrs_rng_next(&rng_state) % (info->freq_count - 1U)) + 1U);
            uint8_t tmp    = fhss->sequence[i];
            fhss->sequence[i] = fhss->sequence[offset + r];
            fhss->sequence[offset + r] = tmp;
        }
    }
}

uint32_t elrs_fhss_get_initial_freq(const elrs_fhss_state_t *fhss) {
    return fhss->info->freq_start_hz + (uint32_t)fhss->sync_channel * fhss->freq_spread_hz;
}

uint32_t elrs_fhss_get_curr_freq(const elrs_fhss_state_t *fhss) {
    uint8_t ch = fhss->sequence[fhss->hop_ptr % fhss->sequence_count];
    return fhss->info->freq_start_hz + (uint32_t)ch * fhss->freq_spread_hz;
}

uint32_t elrs_fhss_get_gemini_freq(const elrs_fhss_state_t *fhss) {
    uint8_t ch = fhss->sequence[fhss->hop_ptr % fhss->sequence_count];
    uint8_t gemini_ch = (uint8_t)((ch + (fhss->info->freq_count / 2U)) % fhss->info->freq_count);
    return fhss->info->freq_start_hz + (uint32_t)gemini_ch * fhss->freq_spread_hz;
}

uint32_t elrs_fhss_advance_and_get_freq(elrs_fhss_state_t *fhss) {
    fhss->hop_ptr = (uint16_t)((fhss->hop_ptr + 1U) % fhss->sequence_count);
    return elrs_fhss_get_curr_freq(fhss);
}

void elrs_fhss_set_index(elrs_fhss_state_t *fhss, uint8_t idx) {
    fhss->hop_ptr = (uint16_t)(idx % fhss->sequence_count);
}

bool elrs_fhss_on_sync_channel(const elrs_fhss_state_t *fhss) {
    return fhss->sequence[fhss->hop_ptr % fhss->sequence_count] == fhss->sync_channel;
}

/* ============================================================================
 * 4. ExpressLRS Air Rate Configurations for LR2021
 * ============================================================================ */

static const elrs_air_rate_config_t k_air_rates[] = {
    /* Sub-GHz (868 / 915 MHz) */
    {"900_K1000_8CH", ELRS_RATE_FSK_900_1000HZ_8CH,  ELRS_PHY_FSK_900,  LR2021_RADIO_GFSK_BITRATE_300k, LR2021_RADIO_GFSK_BW_476000, LR2021_RADIO_GFSK_FDEV_100k, 16,  1000, 2, 128, 13, 1},
    {"900_250HZ",     ELRS_RATE_LORA_900_250HZ,      ELRS_PHY_LORA_900, LR2021_RADIO_LORA_BW_500,       LR2021_RADIO_LORA_SF5,       LR2021_RADIO_LORA_CR_4_8,     8,  4000, 4,  64,  8, 1},
    {"900_200HZ_8CH", ELRS_RATE_LORA_900_200HZ_8CH,  ELRS_PHY_LORA_900, LR2021_RADIO_LORA_BW_500,       LR2021_RADIO_LORA_SF5,       LR2021_RADIO_LORA_CR_4_7,     8,  5000, 4,  64, 13, 1},
    {"900_200HZ",     ELRS_RATE_LORA_900_200HZ,      ELRS_PHY_LORA_900, LR2021_RADIO_LORA_BW_500,       LR2021_RADIO_LORA_SF6,       LR2021_RADIO_LORA_CR_4_7,     8,  5000, 4,  64,  8, 1},
    {"900_100HZ_8CH", ELRS_RATE_LORA_900_100HZ_8CH,  ELRS_PHY_LORA_900, LR2021_RADIO_LORA_BW_500,       LR2021_RADIO_LORA_SF6,       LR2021_RADIO_LORA_CR_4_8,     8, 10000, 4,  32, 13, 1},
    {"900_100HZ",     ELRS_RATE_LORA_900_100HZ,      ELRS_PHY_LORA_900, LR2021_RADIO_LORA_BW_500,       LR2021_RADIO_LORA_SF7,       LR2021_RADIO_LORA_CR_4_7,     8, 10000, 4,  32,  8, 1},
    {"900_50HZ",      ELRS_RATE_LORA_900_50HZ,       ELRS_PHY_LORA_900, LR2021_RADIO_LORA_BW_500,       LR2021_RADIO_LORA_SF8,       LR2021_RADIO_LORA_CR_4_7,    10, 20000, 4,  16,  8, 1},
    {"900_25HZ",      ELRS_RATE_LORA_900_25HZ,       ELRS_PHY_LORA_900, LR2021_RADIO_LORA_BW_500,       LR2021_RADIO_LORA_SF9,       LR2021_RADIO_LORA_CR_4_7,    10, 40000, 2,   8,  8, 1},
    {"900_D50HZ",     ELRS_RATE_LORA_900_50HZ_DVDA,  ELRS_PHY_LORA_900, LR2021_RADIO_LORA_BW_500,       LR2021_RADIO_LORA_SF6,       LR2021_RADIO_LORA_CR_4_7,     8,  5000, 2,  64,  8, 4},

    /* 2.4 GHz (FLRC, FSK, LoRa) */
    {"2G4_F1000",     ELRS_RATE_FLRC_2G4_1000HZ,     ELRS_PHY_FLRC_2G4, LR2021_RADIO_FLRC_BR_0_650_BW_0_6, LR2021_RADIO_FLRC_BT_1,   LR2021_RADIO_FLRC_CR_1_2,    32,  1000, 2, 128,  8, 1},
    {"2G4_F500",      ELRS_RATE_FLRC_2G4_500HZ,      ELRS_PHY_FLRC_2G4, LR2021_RADIO_FLRC_BR_0_650_BW_0_6, LR2021_RADIO_FLRC_BT_1,   LR2021_RADIO_FLRC_CR_1_2,    32,  2000, 2, 128,  8, 1},
    {"2G4_D500",      ELRS_RATE_FLRC_2G4_500HZ_DVDA, ELRS_PHY_FLRC_2G4, LR2021_RADIO_FLRC_BR_0_650_BW_0_6, LR2021_RADIO_FLRC_BT_1,   LR2021_RADIO_FLRC_CR_1_2,    32,  1000, 2, 128,  8, 2},
    {"2G4_D250",      ELRS_RATE_FLRC_2G4_250HZ_DVDA, ELRS_PHY_FLRC_2G4, LR2021_RADIO_FLRC_BR_0_650_BW_0_6, LR2021_RADIO_FLRC_BT_1,   LR2021_RADIO_FLRC_CR_1_2,    32,  1000, 2, 128,  8, 4},
    {"2G4_K1000",     ELRS_RATE_FSK_2G4_1000HZ,      ELRS_PHY_FSK_2G4,  LR2021_RADIO_GFSK_BITRATE_300k, LR2021_RADIO_GFSK_BW_476000, LR2021_RADIO_GFSK_FDEV_100k, 16,  1000, 2, 128,  8, 1},
    {"2G4_DK500",     ELRS_RATE_FSK_2G4_500HZ_DVDA,  ELRS_PHY_FSK_2G4,  LR2021_RADIO_GFSK_BITRATE_300k, LR2021_RADIO_GFSK_BW_476000, LR2021_RADIO_GFSK_FDEV_100k, 16,  1000, 2, 128,  8, 2},
    {"2G4_DK250",     ELRS_RATE_FSK_2G4_250HZ_DVDA,  ELRS_PHY_FSK_2G4,  LR2021_RADIO_GFSK_BITRATE_300k, LR2021_RADIO_GFSK_BW_476000, LR2021_RADIO_GFSK_FDEV_100k, 16,  1000, 2, 128,  8, 4},
    {"2G4_500HZ",     ELRS_RATE_LORA_2G4_500HZ,      ELRS_PHY_LORA_2G4, LR2021_RADIO_LORA_BW_800,       LR2021_RADIO_LORA_SF5,       LR2021_RADIO_LORA_CR_LI_4_6, 12,  2000, 4, 128,  8, 1},
    {"2G4_333HZ_8CH", ELRS_RATE_LORA_2G4_333HZ_8CH,  ELRS_PHY_LORA_2G4, LR2021_RADIO_LORA_BW_800,       LR2021_RADIO_LORA_SF5,       LR2021_RADIO_LORA_CR_LI_4_8, 12,  3003, 4, 128, 13, 1},
    {"2G4_250HZ",     ELRS_RATE_LORA_2G4_250HZ,      ELRS_PHY_LORA_2G4, LR2021_RADIO_LORA_BW_800,       LR2021_RADIO_LORA_SF6,       LR2021_RADIO_LORA_CR_LI_4_8, 14,  4000, 4,  64,  8, 1},
    {"2G4_150HZ",     ELRS_RATE_LORA_2G4_150HZ,      ELRS_PHY_LORA_2G4, LR2021_RADIO_LORA_BW_800,       LR2021_RADIO_LORA_SF7,       LR2021_RADIO_LORA_CR_LI_4_8, 12,  6666, 4,  32,  8, 1},
    {"2G4_100HZ_8CH", ELRS_RATE_LORA_2G4_100HZ_8CH,  ELRS_PHY_LORA_2G4, LR2021_RADIO_LORA_BW_800,       LR2021_RADIO_LORA_SF7,       LR2021_RADIO_LORA_CR_LI_4_8, 12, 10000, 4,  32, 13, 1},
    {"2G4_50HZ",      ELRS_RATE_LORA_2G4_50HZ,       ELRS_PHY_LORA_2G4, LR2021_RADIO_LORA_BW_800,       LR2021_RADIO_LORA_SF8,       LR2021_RADIO_LORA_CR_LI_4_8, 12, 20000, 2,  16,  8, 1},
};

uint8_t elrs_get_air_rate_count(void) {
    return (uint8_t)(sizeof(k_air_rates) / sizeof(k_air_rates[0]));
}

const elrs_air_rate_config_t *elrs_get_air_rate_by_index(uint8_t idx) {
    uint8_t cnt = elrs_get_air_rate_count();
    if (idx >= cnt) {
        return NULL;
    }
    return &k_air_rates[idx];
}

const elrs_air_rate_config_t *elrs_get_air_rate_by_enum(uint8_t rate_enum) {
    uint8_t cnt = elrs_get_air_rate_count();
    for (uint8_t i = 0U; i < cnt; i++) {
        if ((uint8_t)k_air_rates[i].rate_enum == rate_enum) {
            return &k_air_rates[i];
        }
    }
    return NULL;
}
