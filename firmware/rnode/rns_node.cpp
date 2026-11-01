/* SPDX-License-Identifier: AGPL-3.0-or-later */
/*
 * Reticulum / RNode KISS interface, Airborne Store-and-Forward Relay,
 * MAVLink Flight Controller Bridge, and ExpressLRS OTA DATA Tunnel.
 */

#include "rns_node.h"
#include "../elrs/elrs_protocol.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* SHA256(SHA256("rnlog.beacon")[:10])[:16] */
const uint8_t RNS_DEST_RNLOG_BEACON[16] = {
    0x18, 0xbc, 0xd8, 0xa3, 0xde, 0xa1, 0x6e, 0xf6,
    0x76, 0x5c, 0x6b, 0x27, 0xd0, 0x08, 0xd2, 0x20
};

/* SHA256(SHA256("elrs.drone.cmd")[:10])[:16] */
const uint8_t RNS_DEST_DRONE_CMD[16] = {
    0x73, 0x45, 0x47, 0xdb, 0x63, 0x02, 0x36, 0x18,
    0xae, 0x2b, 0xd9, 0xa6, 0x51, 0x87, 0xba, 0xfc
};

/* ---------------------------------------------------------------------------
 * 1. RNode KISS Framing & PHYPRM Builder
 * --------------------------------------------------------------------------- */
void rnode_kiss_init(rnode_kiss_parser_t *p, rnode_kiss_frame_cb_t cb, void *user_ctx) {
    if (p == NULL) return;
    p->state    = RNODE_KISS_WAIT_FEND;
    p->cmd      = 0U;
    p->pos      = 0U;
    p->callback = cb;
    p->user_ctx = user_ctx;
}

void rnode_kiss_feed_byte(rnode_kiss_parser_t *p, uint8_t b) {
    if (p == NULL) return;

    switch (p->state) {
        case RNODE_KISS_WAIT_FEND:
            if (b == RNODE_FEND) {
                p->state = RNODE_KISS_WAIT_CMD;
            }
            break;

        case RNODE_KISS_WAIT_CMD:
            if (b == RNODE_FEND) {
                break;
            }
            p->cmd   = b;
            p->pos   = 0U;
            p->state = RNODE_KISS_IN_FRAME;
            break;

        case RNODE_KISS_IN_FRAME:
            if (b == RNODE_FEND) {
                if (p->callback != NULL) {
                    p->callback(p->cmd, p->buf, p->pos, p->user_ctx);
                }
                p->state = RNODE_KISS_WAIT_CMD;
            } else if (b == RNODE_FESC) {
                p->state = RNODE_KISS_ESCAPE;
            } else {
                if (p->pos < RNODE_KISS_BUF_SIZE) {
                    p->buf[p->pos++] = b;
                }
            }
            break;

        case RNODE_KISS_ESCAPE:
            if (b == RNODE_TFEND) {
                if (p->pos < RNODE_KISS_BUF_SIZE) {
                    p->buf[p->pos++] = RNODE_FEND;
                }
            } else if (b == RNODE_TFESC) {
                if (p->pos < RNODE_KISS_BUF_SIZE) {
                    p->buf[p->pos++] = RNODE_FESC;
                }
            }
            p->state = RNODE_KISS_IN_FRAME;
            break;
    }
}

size_t rnode_kiss_build_frame(uint8_t *out, size_t out_size, uint8_t cmd, const uint8_t *data, size_t len) {
    if (out == NULL || out_size < 3U) {
        return 0U;
    }
    size_t i = 0U;
    out[i++] = RNODE_FEND;
    out[i++] = cmd;

    for (size_t j = 0U; j < len && i < (out_size - 1U); j++) {
        uint8_t b = data[j];
        if (b == RNODE_FEND) {
            if (i + 2U > (out_size - 1U)) break;
            out[i++] = RNODE_FESC;
            out[i++] = RNODE_TFEND;
        } else if (b == RNODE_FESC) {
            if (i + 2U > (out_size - 1U)) break;
            out[i++] = RNODE_FESC;
            out[i++] = RNODE_TFESC;
        } else {
            out[i++] = b;
        }
    }

    out[i++] = RNODE_FEND;
    return i;
}

size_t rnode_build_phyprm(uint32_t bw_hz, uint8_t sf, uint16_t preamble_sym,
                          uint16_t *out_slot_ms, uint16_t *out_difs_ms, uint8_t out_buf[12]) {
    if (bw_hz == 0UL || sf < 5U || sf > 12U) {
        return 0U;
    }
    uint32_t sym_us = ((1UL << sf) * 1000000UL) / bw_hz;
    uint16_t sym_time = (sym_us > 65535UL) ? 65535U : (uint16_t)sym_us;
    uint16_t sym_rate = (uint16_t)(bw_hz / (1UL << sf));
    uint32_t pre_us   = (uint32_t)preamble_sym * sym_us;
    uint16_t pre_ms   = (uint16_t)((pre_us + 999UL) / 1000UL);

    uint32_t slot = (sym_us + 999UL) / 1000UL;
    if (slot < 1UL) slot = 1UL;
    if (slot > 1000UL) slot = 1000UL;
    uint16_t slot_ms = (uint16_t)slot;
    uint16_t difs_ms = (uint16_t)(slot_ms * 3U);

    if (out_slot_ms != NULL) *out_slot_ms = slot_ms;
    if (out_difs_ms != NULL) *out_difs_ms = difs_ms;

    if (out_buf != NULL) {
        out_buf[0]  = (uint8_t)(sym_time >> 8U);
        out_buf[1]  = (uint8_t)(sym_time & 0xFFU);
        out_buf[2]  = (uint8_t)(sym_rate >> 8U);
        out_buf[3]  = (uint8_t)(sym_rate & 0xFFU);
        out_buf[4]  = (uint8_t)(preamble_sym >> 8U);
        out_buf[5]  = (uint8_t)(preamble_sym & 0xFFU);
        out_buf[6]  = (uint8_t)(pre_ms >> 8U);
        out_buf[7]  = (uint8_t)(pre_ms & 0xFFU);
        out_buf[8]  = (uint8_t)(slot_ms >> 8U);
        out_buf[9]  = (uint8_t)(slot_ms & 0xFFU);
        out_buf[10] = (uint8_t)(difs_ms >> 8U);
        out_buf[11] = (uint8_t)(difs_ms & 0xFFU);
    }
    return 12U;
}

/* ---------------------------------------------------------------------------
 * 2. Reticulum Airborne Store-and-Forward Relay & Deduplicator
 * --------------------------------------------------------------------------- */
void rns_relay_init(rns_relay_table_t *t) {
    if (t == NULL) return;
    memset(t, 0, sizeof(*t));
}

/* Computes a 64-bit FNV-1a invariant hash over the Reticulum packet:
 * - Excludes byte 1 (hops) for unmasked packets so incremented-hop echoes match.
 * - Includes all payload bytes from offset 2..len-1.
 */
uint64_t rns_packet_invariant_hash(const uint8_t *pkt, size_t len) {
    if (pkt == NULL || len < RNS_MIN_PACKET_LEN) {
        return 0ULL;
    }
    uint64_t h = 14695981039346656037ULL;
    const uint64_t prime = 1099511628211ULL;

    uint8_t invariant_flags = (uint8_t)(pkt[0] & 0x8FU);
    h ^= (uint64_t)invariant_flags;
    h *= prime;

    for (size_t i = 2U; i < len; i++) {
        h ^= (uint64_t)pkt[i];
        h *= prime;
    }
    return h;
}

static bool rns_dedup_contains(const rns_relay_table_t *t, uint64_t hash, uint32_t now_ms) {
    for (uint8_t i = 0U; i < RNS_DEDUP_RING_SIZE; i++) {
        if (t->ring[i].valid && t->ring[i].hash == hash) {
            if ((uint32_t)(now_ms - t->ring[i].timestamp_ms) <= RNS_DEDUP_WINDOW_MS) {
                return true;
            }
        }
    }
    return false;
}

void rns_relay_record_sent(rns_relay_table_t *t, const uint8_t *pkt, size_t len, uint32_t now_ms) {
    if (t == NULL || pkt == NULL || len < RNS_MIN_PACKET_LEN) {
        return;
    }
    uint64_t h = rns_packet_invariant_hash(pkt, len);
    t->ring[t->head].hash         = h;
    t->ring[t->head].timestamp_ms = now_ms;
    t->ring[t->head].valid        = true;
    t->head = (uint8_t)((t->head + 1U) % RNS_DEDUP_RING_SIZE);
}

bool rns_relay_should_forward(rns_relay_table_t *t, uint8_t *inout_pkt, size_t len, uint32_t now_ms) {
    if (t == NULL || inout_pkt == NULL || len < RNS_MIN_PACKET_LEN || len > RNODE_HW_MTU) {
        return false;
    }

    uint64_t h = rns_packet_invariant_hash(inout_pkt, len);
    if (rns_dedup_contains(t, h, now_ms)) {
        t->dropped_dup_count++;
        return false;
    }

    /* If IFAC bit (0x80) is NOT set, increment hop count (up to max 128).
     * If IFAC bit (0x80) IS set, preserve exact bytes so end-to-end IFAC MAC stays valid.
     */
    if ((inout_pkt[0] & 0x80U) == 0U) {
        if (inout_pkt[1] >= 128U) {
            return false;
        }
        inout_pkt[1]++;
    }

    t->ring[t->head].hash         = h;
    t->ring[t->head].timestamp_ms = now_ms;
    t->ring[t->head].valid        = true;
    t->head = (uint8_t)((t->head + 1U) % RNS_DEDUP_RING_SIZE);
    t->relayed_count++;
    return true;
}

/* ---------------------------------------------------------------------------
 * 3. Flight Controller (MAVLink <-> Reticulum) Telemetry & Command Bridge
 * --------------------------------------------------------------------------- */
void rns_mav_parser_init(rns_mav_parser_t *p) {
    if (p == NULL) return;
    memset(p, 0, sizeof(*p));
}

static uint32_t read_u32_le(const uint8_t *b) {
    return ((uint32_t)b[0]) |
           ((uint32_t)b[1] << 8U) |
           ((uint32_t)b[2] << 16U) |
           ((uint32_t)b[3] << 24U);
}

static int32_t read_i32_le(const uint8_t *b) {
    return (int32_t)read_u32_le(b);
}

static uint16_t read_u16_le(const uint8_t *b) {
    return (uint16_t)(((uint16_t)b[0]) | ((uint16_t)b[1] << 8U));
}

static void rns_mav_handle_msg(uint32_t msg_id, const uint8_t *pay, uint8_t len, rns_fc_telemetry_t *tlm) {
    uint8_t padded[36];
    memset(padded, 0, sizeof(padded));
    if (len > sizeof(padded)) len = sizeof(padded);
    memcpy(padded, pay, len);

    if (msg_id == 0UL) { /* HEARTBEAT */
        tlm->custom_mode = (uint8_t)(read_u32_le(&padded[0]) & 0xFFU);
        tlm->armed       = (padded[6] & 0x80U) != 0U;
        tlm->rx_mav_pkts++;
    } else if (msg_id == 1UL) { /* SYS_STATUS */
        tlm->vbatt_mv     = read_u16_le(&padded[14]);
        tlm->curr_ca      = read_u16_le(&padded[16]);
        tlm->batt_rem_pct = (int8_t)padded[30];
        tlm->rx_mav_pkts++;
    } else if (msg_id == 33UL) { /* GLOBAL_POSITION_INT */
        tlm->lat_deg_e7  = read_i32_le(&padded[4]);
        tlm->lon_deg_e7  = read_i32_le(&padded[8]);
        tlm->alt_mm      = read_i32_le(&padded[12]);
        tlm->rel_alt_mm  = read_i32_le(&padded[16]);
        int16_t vx       = (int16_t)read_u16_le(&padded[20]);
        int16_t vy       = (int16_t)read_u16_le(&padded[22]);
        uint32_t v2      = (uint32_t)((int32_t)vx * vx + (int32_t)vy * vy);
        /* Fast integer square root for groundspeed_cms */
        uint32_t root = 0UL, bit = 1UL << 30;
        while (bit > v2) bit >>= 2U;
        while (bit != 0UL) {
            if (v2 >= root + bit) {
                v2 -= root + bit;
                root = (root >> 1U) + bit;
            } else {
                root >>= 1U;
            }
            bit >>= 2U;
        }
        tlm->groundspeed_cms = (uint16_t)root;
        tlm->heading_cdeg    = read_u16_le(&padded[26]);
        tlm->has_gps_fix     = (tlm->lat_deg_e7 != 0 || tlm->lon_deg_e7 != 0);
        tlm->rx_mav_pkts++;
    }
}

bool rns_mav_parse_byte(rns_mav_parser_t *p, uint8_t b, rns_fc_telemetry_t *inout_tlm) {
    if (p == NULL || inout_tlm == NULL) return false;

    switch (p->state) {
        case 0U: /* Wait STX */
            if (b == 0xFDU) {
                p->is_v2  = true;
                p->state  = 1U;
            } else if (b == 0xFEU) {
                p->is_v2  = false;
                p->state  = 1U;
            }
            break;

        case 1U: /* Payload len */
            p->payload_len = b;
            p->pay_idx     = 0U;
            p->state       = 2U;
            break;

        case 2U: /* v2 incompat_flags or v1 seq */
            p->incompat_flags = p->is_v2 ? b : 0U;
            p->state          = 3U;
            break;

        case 3U: /* v2 compat_flags or v1 sysid */
            p->state = 4U;
            break;

        case 4U: /* v2 seq or v1 compid */
            p->state = 5U;
            break;

        case 5U: /* v2 sysid or v1 msgid */
            if (!p->is_v2) {
                p->msg_id = b;
                p->state  = (p->payload_len > 0U) ? 9U : 10U;
            } else {
                p->state = 6U;
            }
            break;

        case 6U: /* v2 compid */
            p->state = 7U;
            break;

        case 7U: /* v2 msgid low */
            p->msg_id = b;
            p->state  = 8U;
            break;

        case 8U: /* v2 msgid mid */
            p->msg_id |= ((uint32_t)b << 8U);
            p->state   = 12U;
            break;

        case 12U: /* v2 msgid high */
            p->msg_id |= ((uint32_t)b << 16U);
            p->state   = (p->payload_len > 0U) ? 9U : 10U;
            break;

        case 9U: /* Payload bytes */
            p->payload[p->pay_idx++] = b;
            if (p->pay_idx >= p->payload_len) {
                p->state = 10U;
            }
            break;

        case 10U: /* CRC low */
            p->state = 11U;
            break;

        case 11U: /* CRC high */
            p->state = 0U;
            rns_mav_handle_msg(p->msg_id, p->payload, p->payload_len, inout_tlm);
            return true;

        default:
            p->state = 0U;
            break;
    }
    return false;
}

size_t rns_build_telemetry_beacon(const rns_fc_telemetry_t *tlm,
                                  uint8_t elrs_lq,
                                  int8_t elrs_rssi_dbm,
                                  uint8_t *out_pkt,
                                  size_t max_len) {
    if (tlm == NULL || out_pkt == NULL || max_len < 64U) {
        return 0U;
    }

    /* Reticulum HEADER_1 + PLAIN + DATA (0x08), hops=0, dest=rnlog.beacon (16B), context=0x00 */
    out_pkt[0] = 0x08U;
    out_pkt[1] = 0x00U;
    memcpy(&out_pkt[2], RNS_DEST_RNLOG_BEACON, 16U);
    out_pkt[18] = 0x00U;

    const char *lat_sign = (tlm->lat_deg_e7 < 0) ? "-" : "";
    int32_t lat_abs = labs(tlm->lat_deg_e7);
    int32_t lat_i   = lat_abs / 10000000L;
    int32_t lat_f   = (lat_abs % 10000000L) / 100L;

    const char *lon_sign = (tlm->lon_deg_e7 < 0) ? "-" : "";
    int32_t lon_abs = labs(tlm->lon_deg_e7);
    int32_t lon_i   = lon_abs / 10000000L;
    int32_t lon_f   = (lon_abs % 10000000L) / 100L;
    int32_t alt_m   = tlm->rel_alt_mm / 1000L;

    int n = snprintf((char *)&out_pkt[19], max_len - 19U,
                     "{\"src\":\"lr2021-drone\",\"lat\":%s%ld.%05ld,\"lon\":%s%ld.%05ld,"
                     "\"alt_m\":%ld,\"vbatt_mv\":%u,\"mode\":%u,\"armed\":%u,"
                     "\"lq\":%u,\"rssi\":%d}",
                     lat_sign, (long)lat_i, (long)lat_f,
                     lon_sign, (long)lon_i, (long)lon_f,
                     (long)alt_m,
                     (unsigned)tlm->vbatt_mv,
                     (unsigned)tlm->custom_mode,
                     tlm->armed ? 1U : 0U,
                     (unsigned)elrs_lq,
                     (int)elrs_rssi_dbm);
    if (n <= 0) {
        return 0U;
    }
    size_t pay_len = (size_t)n;
    if (19U + pay_len > max_len) {
        pay_len = max_len - 19U;
    }
    return 19U + pay_len;
}

size_t rns_build_drone_command_pkt(const char *cmd_str, uint8_t *out_pkt, size_t max_len) {
    if (cmd_str == NULL || out_pkt == NULL || max_len < RNS_MIN_PACKET_LEN) {
        return 0U;
    }
    size_t slen = strlen(cmd_str);
    if (19U + slen > max_len) {
        return 0U;
    }
    out_pkt[0] = 0x08U; /* HEADER_1 | BROADCAST | PLAIN | DATA */
    out_pkt[1] = 0x00U; /* hops = 0 */
    memcpy(&out_pkt[2], RNS_DEST_DRONE_CMD, 16U);
    out_pkt[18] = 0x00U; /* context */
    memcpy(&out_pkt[19], cmd_str, slen);
    return 19U + slen;
}

bool rns_parse_drone_command_pkt(const uint8_t *pkt, size_t len, rns_drone_cmd_t *out_cmd) {
    if (pkt == NULL || out_cmd == NULL || len <= RNS_MIN_PACKET_LEN) {
        return false;
    }
    if ((pkt[0] & 0x4FU) != 0x08U) {
        return false;
    }
    if (memcmp(&pkt[2], RNS_DEST_DRONE_CMD, 16U) != 0) {
        return false;
    }

    char buf[128];
    size_t pay_len = len - 19U;
    if (pay_len >= sizeof(buf)) {
        pay_len = sizeof(buf) - 1U;
    }
    memcpy(buf, &pkt[19], pay_len);
    buf[pay_len] = '\0';

    memset(out_cmd, 0, sizeof(*out_cmd));
    if (strncmp(buf, "RTL", 3) == 0) {
        out_cmd->type = RNS_DRONE_CMD_RTL;
        return true;
    }
    if (strncmp(buf, "LOITER", 6) == 0) {
        out_cmd->type = RNS_DRONE_CMD_LOITER;
        return true;
    }
    if (strncmp(buf, "LAND", 4) == 0) {
        out_cmd->type = RNS_DRONE_CMD_LAND;
        return true;
    }
    if (strncmp(buf, "DISARM", 6) == 0) {
        out_cmd->type = RNS_DRONE_CMD_DISARM;
        return true;
    }
    if (strncmp(buf, "ARM", 3) == 0) {
        out_cmd->type = RNS_DRONE_CMD_ARM;
        return true;
    }
    if (strncmp(buf, "MODE ", 5) == 0) {
        const char *p = buf + 5;
        char *endp = NULL;
        long m = strtol(p, &endp, 10);
        if (endp == p || m < 0 || m > 255) return false;
        out_cmd->type     = RNS_DRONE_CMD_SET_MODE;
        out_cmd->mode_num = (uint8_t)m;
        return true;
    }
    if (strncmp(buf, "GOTO ", 5) == 0) {
        const char *p = buf + 5;
        char *endp = NULL;
        long lat = strtol(p, &endp, 10);
        if (endp == p) return false;
        p = endp;
        long lon = strtol(p, &endp, 10);
        if (endp == p) return false;
        p = endp;
        long alt = strtol(p, &endp, 10);
        out_cmd->type       = RNS_DRONE_CMD_GUIDED_GOTO;
        out_cmd->lat_deg_e7 = (int32_t)lat;
        out_cmd->lon_deg_e7 = (int32_t)lon;
        out_cmd->alt_m      = (int32_t)alt;
        return true;
    }
    if (strncmp(buf, "CH ", 3) == 0 || strncmp(buf, "RC ", 3) == 0) {
        const char *p = buf + 3;
        char *endp = NULL;
        for (uint8_t i = 0U; i < 4U; i++) {
            long v = strtol(p, &endp, 10);
            if (endp == p) return false;
            out_cmd->rc_us[i] = (uint16_t)v;
            p = endp;
        }
        out_cmd->type = RNS_DRONE_CMD_RC_OVERRIDE;
        return true;
    }
    if (strncmp(buf, "VEL ", 4) == 0 || strncmp(buf, "BVL ", 4) == 0) {
        bool is_body = (buf[0] == 'B');
        const char *p = buf + 4;
        char *endp = NULL;
        long vx = strtol(p, &endp, 10);
        if (endp == p) return false;
        p = endp;
        long vy = strtol(p, &endp, 10);
        if (endp == p) return false;
        p = endp;
        long vz = strtol(p, &endp, 10);
        if (endp == p) return false;
        p = endp;
        out_cmd->type           = RNS_DRONE_CMD_VEL_NED;
        out_cmd->vx_cms         = (int16_t)vx;
        out_cmd->vy_cms         = (int16_t)vy;
        out_cmd->vz_cms         = (int16_t)vz;
        out_cmd->use_body_frame = is_body;
        long yr = strtol(p, &endp, 10);
        if (endp != p) {
            out_cmd->yaw_rate_cdps = (int16_t)yr;
            out_cmd->has_yaw_rate  = true;
        }
        return true;
    }
    return false;
}

/* MAVLink X.25 CRC-16/MCRF4XX helper */
static void mav_crc_accumulate(uint8_t b, uint16_t *crc) {
    uint8_t tmp = b ^ (uint8_t)(*crc & 0xFFU);
    tmp ^= (uint8_t)(tmp << 4U);
    *crc = (uint16_t)((*crc >> 8U) ^
                      ((uint16_t)tmp << 8U) ^
                      ((uint16_t)tmp << 3U) ^
                      ((uint16_t)tmp >> 4U));
}

static void write_u16_le(uint8_t *b, uint16_t v) {
    b[0] = (uint8_t)(v & 0xFFU);
    b[1] = (uint8_t)(v >> 8U);
}

static void write_u32_le(uint8_t *b, uint32_t v) {
    b[0] = (uint8_t)(v & 0xFFU);
    b[1] = (uint8_t)((v >> 8U) & 0xFFU);
    b[2] = (uint8_t)((v >> 16U) & 0xFFU);
    b[3] = (uint8_t)((v >> 24U) & 0xFFU);
}

static void write_f32_le(uint8_t *b, float f) {
    uint32_t u = 0UL;
    memcpy(&u, &f, sizeof(u));
    write_u32_le(b, u);
}

size_t rns_build_mavlink_command(const rns_drone_cmd_t *cmd, uint8_t seq, uint8_t *out_frame, size_t max_len) {
    if (cmd == NULL || out_frame == NULL || max_len < 50U) {
        return 0U;
    }

    uint8_t payload[56];
    memset(payload, 0, sizeof(payload));
    uint8_t pay_len   = 0U;
    uint32_t msg_id   = 0UL;
    uint8_t crc_extra = 0U;

    if (cmd->type == RNS_DRONE_CMD_RTL ||
        cmd->type == RNS_DRONE_CMD_LOITER ||
        cmd->type == RNS_DRONE_CMD_LAND) {
        /* MAVLINK_MSG_ID_COMMAND_LONG (76, len=33, CRC_EXTRA=152) */
        msg_id    = 76UL;
        pay_len   = 33U;
        crc_extra = 152U;
        uint16_t mav_cmd = 20U; /* MAV_CMD_NAV_RETURN_TO_LAUNCH */
        if (cmd->type == RNS_DRONE_CMD_LOITER) mav_cmd = 17U; /* MAV_CMD_NAV_LOITER_UNLIM */
        if (cmd->type == RNS_DRONE_CMD_LAND)   mav_cmd = 21U; /* MAV_CMD_NAV_LAND */
        write_u16_le(&payload[28], mav_cmd);
        payload[30] = 1U; /* target_system = 1 */
        payload[31] = 1U; /* target_component = 1 */
    } else if (cmd->type == RNS_DRONE_CMD_SET_MODE) {
        /* MAV_CMD_DO_SET_MODE (176, param1=MAV_MODE_FLAG_CUSTOM_MODE_ENABLED=1, param2=custom_mode) */
        msg_id    = 76UL;
        pay_len   = 33U;
        crc_extra = 152U;
        write_f32_le(&payload[0], 1.0f);
        write_f32_le(&payload[4], (float)cmd->mode_num);
        write_u16_le(&payload[28], 176U);
        payload[30] = 1U;
        payload[31] = 1U;
    } else if (cmd->type == RNS_DRONE_CMD_ARM || cmd->type == RNS_DRONE_CMD_DISARM) {
        /* MAV_CMD_COMPONENT_ARM_DISARM (400, param1=1.0 arm / 0.0 disarm) */
        msg_id    = 76UL;
        pay_len   = 33U;
        crc_extra = 152U;
        write_f32_le(&payload[0], (cmd->type == RNS_DRONE_CMD_ARM) ? 1.0f : 0.0f);
        write_u16_le(&payload[28], 400U);
        payload[30] = 1U;
        payload[31] = 1U;
    } else if (cmd->type == RNS_DRONE_CMD_GUIDED_GOTO) {
        /* MAVLINK_MSG_ID_MISSION_ITEM_INT (73, len=37, CRC_EXTRA=38) */
        msg_id    = 73UL;
        pay_len   = 37U;
        crc_extra = 38U;
        write_u32_le(&payload[16], (uint32_t)cmd->lat_deg_e7);
        write_u32_le(&payload[20], (uint32_t)cmd->lon_deg_e7);
        write_f32_le(&payload[24], (float)cmd->alt_m);
        write_u16_le(&payload[30], 16U); /* MAV_CMD_NAV_WAYPOINT */
        payload[32] = 1U; /* target_system = 1 */
        payload[33] = 1U; /* target_component = 1 */
        payload[34] = 6U; /* MAV_FRAME_GLOBAL_RELATIVE_ALT_INT */
        payload[35] = 2U; /* current = 2 (Guided mode target) */
        payload[36] = 1U; /* autocontinue = 1 */
    } else if (cmd->type == RNS_DRONE_CMD_RC_OVERRIDE) {
        /* MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE (70, len=18, CRC_EXTRA=124) */
        msg_id    = 70UL;
        pay_len   = 18U;
        crc_extra = 124U;
        for (uint8_t i = 0U; i < 4U; i++) {
            write_u16_le(&payload[i * 2U], cmd->rc_us[i]);
        }
        payload[16] = 1U; /* target_system = 1 */
        payload[17] = 1U; /* target_component = 1 */
    } else if (cmd->type == RNS_DRONE_CMD_VEL_NED) {
        /* MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED (84, len=53, CRC_EXTRA=143)
         * Matches Anisoptera sim/mav_common.py:
         *   mask 0x0DC7 (velocity only) or 0x05C7 (velocity + yaw_rate)
         *   frame MAV_FRAME_BODY_OFFSET_NED (9) or MAV_FRAME_LOCAL_NED (1)
         */
        if (max_len < 65U) return 0U;
        msg_id    = 84UL;
        pay_len   = 53U;
        crc_extra = 143U;
        write_f32_le(&payload[16], (float)cmd->vx_cms * 0.01f);
        write_f32_le(&payload[20], (float)cmd->vy_cms * 0.01f);
        write_f32_le(&payload[24], (float)cmd->vz_cms * 0.01f);
        if (cmd->has_yaw_rate) {
            write_f32_le(&payload[44], ((float)cmd->yaw_rate_cdps * 0.01f) * 0.0174532925f);
        }
        uint16_t mask = cmd->has_yaw_rate ? 0x05C7U : 0x0DC7U;
        write_u16_le(&payload[48], mask);
        payload[50] = 1U; /* target_system = 1 */
        payload[51] = 1U; /* target_component = 1 */
        payload[52] = cmd->use_body_frame ? 9U : 1U; /* BODY_OFFSET_NED (9) or LOCAL_NED (1) */
    } else if (cmd->type == RNS_DRONE_CMD_GLOBAL_POS_VEL) {
        /* MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT (86, len=53, CRC_EXTRA=5)
         * Pos + NED velocity feed-forward + heading in MAV_FRAME_GLOBAL_RELATIVE_ALT_INT (6)
         */
        if (max_len < 65U) return 0U;
        msg_id    = 86UL;
        pay_len   = 53U;
        crc_extra = 5U;
        write_u32_le(&payload[4],  (uint32_t)cmd->lat_deg_e7);
        write_u32_le(&payload[8],  (uint32_t)cmd->lon_deg_e7);
        write_f32_le(&payload[12], (float)cmd->alt_m);
        write_f32_le(&payload[16], (float)cmd->vx_cms * 0.01f);
        write_f32_le(&payload[20], (float)cmd->vy_cms * 0.01f);
        write_f32_le(&payload[24], (float)cmd->vz_cms * 0.01f);
        write_f32_le(&payload[40], ((float)cmd->heading_cdeg * 0.01f) * 0.0174532925f);
        write_u16_le(&payload[48], 0x09C0U); /* Use Pos + Vel + Yaw (ignore accel + yaw_rate) */
        payload[50] = 1U; /* target_system = 1 */
        payload[51] = 1U; /* target_component = 1 */
        payload[52] = 6U; /* MAV_FRAME_GLOBAL_RELATIVE_ALT_INT */
    } else {
        return 0U;
    }

    out_frame[0] = 0xFDU; /* MAVLink v2 STX */
    out_frame[1] = pay_len;
    out_frame[2] = 0U;    /* incompat_flags */
    out_frame[3] = 0U;    /* compat_flags */
    out_frame[4] = seq;
    out_frame[5] = 255U;  /* sysid (GCS/Companion) */
    out_frame[6] = 190U;  /* compid (MAV_COMP_ID_MISSIONPLANNER) */
    out_frame[7] = (uint8_t)(msg_id & 0xFFU);
    out_frame[8] = (uint8_t)((msg_id >> 8U) & 0xFFU);
    out_frame[9] = (uint8_t)((msg_id >> 16U) & 0xFFU);
    memcpy(&out_frame[10], payload, pay_len);

    uint16_t crc = 0xFFFFU;
    for (size_t i = 1U; i < (size_t)(10U + pay_len); i++) {
        mav_crc_accumulate(out_frame[i], &crc);
    }
    mav_crc_accumulate(crc_extra, &crc);

    out_frame[10U + pay_len] = (uint8_t)(crc & 0xFFU);
    out_frame[11U + pay_len] = (uint8_t)(crc >> 8U);
    return (size_t)(12U + pay_len);
}

/* ---------------------------------------------------------------------------
 * 4. ExpressLRS OTA DATA (ELRS_PKT_TYPE_DATA = 0x01) Fragmenter / Reassembler
 * --------------------------------------------------------------------------- */
void elrs_tunnel_init(elrs_data_tunnel_t *t, uint8_t repeats_per_frag) {
    if (t == NULL) return;
    memset(t, 0, sizeof(*t));
    t->tx_repeats_per_frag = (repeats_per_frag == 0U) ? 1U : repeats_per_frag;
}

bool elrs_tunnel_enqueue_tx(elrs_data_tunnel_t *t, const uint8_t *data, uint16_t len, bool is_ota8) {
    if (t == NULL || data == NULL || len == 0U || len > ELRS_TUNNEL_MAX_MTU || t->tx_active) {
        return false;
    }

    /* Fragment capacity:
     * - OTA8 (13-byte packet, pkt[1..10] payload):
     *     frag 0: pkt[1]=seq|0, pkt[2]=total_frags, pkt[3..4]=len16, pkt[5..10]=6 data bytes
     *     frags 1..N: pkt[1]=seq|fidx, pkt[2..10]=9 data bytes
     * - OTA4 (8-byte packet, pkt[1..6] payload):
     *     frag 0: pkt[1]=seq|0, pkt[2]=total_frags, pkt[3]=len8, pkt[4..6]=3 data bytes
     *     frags 1..N: pkt[1]=seq|fidx, pkt[2..6]=5 data bytes
     */
    uint16_t total_frags = 1U;
    if (is_ota8) {
        if (len > 6U) {
            total_frags = (uint16_t)(1U + ((len - 6U + 8U) / 9U));
        }
    } else {
        if (len > 255U) {
            return false;
        }
        if (len > 3U) {
            total_frags = (uint16_t)(1U + ((len - 3U + 4U) / 5U));
        }
    }
    if (total_frags > ELRS_TUNNEL_MAX_FRAGS) {
        return false;
    }

    memcpy(t->tx_buf, data, len);
    t->tx_len         = len;
    t->tx_seq_id      = (uint8_t)((t->tx_seq_id + 1U) & 0x03U);
    if (t->tx_seq_id == 0U) t->tx_seq_id = 1U;
    t->tx_frag_idx    = 0U;
    t->tx_total_frags = (uint8_t)total_frags;
    t->tx_repeat_rem  = t->tx_repeats_per_frag;
    t->tx_active      = true;
    return true;
}

bool elrs_tunnel_has_pending_tx(const elrs_data_tunnel_t *t) {
    return (t != NULL) && t->tx_active;
}

bool elrs_tunnel_next_tx_frame(elrs_data_tunnel_t *t,
                               const elrs_ota_state_t *ota,
                               uint8_t *out_ota_pkt) {
    if (t == NULL || ota == NULL || out_ota_pkt == NULL || !t->tx_active) {
        return false;
    }

    bool is_ota8 = ota->is_full_res;
    uint8_t fidx = t->tx_frag_idx;
    if (is_ota8) {
        memset(out_ota_pkt, 0, ELRS_OTA8_PACKET_SIZE);
        out_ota_pkt[0] = ELRS_PKT_TYPE_DATA;
        out_ota_pkt[1] = (uint8_t)(((t->tx_seq_id & 0x03U) << 6U) | (fidx & 0x3FU));
        if (fidx == 0U) {
            out_ota_pkt[2] = (uint8_t)(t->tx_total_frags & 0x3FU);
            out_ota_pkt[3] = (uint8_t)(t->tx_len >> 8U);
            out_ota_pkt[4] = (uint8_t)(t->tx_len & 0xFFU);
            uint16_t copy_len = (t->tx_len < 6U) ? t->tx_len : 6U;
            memcpy(&out_ota_pkt[5], t->tx_buf, copy_len);
        } else {
            uint16_t offset = (uint16_t)(6U + (uint16_t)(fidx - 1U) * 9U);
            if (offset < t->tx_len) {
                uint16_t rem = (uint16_t)(t->tx_len - offset);
                if (rem > 9U) rem = 9U;
                memcpy(&out_ota_pkt[2], &t->tx_buf[offset], rem);
            }
        }
    } else {
        memset(out_ota_pkt, 0, ELRS_OTA4_PACKET_SIZE);
        out_ota_pkt[0] = ELRS_PKT_TYPE_DATA;
        out_ota_pkt[1] = (uint8_t)(((t->tx_seq_id & 0x03U) << 6U) | (fidx & 0x3FU));
        if (fidx == 0U) {
            out_ota_pkt[2] = (uint8_t)(t->tx_total_frags & 0x3FU);
            out_ota_pkt[3] = (uint8_t)(t->tx_len & 0xFFU);
            uint16_t copy_len = (t->tx_len < 3U) ? t->tx_len : 3U;
            memcpy(&out_ota_pkt[4], t->tx_buf, copy_len);
        } else {
            uint16_t offset = (uint16_t)(3U + (uint16_t)(fidx - 1U) * 5U);
            if (offset < t->tx_len) {
                uint16_t rem = (uint16_t)(t->tx_len - offset);
                if (rem > 5U) rem = 5U;
                memcpy(&out_ota_pkt[2], &t->tx_buf[offset], rem);
            }
        }
    }
    elrs_ota_generate_crc(ota, out_ota_pkt, ota->ota_nonce);

    if (t->tx_repeat_rem > 1U) {
        t->tx_repeat_rem--;
    } else {
        t->tx_frag_idx++;
        if (t->tx_frag_idx >= t->tx_total_frags) {
            t->tx_active = false;
            t->tx_msgs_sent++;
        } else {
            t->tx_repeat_rem = t->tx_repeats_per_frag;
        }
    }
    return true;
}

bool elrs_tunnel_process_rx_frame(elrs_data_tunnel_t *t,
                                  const uint8_t *ota_pkt,
                                  bool is_ota8,
                                  uint8_t *out_msg,
                                  uint16_t *out_msg_len) {
    if (t == NULL || ota_pkt == NULL || (ota_pkt[0] & 0x03U) != ELRS_PKT_TYPE_DATA) {
        return false;
    }

    uint8_t seq_id = (uint8_t)((ota_pkt[1] >> 6U) & 0x03U);
    uint8_t fval   = (uint8_t)(ota_pkt[1] & 0x3FU);
    if (seq_id == 0U) {
        return false;
    }
    if (seq_id == t->last_completed_seq_id && fval != 0U) {
        return false;
    }

    if (fval == 0U) {
        if (seq_id == t->last_completed_seq_id && t->rx_total_frags == 1U) {
            return false;
        }
        uint8_t total_frags = (uint8_t)(ota_pkt[2] & 0x3FU);
        if (total_frags == 0U || total_frags > ELRS_TUNNEL_MAX_FRAGS) {
            return false;
        }
        uint16_t exp_len = is_ota8 ? (uint16_t)(((uint16_t)ota_pkt[3] << 8U) | ota_pkt[4])
                                   : (uint16_t)ota_pkt[3];
        if (exp_len == 0U || exp_len > ELRS_TUNNEL_MAX_MTU) {
            return false;
        }
        if (!t->rx_in_progress || t->rx_seq_id != seq_id || (t->rx_frag_bitmap & ~1ULL) != 0ULL) {
            memset(t->rx_buf, 0, sizeof(t->rx_buf));
            t->rx_seq_id             = seq_id;
            t->rx_total_frags        = total_frags;
            t->rx_expected_len       = exp_len;
            t->rx_frag_bitmap        = 0ULL;
            t->rx_in_progress        = true;
            t->last_completed_seq_id = 0U;
        }
        uint16_t copy_len = is_ota8 ? ((exp_len < 6U) ? exp_len : 6U)
                                    : ((exp_len < 3U) ? exp_len : 3U);
        memcpy(t->rx_buf, is_ota8 ? &ota_pkt[5] : &ota_pkt[4], copy_len);
        t->rx_frag_bitmap |= 1ULL;
    } else {
        if (!t->rx_in_progress || t->rx_seq_id != seq_id || fval >= t->rx_total_frags) {
            return false;
        }
        uint16_t offset = is_ota8 ? (uint16_t)(6U + (uint16_t)(fval - 1U) * 9U)
                                  : (uint16_t)(3U + (uint16_t)(fval - 1U) * 5U);
        uint16_t chunk_max = is_ota8 ? 9U : 5U;
        if (offset < t->rx_expected_len) {
            uint16_t rem = (uint16_t)(t->rx_expected_len - offset);
            if (rem > chunk_max) rem = chunk_max;
            memcpy(&t->rx_buf[offset], &ota_pkt[2], rem);
        }
        t->rx_frag_bitmap |= (1ULL << fval);
    }

    uint64_t full_mask = (t->rx_total_frags == 64U) ? ~0ULL : ((1ULL << t->rx_total_frags) - 1ULL);
    if ((t->rx_frag_bitmap & full_mask) == full_mask) {
        t->rx_in_progress = false;
        t->last_completed_seq_id = seq_id;
        t->rx_msgs_completed++;
        if (out_msg != NULL) {
            memcpy(out_msg, t->rx_buf, t->rx_expected_len);
        }
        if (out_msg_len != NULL) {
            *out_msg_len = t->rx_expected_len;
        }
        return true;
    }
    return false;
}

/* ---------------------------------------------------------------------------
 * 5. Per-Node Reticulum Identity, Peer Table, OTA Pairing & Targeted Commands
 * --------------------------------------------------------------------------- */
static const uint32_t SHA256_K[64] = {
    0x428a2f98UL, 0x71374491UL, 0xb5c0fbcfUL, 0xe9b5dba5UL,
    0x3956c25bUL, 0x59f111f1UL, 0x923f82a4UL, 0xab1c5ed5UL,
    0xd807aa98UL, 0x12835b01UL, 0x243185beUL, 0x550c7dc3UL,
    0x72be5d74UL, 0x80deb1feUL, 0x9bdc06a7UL, 0xc19bf174UL,
    0xe49b69c1UL, 0xefbe4786UL, 0x0fc19dc6UL, 0x240ca1ccUL,
    0x2de92c6fUL, 0x4a7484aaUL, 0x5cb0a9dcUL, 0x76f988daUL,
    0x983e5152UL, 0xa831c66dUL, 0xb00327c8UL, 0xbf597fc7UL,
    0xc6e00bf3UL, 0xd5a79147UL, 0x06ca6351UL, 0x14292967UL,
    0x27b70a85UL, 0x2e1b2138UL, 0x4d2c6dfcUL, 0x53380d13UL,
    0x650a7354UL, 0x766a0abbUL, 0x81c2c92eUL, 0x92722c85UL,
    0xa2bfe8a1UL, 0xa81a664bUL, 0xc24b8b70UL, 0xc76c51a3UL,
    0xd192e819UL, 0xd6990624UL, 0xf40e3585UL, 0x106aa070UL,
    0x19a4c116UL, 0x1e376c08UL, 0x2748774cUL, 0x34b0bcb5UL,
    0x391c0cb3UL, 0x4ed8aa4aUL, 0x5b9cca4fUL, 0x682e6ff3UL,
    0x748f82eeUL, 0x78a5636fUL, 0x84c87814UL, 0x8cc70208UL,
    0x90befffaUL, 0xa4506cebUL, 0xbef9a3f7UL, 0xc67178f2UL
};

static inline uint32_t rotr32(uint32_t x, uint32_t n) {
    return (x >> n) | (x << (32U - n));
}

static void sha256_transform(uint32_t state[8], const uint8_t block[64]) {
    uint32_t w[64];
    for (uint8_t i = 0U; i < 16U; i++) {
        w[i] = ((uint32_t)block[i * 4U] << 24U) |
               ((uint32_t)block[i * 4U + 1U] << 16U) |
               ((uint32_t)block[i * 4U + 2U] << 8U) |
               ((uint32_t)block[i * 4U + 3U]);
    }
    for (uint8_t i = 16U; i < 64U; i++) {
        uint32_t s0 = rotr32(w[i - 15U], 7U) ^ rotr32(w[i - 15U], 18U) ^ (w[i - 15U] >> 3U);
        uint32_t s1 = rotr32(w[i - 2U], 17U) ^ rotr32(w[i - 2U], 19U) ^ (w[i - 2U] >> 10U);
        w[i] = w[i - 16U] + s0 + w[i - 7U] + s1;
    }

    uint32_t a = state[0], b = state[1], c = state[2], d = state[3];
    uint32_t e = state[4], f = state[5], g = state[6], h = state[7];

    for (uint8_t i = 0U; i < 64U; i++) {
        uint32_t S1 = rotr32(e, 6U) ^ rotr32(e, 11U) ^ rotr32(e, 25U);
        uint32_t ch = (e & f) ^ ((~e) & g);
        uint32_t t1 = h + S1 + ch + SHA256_K[i] + w[i];
        uint32_t S0 = rotr32(a, 2U) ^ rotr32(a, 13U) ^ rotr32(a, 22U);
        uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
        uint32_t t2 = S0 + maj;

        h = g;
        g = f;
        f = e;
        e = d + t1;
        d = c;
        c = b;
        b = a;
        a = t1 + t2;
    }

    state[0] += a; state[1] += b; state[2] += c; state[3] += d;
    state[4] += e; state[5] += f; state[6] += g; state[7] += h;
}

void rns_sha256(const uint8_t *data, size_t len, uint8_t out_hash[32]) {
    uint32_t state[8] = {
        0x6a09e667UL, 0xbb67ae85UL, 0x3c6ef372UL, 0xa54ff53aUL,
        0x510e527fUL, 0x9b05688cUL, 0x1f83d9abUL, 0x5be0cd19UL
    };
    uint8_t block[64];
    size_t offset = 0U;
    while (len - offset >= 64U) {
        sha256_transform(state, data + offset);
        offset += 64U;
    }
    size_t rem = len - offset;
    memset(block, 0, sizeof(block));
    if (rem > 0U && data != NULL) {
        memcpy(block, data + offset, rem);
    }
    block[rem] = 0x80U;
    if (rem >= 56U) {
        sha256_transform(state, block);
        memset(block, 0, sizeof(block));
    }
    uint64_t bit_len = (uint64_t)len * 8ULL;
    for (uint8_t i = 0U; i < 8U; i++) {
        block[63U - i] = (uint8_t)((bit_len >> (i * 8U)) & 0xFFULL);
    }
    sha256_transform(state, block);

    for (uint8_t i = 0U; i < 8U; i++) {
        out_hash[i * 4U]      = (uint8_t)((state[i] >> 24U) & 0xFFU);
        out_hash[i * 4U + 1U] = (uint8_t)((state[i] >> 16U) & 0xFFU);
        out_hash[i * 4U + 2U] = (uint8_t)((state[i] >> 8U) & 0xFFU);
        out_hash[i * 4U + 3U] = (uint8_t)(state[i] & 0xFFU);
    }
}

void rns_identity_init(rns_node_identity_t *id,
                       const uint8_t hw_device_id[8],
                       const uint8_t elrs_uid[6],
                       uint8_t role,
                       const char *name) {
    if (id == NULL) return;
    memset(id, 0, sizeof(*id));
    if (hw_device_id != NULL) {
        memcpy(id->hw_device_id, hw_device_id, 8U);
    }
    if (elrs_uid != NULL) {
        memcpy(id->elrs_uid, elrs_uid, 6U);
    }
    id->role = role;
    if (name != NULL) {
        strncpy(id->name, name, sizeof(id->name) - 1U);
    }

    /* Derive 64-byte public identity material (X25519 32B + Ed25519 32B) */
    uint8_t seed_buf[24];
    memcpy(seed_buf, "RNS-X25519", 10U);
    memcpy(&seed_buf[10], id->hw_device_id, 8U);
    memcpy(&seed_buf[18], id->elrs_uid, 6U);
    rns_sha256(seed_buf, sizeof(seed_buf), &id->pub_key[0]);

    memcpy(seed_buf, "RNS-ED2551", 10U);
    rns_sha256(seed_buf, sizeof(seed_buf), &id->pub_key[32]);

    /* Reticulum identity_hash = SHA256(pub_key)[:16] */
    uint8_t full_hash[32];
    rns_sha256(id->pub_key, 64U, full_hash);
    memcpy(id->identity_hash, full_hash, 16U);

    /* Reticulum SINGLE destination hash:
     * name_hash = SHA256("elrs.rns.node")[:10]
     * dest_hash = SHA256(name_hash || identity_hash)[:16]
     */
    uint8_t name_full[32];
    const char *app_aspect = "elrs.rns.node";
    rns_sha256((const uint8_t *)app_aspect, strlen(app_aspect), name_full);

    uint8_t addr_material[26];
    memcpy(addr_material, name_full, 10U);
    memcpy(&addr_material[10], id->identity_hash, 16U);
    rns_sha256(addr_material, sizeof(addr_material), full_hash);
    memcpy(id->dest_hash, full_hash, 16U);
}

void rns_identity_set_ifac(rns_node_identity_t *id, bool enabled, const char *passphrase) {
    if (id == NULL) return;
    id->ifac_enabled = enabled;
    memset(id->ifac_pass, 0, sizeof(id->ifac_pass));
    if (passphrase != NULL) {
        strncpy(id->ifac_pass, passphrase, sizeof(id->ifac_pass) - 1U);
    }
    uint8_t full[32];
    rns_sha256((const uint8_t *)id->ifac_pass, strlen(id->ifac_pass), full);
    memcpy(id->ifac_key, full, 16U);
}

void rns_peer_table_init(rns_peer_table_t *pt) {
    if (pt == NULL) return;
    memset(pt, 0, sizeof(*pt));
    pt->selected_idx = -1;
}

size_t rns_build_node_announce(const rns_node_identity_t *id,
                               uint32_t freq_hz,
                               uint32_t bw_hz,
                               uint8_t sf,
                               uint8_t cr,
                               const rns_fc_telemetry_t *tlm,
                               uint8_t *out_pkt,
                               size_t max_len) {
    if (id == NULL || out_pkt == NULL || max_len < 68U) {
        return 0U;
    }
    memset(out_pkt, 0, 68U);
    out_pkt[0] = 0x01U; /* HEADER_1 | BROADCAST | SINGLE | ANNOUNCE */
    out_pkt[1] = 0x00U; /* hops = 0 */
    memcpy(&out_pkt[2], id->dest_hash, 16U);
    out_pkt[18] = 0x00U; /* context */

    /* Payload [19..67]:
     * [19..22] "RNSA" magic (4B)
     * [23..38] identity_hash (16B)
     * [39]     role (1B)
     * [40..51] node name (12B)
     * [52..55] freq_hz (4B LE)
     * [56..59] bw_hz (4B LE)
     * [60]     sf (1B)
     * [61]     cr (1B)
     * [62..63] vbatt_mv (2B LE)
     * [64]     custom_mode (1B)
     * [65]     armed (1B)
     * [66..67] 2B SHA-256 auth tag
     */
    memcpy(&out_pkt[19], "RNSA", 4U);
    memcpy(&out_pkt[23], id->identity_hash, 16U);
    out_pkt[39] = id->role;
    memcpy(&out_pkt[40], id->name, 12U);
    write_u32_le(&out_pkt[52], freq_hz);
    write_u32_le(&out_pkt[56], bw_hz);
    out_pkt[60] = sf;
    out_pkt[61] = cr;
    write_u16_le(&out_pkt[62], tlm ? tlm->vbatt_mv : 0U);
    out_pkt[64] = tlm ? tlm->custom_mode : 0U;
    out_pkt[65] = (tlm && tlm->armed) ? 1U : 0U;

    uint8_t digest[32];
    rns_sha256(&out_pkt[2], 64U, digest);
    out_pkt[66] = digest[0];
    out_pkt[67] = digest[1];
    return 68U;
}

bool rns_parse_node_announce(const uint8_t *pkt,
                             size_t len,
                             int8_t rssi_dbm,
                             int8_t snr_db,
                             uint32_t now_ms,
                             rns_peer_table_t *pt) {
    if (pkt == NULL || pt == NULL || len < 19U) {
        return false;
    }
    /* Match either compact 68B RNSA announce OR standard RNS ANNOUNCE (pkt[0] & 0x03 == 0x01) */
    if ((pkt[0] & 0x03U) != 0x01U) {
        return false;
    }

    uint8_t dest_hash[16];
    memcpy(dest_hash, &pkt[2], 16U);

    bool is_rnsa = (len >= 68U && memcmp(&pkt[19], "RNSA", 4U) == 0);
    if (is_rnsa) {
        uint8_t digest[32];
        rns_sha256(&pkt[2], 64U, digest);
        if (pkt[66] != digest[0] || pkt[67] != digest[1]) {
            return false;
        }
    }

    /* Find existing peer slot or allocate a slot */
    int slot = -1;
    for (uint8_t i = 0U; i < pt->count; i++) {
        if (pt->peers[i].valid && memcmp(pt->peers[i].dest_hash, dest_hash, 16U) == 0) {
            slot = (int)i;
            break;
        }
    }
    if (slot < 0) {
        if (pt->count < RNS_MAX_PEERS) {
            slot = (int)pt->count++;
        } else {
            /* Evict oldest peer */
            slot = 0;
            uint32_t oldest_age = 0U;
            for (uint8_t i = 0U; i < RNS_MAX_PEERS; i++) {
                uint32_t age = now_ms - pt->peers[i].last_seen_ms;
                if (age >= oldest_age) {
                    oldest_age = age;
                    slot = (int)i;
                }
            }
        }
    }

    rns_peer_entry_t *e = &pt->peers[slot];
    e->valid = true;
    memcpy(e->dest_hash, dest_hash, 16U);
    e->hops         = pkt[1];
    e->rssi_dbm     = rssi_dbm;
    e->snr_db       = snr_db;
    e->last_seen_ms = now_ms;

    if (is_rnsa) {
        memcpy(e->identity_hash, &pkt[23], 16U);
        e->role = pkt[39];
        memset(e->name, 0, sizeof(e->name));
        memcpy(e->name, &pkt[40], 12U);
        e->freq_hz     = read_u32_le(&pkt[52]);
        e->bw_hz       = read_u32_le(&pkt[56]);
        e->sf          = pkt[60];
        e->cr          = pkt[61];
        e->vbatt_mv    = read_u16_le(&pkt[62]);
        e->custom_mode = pkt[64];
        e->armed       = (pkt[65] != 0U);
    } else if (len >= 83U) {
        /* Standard Reticulum ANNOUNCE: [19..82] is 64B public key */
        uint8_t id_full[32];
        rns_sha256(&pkt[19], 64U, id_full);
        memcpy(e->identity_hash, id_full, 16U);
        e->role = RNS_NODE_ROLE_RNODE;
        snprintf(e->name, sizeof(e->name), "RNS-%02X%02X", dest_hash[0], dest_hash[1]);
    }

    if (pt->selected_idx < 0) {
        pt->selected_idx = (int8_t)slot;
    }
    return true;
}

/* SHA256(SHA256("elrs.rns.pair")[:10])[:16] broadcast pairing destination */
static const uint8_t RNS_DEST_PAIRING_BCAST[16] = {
    0x4a, 0x91, 0xc3, 0x0e, 0x55, 0xb2, 0x7d, 0x84,
    0x19, 0x6f, 0xe0, 0x3b, 0xa8, 0x12, 0x9c, 0x5d
};

static bool is_all_zeros_16(const uint8_t b[16]) {
    for (uint8_t i = 0U; i < 16U; i++) {
        if (b[i] != 0U) return false;
    }
    return true;
}

size_t rns_build_pairing_pkt(const rns_pairing_cfg_t *cfg,
                             uint8_t *out_pkt,
                             size_t max_len) {
    if (cfg == NULL || out_pkt == NULL || max_len < 68U) {
        return 0U;
    }
    memset(out_pkt, 0, 68U);
    bool bcast = is_all_zeros_16(cfg->target_dest);
    out_pkt[0] = bcast ? 0x08U : 0x00U; /* PLAIN DATA or SINGLE DATA */
    out_pkt[1] = 0x00U;                 /* hops = 0 */
    memcpy(&out_pkt[2], bcast ? RNS_DEST_PAIRING_BCAST : cfg->target_dest, 16U);
    out_pkt[18] = 0xA5U;                /* Pairing context */

    memcpy(&out_pkt[19], "RNSP", 4U);
    memcpy(&out_pkt[23], cfg->tx_dest, 16U);
    write_u32_le(&out_pkt[39], cfg->freq_hz);
    write_u32_le(&out_pkt[43], cfg->bw_hz);
    out_pkt[47] = cfg->sf;
    out_pkt[48] = cfg->cr;
    out_pkt[49] = (uint8_t)cfg->txpower_dbm;
    out_pkt[50] = cfg->ifac_enabled ? 1U : 0U;
    memcpy(&out_pkt[51], cfg->ifac_pass, 15U);

    uint8_t digest[32];
    rns_sha256(&out_pkt[2], 64U, digest);
    out_pkt[66] = digest[0];
    out_pkt[67] = digest[1];
    return 68U;
}

bool rns_parse_pairing_pkt(const uint8_t *pkt,
                           size_t len,
                           const uint8_t local_dest[16],
                           rns_pairing_cfg_t *out_cfg) {
    if (pkt == NULL || out_cfg == NULL || len < 68U) {
        return false;
    }
    if (pkt[18] != 0xA5U || memcmp(&pkt[19], "RNSP", 4U) != 0) {
        return false;
    }
    uint8_t digest[32];
    rns_sha256(&pkt[2], 64U, digest);
    if (pkt[66] != digest[0] || pkt[67] != digest[1]) {
        return false;
    }

    bool match_bcast = (memcmp(&pkt[2], RNS_DEST_PAIRING_BCAST, 16U) == 0);
    bool match_local = (local_dest != NULL && memcmp(&pkt[2], local_dest, 16U) == 0);
    if (!match_bcast && !match_local) {
        return false;
    }

    memset(out_cfg, 0, sizeof(*out_cfg));
    if (match_local) {
        memcpy(out_cfg->target_dest, &pkt[2], 16U);
    }
    memcpy(out_cfg->tx_dest, &pkt[23], 16U);
    out_cfg->freq_hz      = read_u32_le(&pkt[39]);
    out_cfg->bw_hz        = read_u32_le(&pkt[43]);
    out_cfg->sf           = pkt[47];
    out_cfg->cr           = pkt[48];
    out_cfg->txpower_dbm  = (int8_t)pkt[49];
    out_cfg->ifac_enabled = (pkt[50] != 0U);
    memcpy(out_cfg->ifac_pass, &pkt[51], 15U);
    out_cfg->ifac_pass[15] = '\0';
    return true;
}

size_t rns_build_targeted_cmd_pkt(const uint8_t target_dest[16],
                                  const uint8_t sender_dest[16],
                                  uint8_t seq,
                                  const char *cmd_str,
                                  uint8_t *out_pkt,
                                  size_t max_len) {
    if (target_dest == NULL || sender_dest == NULL || cmd_str == NULL || out_pkt == NULL) {
        return 0U;
    }
    size_t slen = strlen(cmd_str);
    if (40U + slen > max_len) {
        return 0U;
    }
    bool bcast = is_all_zeros_16(target_dest);
    out_pkt[0] = bcast ? 0x08U : 0x00U; /* HEADER_1 | SINGLE/PLAIN | DATA */
    out_pkt[1] = 0x00U;                 /* hops = 0 */
    memcpy(&out_pkt[2], bcast ? RNS_DEST_DRONE_CMD : target_dest, 16U);
    out_pkt[18] = 0xA6U;                /* Targeted command context */
    memcpy(&out_pkt[19], "RNSC", 4U);
    memcpy(&out_pkt[23], sender_dest, 16U);
    out_pkt[39] = seq;
    memcpy(&out_pkt[40], cmd_str, slen);
    return 40U + slen;
}

bool rns_parse_targeted_cmd_pkt(const uint8_t *pkt,
                                size_t len,
                                const uint8_t local_dest[16],
                                uint8_t out_sender_dest[16],
                                uint8_t *out_seq,
                                char *out_cmd_str,
                                size_t max_cmd_str) {
    if (pkt == NULL || len <= 40U || pkt[18] != 0xA6U || memcmp(&pkt[19], "RNSC", 4U) != 0) {
        return false;
    }
    bool match_bcast = (memcmp(&pkt[2], RNS_DEST_DRONE_CMD, 16U) == 0);
    bool match_local = (local_dest != NULL && memcmp(&pkt[2], local_dest, 16U) == 0);
    if (!match_bcast && !match_local) {
        return false;
    }
    if (out_sender_dest != NULL) {
        memcpy(out_sender_dest, &pkt[23], 16U);
    }
    if (out_seq != NULL) {
        *out_seq = pkt[39];
    }
    if (out_cmd_str != NULL && max_cmd_str > 0U) {
        size_t slen = len - 40U;
        if (slen >= max_cmd_str) {
            slen = max_cmd_str - 1U;
        }
        memcpy(out_cmd_str, &pkt[40], slen);
        out_cmd_str[slen] = '\0';
    }
    return true;
}

/* ---------------------------------------------------------------------------
 * 6. Multi-Drone Formation & Mesh Control Distribution ("RNSF" context 0xA7)
 * --------------------------------------------------------------------------- */
size_t rns_build_formation_pkt(const rns_formation_pkt_t *fp,
                               uint8_t *out_pkt,
                               size_t max_len) {
    if (fp == NULL || out_pkt == NULL || max_len < 64U) {
        return 0U;
    }
    memset(out_pkt, 0, 64U);
    out_pkt[0] = 0x08U; /* HEADER_1 | BROADCAST | PLAIN | DATA */
    out_pkt[1] = 0x00U; /* hops = 0 */
    memcpy(&out_pkt[2], RNS_DEST_DRONE_CMD, 16U);
    out_pkt[18] = 0xA7U; /* Formation context */

    memcpy(&out_pkt[19], "RNSF", 4U);
    memcpy(&out_pkt[23], fp->leader_dest, 16U);
    out_pkt[39] = fp->seq;
    out_pkt[40] = fp->shape;
    write_u16_le(&out_pkt[41], fp->spacing_dm);
    write_u32_le(&out_pkt[43], (uint32_t)fp->leader_lat_e7);
    write_u32_le(&out_pkt[47], (uint32_t)fp->leader_lon_e7);
    write_u32_le(&out_pkt[51], (uint32_t)fp->leader_alt_mm);
    write_u16_le(&out_pkt[55], (uint16_t)fp->vx_cms);
    write_u16_le(&out_pkt[57], (uint16_t)fp->vy_cms);
    write_u16_le(&out_pkt[59], (uint16_t)fp->vz_cms);
    /* Store heading in [61] as 2-deg steps or use [59..60] if needed; wait:
     * Let's store leader_alt_dm in 2B [51..52] (0.1m units from leader_alt_mm / 100),
     * vx_cms [53..54], vy_cms [55..56], vz_cms [57..58], heading_cdeg [59..60]!
     */
    int16_t alt_dm = (int16_t)(fp->leader_alt_mm / 100);
    write_u16_le(&out_pkt[51], (uint16_t)alt_dm);
    write_u16_le(&out_pkt[53], (uint16_t)fp->vx_cms);
    write_u16_le(&out_pkt[55], (uint16_t)fp->vy_cms);
    write_u16_le(&out_pkt[57], (uint16_t)fp->vz_cms);
    write_u16_le(&out_pkt[59], fp->heading_cdeg);
    out_pkt[61] = 0U;

    /* 2-byte SHA-256 integrity tag over [2..61] (excludes hop count at [1] so relays can increment hops) */
    uint8_t digest[32];
    rns_sha256(&out_pkt[2], 60U, digest);
    out_pkt[62] = digest[0];
    out_pkt[63] = digest[1];
    return 64U;
}

bool rns_parse_formation_pkt(const uint8_t *pkt,
                             size_t len,
                             rns_formation_pkt_t *out_fp) {
    if (pkt == NULL || out_fp == NULL || len < 64U) {
        return false;
    }
    if (pkt[18] != 0xA7U || memcmp(&pkt[19], "RNSF", 4U) != 0 ||
        memcmp(&pkt[2], RNS_DEST_DRONE_CMD, 16U) != 0) {
        return false;
    }
    uint8_t digest[32];
    rns_sha256(&pkt[2], 60U, digest);
    if (pkt[62] != digest[0] || pkt[63] != digest[1]) {
        return false;
    }

    memset(out_fp, 0, sizeof(*out_fp));
    memcpy(out_fp->leader_dest, &pkt[23], 16U);
    out_fp->seq           = pkt[39];
    out_fp->shape         = pkt[40];
    out_fp->spacing_dm    = read_u16_le(&pkt[41]);
    out_fp->leader_lat_e7 = (int32_t)read_u32_le(&pkt[43]);
    out_fp->leader_lon_e7 = (int32_t)read_u32_le(&pkt[47]);
    out_fp->leader_alt_mm = (int32_t)((int16_t)read_u16_le(&pkt[51])) * 100L;
    out_fp->vx_cms        = (int16_t)read_u16_le(&pkt[53]);
    out_fp->vy_cms        = (int16_t)read_u16_le(&pkt[55]);
    out_fp->vz_cms        = (int16_t)read_u16_le(&pkt[57]);
    out_fp->heading_cdeg  = read_u16_le(&pkt[59]);
    return true;
}

bool rns_compute_formation_slot_cmd(const rns_formation_pkt_t *fp,
                                    uint8_t slot_id,
                                    rns_drone_cmd_t *out_cmd) {
    if (fp == NULL || out_cmd == NULL) {
        return false;
    }
    memset(out_cmd, 0, sizeof(*out_cmd));

    if (fp->shape == RNS_FORM_HOLD_BRAKE) {
        /* Immediate zero-velocity brake in body/NED frame */
        out_cmd->type           = RNS_DRONE_CMD_VEL_NED;
        out_cmd->vx_cms         = 0;
        out_cmd->vy_cms         = 0;
        out_cmd->vz_cms         = 0;
        out_cmd->yaw_rate_cdps  = 0;
        out_cmd->has_yaw_rate   = false;
        out_cmd->use_body_frame = true;
        return true;
    }

    /* Compute slot offset in Leader body frame (x_fwd_m, y_right_m) */
    float spacing_m = (float)fp->spacing_dm * 0.1f;
    float x_fwd_m   = 0.0f;
    float y_right_m = 0.0f;

    if (slot_id > 0U) {
        uint8_t rank = (uint8_t)((slot_id + 1U) / 2U);
        float side   = ((slot_id & 1U) != 0U) ? -1.0f : +1.0f; /* odd=left (-Y), even=right (+Y) */

        switch (fp->shape) {
            case RNS_FORM_VEE:
                x_fwd_m   = -(float)rank * spacing_m;
                y_right_m = side * (float)rank * spacing_m;
                break;
            case RNS_FORM_LINE_ABREAST:
                x_fwd_m   = 0.0f;
                y_right_m = side * (float)rank * spacing_m;
                break;
            case RNS_FORM_COLUMN:
            default:
                x_fwd_m   = -(float)slot_id * spacing_m;
                y_right_m = 0.0f;
                break;
        }
    }

    /* Rotate body offset (x_fwd_m, y_right_m) by leader heading into NED (offset_n_m, offset_e_m) */
    float hdg_rad = ((float)fp->heading_cdeg * 0.01f) * (3.1415926535f / 180.0f);
    float cos_h   = cosf(hdg_rad);
    float sin_h   = sinf(hdg_rad);

    float offset_n_m = x_fwd_m * cos_h - y_right_m * sin_h;
    float offset_e_m = x_fwd_m * sin_h + y_right_m * cos_h;

    /* Convert NED meters to degE7:
     * 1 deg latitude ~= 111,319.5 m -> 1 m ~= 89.8315 degE7
     * 1 deg longitude ~= 111,319.5 * cos(lat) m
     */
    float lat_rad = ((float)fp->leader_lat_e7 * 1e-7f) * (3.1415926535f / 180.0f);
    float cos_lat = cosf(lat_rad);
    if (cos_lat < 0.1f) {
        cos_lat = 0.1f;
    }
    int32_t dlat_e7 = (int32_t)(offset_n_m * 89.831528f);
    int32_t dlon_e7 = (int32_t)((offset_e_m * 89.831528f) / cos_lat);

    out_cmd->type         = RNS_DRONE_CMD_GLOBAL_POS_VEL;
    out_cmd->lat_deg_e7   = fp->leader_lat_e7 + dlat_e7;
    out_cmd->lon_deg_e7   = fp->leader_lon_e7 + dlon_e7;
    out_cmd->alt_m        = fp->leader_alt_mm / 1000L;
    out_cmd->vx_cms       = fp->vx_cms;
    out_cmd->vy_cms       = fp->vy_cms;
    out_cmd->vz_cms       = fp->vz_cms;
    out_cmd->heading_cdeg = fp->heading_cdeg;
    return true;
}

