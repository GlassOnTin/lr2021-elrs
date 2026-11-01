/* SPDX-License-Identifier: AGPL-3.0-or-later */
/*
 * Reticulum / RNode KISS interface, Airborne Store-and-Forward Relay,
 * MAVLink Flight Controller Bridge, and ExpressLRS OTA DATA Tunnel.
 */

#include "rns_node.h"
#include "../elrs/elrs_protocol.h"

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
    if (strncmp(buf, "CH ", 3) == 0) {
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

    uint8_t payload[37];
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
    if (seq_id == 0U || seq_id == t->last_completed_seq_id) {
        return false;
    }

    if (fval == 0U) {
        uint8_t total_frags = (uint8_t)(ota_pkt[2] & 0x3FU);
        if (total_frags == 0U || total_frags > ELRS_TUNNEL_MAX_FRAGS) {
            return false;
        }
        uint16_t exp_len = is_ota8 ? (uint16_t)(((uint16_t)ota_pkt[3] << 8U) | ota_pkt[4])
                                   : (uint16_t)ota_pkt[3];
        if (exp_len == 0U || exp_len > ELRS_TUNNEL_MAX_MTU) {
            return false;
        }
        if (!t->rx_in_progress || t->rx_seq_id != seq_id) {
            memset(t->rx_buf, 0, sizeof(t->rx_buf));
            t->rx_seq_id       = seq_id;
            t->rx_total_frags  = total_frags;
            t->rx_expected_len = exp_len;
            t->rx_frag_bitmap  = 0ULL;
            t->rx_in_progress  = true;
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
