/* SPDX-License-Identifier: AGPL-3.0-or-later */
/*
 * Reticulum / RNode KISS interface, Airborne Store-and-Forward Relay,
 * MAVLink Flight Controller Bridge, and ExpressLRS OTA DATA Tunnel.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "../elrs/elrs_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------------
 * 1. RNode KISS Protocol Constants & Framing
 * --------------------------------------------------------------------------- */
#define RNODE_FEND              0xC0U
#define RNODE_FESC              0xDBU
#define RNODE_TFEND             0xDCU
#define RNODE_TFESC             0xDDU

#define RNODE_CMD_DATA          0x00U
#define RNODE_CMD_FREQUENCY     0x01U
#define RNODE_CMD_BANDWIDTH     0x02U
#define RNODE_CMD_TXPOWER       0x03U
#define RNODE_CMD_SF            0x04U
#define RNODE_CMD_CR            0x05U
#define RNODE_CMD_RADIO_STATE   0x06U
#define RNODE_CMD_RADIO_LOCK    0x07U
#define RNODE_CMD_DETECT        0x08U
#define RNODE_CMD_IMPLICIT      0x09U
#define RNODE_CMD_LEAVE         0x0AU
#define RNODE_CMD_ST_ALOCK      0x0BU
#define RNODE_CMD_LT_ALOCK      0x0CU
#define RNODE_CMD_PROMISC       0x0EU
#define RNODE_CMD_READY         0x0FU

#define RNODE_CMD_STAT_RX       0x21U
#define RNODE_CMD_STAT_TX       0x22U
#define RNODE_CMD_STAT_RSSI     0x23U
#define RNODE_CMD_STAT_SNR      0x24U
#define RNODE_CMD_STAT_CHTM     0x25U
#define RNODE_CMD_STAT_PHYPRM   0x26U
#define RNODE_CMD_STAT_BAT      0x27U
#define RNODE_CMD_STAT_CSMA     0x28U
#define RNODE_CMD_STAT_TEMP     0x29U
#define RNODE_CMD_BLINK         0x30U
#define RNODE_CMD_RANDOM        0x40U
#define RNODE_CMD_BOARD         0x47U
#define RNODE_CMD_PLATFORM      0x48U
#define RNODE_CMD_MCU           0x49U
#define RNODE_CMD_FW_VERSION    0x50U
#define RNODE_CMD_ROM_READ      0x51U
#define RNODE_CMD_RESET         0x55U
#define RNODE_CMD_ERROR         0x90U

#define RNODE_DETECT_REQ        0x73U
#define RNODE_DETECT_RESP       0x46U

#define RNODE_RADIO_STATE_OFF   0x00U
#define RNODE_RADIO_STATE_ON    0x01U

#define RNODE_PLATFORM_HMBRW    0xF0U
#define RNODE_MCU_NRF54         0x81U
#define RNODE_BOARD_LR2021      0x4CU
#define RNODE_FW_MAJ            1U
#define RNODE_FW_MIN            63U

#define RNODE_ERR_INITRADIO     0x01U
#define RNODE_ERR_TXFAILED      0x02U
#define RNODE_ERR_QUEUE_FULL    0x04U

#define RNODE_HW_MTU            508U
#define RNODE_LORA_MAX_PKT      255U
#define RNODE_RSSI_OFFSET       157
#define RNODE_KISS_BUF_SIZE     600U

typedef enum {
    RNODE_KISS_WAIT_FEND = 0,
    RNODE_KISS_WAIT_CMD,
    RNODE_KISS_IN_FRAME,
    RNODE_KISS_ESCAPE
} rnode_kiss_state_t;

typedef void (*rnode_kiss_frame_cb_t)(uint8_t cmd, const uint8_t *data, size_t len, void *user_ctx);

typedef struct {
    rnode_kiss_state_t    state;
    uint8_t               cmd;
    uint8_t               buf[RNODE_KISS_BUF_SIZE];
    size_t                pos;
    rnode_kiss_frame_cb_t callback;
    void                 *user_ctx;
} rnode_kiss_parser_t;

void rnode_kiss_init(rnode_kiss_parser_t *p, rnode_kiss_frame_cb_t cb, void *user_ctx);
void rnode_kiss_feed_byte(rnode_kiss_parser_t *p, uint8_t b);
size_t rnode_kiss_build_frame(uint8_t *out, size_t out_size, uint8_t cmd, const uint8_t *data, size_t len);
size_t rnode_build_phyprm(uint32_t bw_hz, uint8_t sf, uint16_t preamble_sym,
                          uint16_t *out_slot_ms, uint16_t *out_difs_ms, uint8_t out_buf[12]);

/* ---------------------------------------------------------------------------
 * 2. Reticulum Airborne Store-and-Forward Relay & Deduplicator
 * --------------------------------------------------------------------------- */
#define RNS_MIN_PACKET_LEN      19U
#define RNS_DEDUP_RING_SIZE     64U
#define RNS_DEDUP_WINDOW_MS     10000UL

/* Pre-computed RNS PLAIN destination hashes:
 * - "rnlog.beacon"   : SHA256(SHA256("rnlog.beacon")[:10])[:16]
 * - "elrs.drone.cmd" : SHA256(SHA256("elrs.drone.cmd")[:10])[:16]
 */
extern const uint8_t RNS_DEST_RNLOG_BEACON[16];
extern const uint8_t RNS_DEST_DRONE_CMD[16];

typedef struct {
    uint64_t hash;
    uint32_t timestamp_ms;
    bool     valid;
} rns_dedup_entry_t;

typedef struct {
    rns_dedup_entry_t ring[RNS_DEDUP_RING_SIZE];
    uint8_t           head;
    uint32_t          relayed_count;
    uint32_t          dropped_dup_count;
} rns_relay_table_t;

void rns_relay_init(rns_relay_table_t *t);
uint64_t rns_packet_invariant_hash(const uint8_t *pkt, size_t len);

/* Returns true if the packet should be relayed and prepares `inout_pkt`
 * (incrementing hops if unmasked RNS packet; preserving exact bytes if IFAC bit 0x80 is set).
 * Returns false if invalid (< 19 bytes), max hops reached, or already seen within RNS_DEDUP_WINDOW_MS.
 */
bool rns_relay_should_forward(rns_relay_table_t *t, uint8_t *inout_pkt, size_t len, uint32_t now_ms);

/* Record an locally originated packet hash in the dedup table so we do not re-relay our own echo */
void rns_relay_record_sent(rns_relay_table_t *t, const uint8_t *pkt, size_t len, uint32_t now_ms);

/* ---------------------------------------------------------------------------
 * 3. Flight Controller (MAVLink <-> Reticulum) Telemetry & Command Bridge
 * --------------------------------------------------------------------------- */
typedef struct {
    int32_t  lat_deg_e7;       /* Latitude * 1e7 */
    int32_t  lon_deg_e7;       /* Longitude * 1e7 */
    int32_t  alt_mm;           /* Altitude MSL in mm */
    int32_t  rel_alt_mm;       /* Relative altitude above home in mm */
    uint16_t vbatt_mv;         /* Battery voltage in mV */
    uint16_t curr_ca;          /* Battery current in 10 mA units */
    int8_t   batt_rem_pct;     /* Remaining battery % */
    uint16_t groundspeed_cms;  /* Groundspeed in cm/s */
    uint16_t heading_cdeg;     /* Heading in centi-degrees (0..35999) */
    uint8_t  custom_mode;      /* ArduCopter flight mode number */
    bool     armed;            /* Motors armed flag */
    bool     has_gps_fix;      /* Valid non-zero coordinates received */
    uint32_t rx_mav_Drop;
    uint32_t rx_mav_pkts;
} rns_fc_telemetry_t;

typedef enum {
    RNS_DRONE_CMD_NONE = 0,
    RNS_DRONE_CMD_RTL,
    RNS_DRONE_CMD_LOITER,
    RNS_DRONE_CMD_LAND,
    RNS_DRONE_CMD_GUIDED_GOTO,
    RNS_DRONE_CMD_RC_OVERRIDE
} rns_drone_cmd_type_t;

typedef struct {
    rns_drone_cmd_type_t type;
    int32_t              lat_deg_e7;
    int32_t              lon_deg_e7;
    int32_t              alt_m;
    uint16_t             rc_us[4];
} rns_drone_cmd_t;

/* Lightweight streaming MAVLink v1/v2 parser state */
typedef struct {
    uint8_t  state;
    bool     is_v2;
    uint8_t  payload_len;
    uint8_t  incompat_flags;
    uint8_t  pay_idx;
    uint32_t msg_id;
    uint8_t  payload[255];
} rns_mav_parser_t;

void rns_mav_parser_init(rns_mav_parser_t *p);
bool rns_mav_parse_byte(rns_mav_parser_t *p, uint8_t b, rns_fc_telemetry_t *inout_tlm);

/* Format a Reticulum PLAIN DATA packet to "rnlog.beacon" containing drone telemetry.
 * Returns total RNS packet length written to `out_pkt`.
 */
size_t rns_build_telemetry_beacon(const rns_fc_telemetry_t *tlm,
                                  uint8_t elrs_lq,
                                  int8_t elrs_rssi_dbm,
                                  uint8_t *out_pkt,
                                  size_t max_len);

/* Format a Reticulum PLAIN DATA packet to "elrs.drone.cmd" containing a command string. */
size_t rns_build_drone_command_pkt(const char *cmd_str, uint8_t *out_pkt, size_t max_len);

/* Check if an incoming RNS packet is addressed to "elrs.drone.cmd" and parse its command */
bool rns_parse_drone_command_pkt(const uint8_t *pkt, size_t len, rns_drone_cmd_t *out_cmd);

/* Build a MAVLink v2 frame (COMMAND_LONG or MISSION_ITEM_INT) to send to TMotorH743 over UART */
size_t rns_build_mavlink_command(const rns_drone_cmd_t *cmd, uint8_t seq, uint8_t *out_frame, size_t max_len);

/* ---------------------------------------------------------------------------
 * 4. ExpressLRS OTA DATA (ELRS_PKT_TYPE_DATA = 0x01) Fragmenter / Reassembler
 * --------------------------------------------------------------------------- */
#define ELRS_TUNNEL_MAX_MTU     508U
#define ELRS_TUNNEL_MAX_FRAGS   63U

typedef struct {
    uint8_t  tx_buf[ELRS_TUNNEL_MAX_MTU];
    uint16_t tx_len;
    uint8_t  tx_seq_id;
    uint8_t  tx_frag_idx;
    uint8_t  tx_total_frags;
    uint8_t  tx_repeat_rem;
    uint8_t  tx_repeats_per_frag;
    bool     tx_active;

    uint8_t  rx_buf[ELRS_TUNNEL_MAX_MTU];
    uint16_t rx_expected_len;
    uint8_t  rx_seq_id;
    uint8_t  last_completed_seq_id;
    uint8_t  rx_total_frags;
    uint64_t rx_frag_bitmap;
    bool     rx_in_progress;
    uint32_t tx_msgs_sent;
    uint32_t rx_msgs_completed;
} elrs_data_tunnel_t;

void elrs_tunnel_init(elrs_data_tunnel_t *t, uint8_t repeats_per_frag);
bool elrs_tunnel_enqueue_tx(elrs_data_tunnel_t *t, const uint8_t *data, uint16_t len, bool is_ota8);
bool elrs_tunnel_has_pending_tx(const elrs_data_tunnel_t *t);

/* Packs the next 8-byte (OTA4) or 13-byte (OTA8) ELRS_PKT_TYPE_DATA frame (including OTA CRC14/CRC16).
 * Returns true if a frame was produced into `out_ota_pkt`.
 */
bool elrs_tunnel_next_tx_frame(elrs_data_tunnel_t *t,
                               const elrs_ota_state_t *ota,
                               uint8_t *out_ota_pkt);

/* Processes an incoming CRC-validated ELRS_PKT_TYPE_DATA (type == 0x01) OTA4/OTA8 frame.
 * When all fragments of a message have been received, copies the reassembled message into
 * `out_msg` / `*out_msg_len` and returns true.
 */
bool elrs_tunnel_process_rx_frame(elrs_data_tunnel_t *t,
                                  const uint8_t *ota_pkt,
                                  bool is_ota8,
                                  uint8_t *out_msg,
                                  uint16_t *out_msg_len);

#ifdef __cplusplus
}
#endif
