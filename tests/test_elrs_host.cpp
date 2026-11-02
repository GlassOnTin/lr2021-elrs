/* SPDX-License-Identifier: AGPL-3.0-or-later */
/* Host unit test suite for ExpressLRS UID, CRC14/CRC16, OTA4/OTA8, FHSS, and CRSF */

#include "../firmware/elrs/elrs_protocol.h"
#include "../firmware/crsf/crsf.h"
#include "../firmware/rnode/rns_node.h"

#include <assert.h>
#include <dlfcn.h>
#include <stdio.h>
#include <string.h>
#include <set>
#include <string>
#include <vector>

static void test_uid_md5(void) {
    /* Verify MD5("-DMY_BINDING_PHRASE=\"default ExpressLRS binding phrase\"")
     * Expected MD5 first 6 bytes: 0a:b7:0c:7c:38:03 ([10, 183, 12, 124, 56, 3])
     */
    uint8_t uid[6] = {0};
    elrs_uid_from_bind_phrase("default ExpressLRS binding phrase", uid);
    const uint8_t expected[6] = {0x0A, 0xB7, 0x0C, 0x7C, 0x38, 0x03};
    assert(memcmp(uid, expected, 6) == 0);

    uint8_t parsed[6] = {0};
    assert(elrs_uid_parse_string("0a:b7:0c:7c:38:03", parsed));
    assert(memcmp(uid, parsed, 6) == 0);
    assert(elrs_uid_parse_string("10,183,12,124,56,3", parsed));
    assert(memcmp(uid, parsed, 6) == 0);
    assert(elrs_uid_parse_string("0AB70C7C3803", parsed));
    assert(memcmp(uid, parsed, 6) == 0);
    printf("[PASS] test_uid_md5 (UID=%02X:%02X:%02X:%02X:%02X:%02X)\n",
           uid[0], uid[1], uid[2], uid[3], uid[4], uid[5]);
}

static void test_fhss_invariants(void) {
    const elrs_domain_t domains[] = {
        ELRS_DOMAIN_AU915,
        ELRS_DOMAIN_FCC915,
        ELRS_DOMAIN_EU868,
        ELRS_DOMAIN_ISM2G4
    };

    for (size_t d = 0; d < sizeof(domains) / sizeof(domains[0]); d++) {
        elrs_fhss_state_t fhss;
        elrs_fhss_init(&fhss, domains[d], 0x01020304UL);
        uint32_t init_freq = elrs_fhss_get_initial_freq(&fhss);
        uint32_t n_ch      = fhss.info->freq_count;

        assert((fhss.sequence_count % n_ch) == 0U);
        std::set<uint32_t> seen;

        for (uint16_t i = 0; i < fhss.sequence_count; i++) {
            uint32_t f = elrs_fhss_get_curr_freq(&fhss);
            if ((i % n_ch) == 0U) {
                assert(f == init_freq);
                assert(elrs_fhss_on_sync_channel(&fhss));
                seen.clear();
                seen.insert(f);
            } else {
                assert(f != init_freq);
                assert(!elrs_fhss_on_sync_channel(&fhss));
                assert(seen.insert(f).second);
            }
            (void)elrs_fhss_advance_and_get_freq(&fhss);
        }
    }
    printf("[PASS] test_fhss_invariants (AU915, FCC915, EU868, ISM2G4)\n");
}

static void test_ota4_and_ota8_roundtrip(void) {
    uint8_t uid[6];
    elrs_uid_from_bind_phrase("lr2021-bench-test", uid);

    /* 1. OTA4 (8-byte packet, 14-bit CRC, Wide & Hybrid8 switches) */
    elrs_ota_state_t tx4, rx4;
    elrs_ota_init(&tx4, uid, ELRS_OTA4_PACKET_SIZE, ELRS_SW_WIDE_OR_8CH);
    elrs_ota_init(&rx4, uid, ELRS_OTA4_PACKET_SIZE, ELRS_SW_WIDE_OR_8CH);

    /* SYNC packet roundtrip */
    elrs_sync_data_t sync_in = {
        .fhss_index      = 42U,
        .nonce           = 17U,
        .rf_rate_enum    = ELRS_RATE_LORA_900_200HZ,
        .switch_enc_mode = ELRS_SW_WIDE_OR_8CH,
        .new_tlm_ratio   = 3U,
        .gemini_mode     = 1U,
        .ota_protocol    = 0U,
        .uid4            = uid[4],
        .uid5            = uid[5],
    };
    uint8_t pkt4[ELRS_OTA4_PACKET_SIZE];
    elrs_ota_pack_sync(&tx4, &sync_in, pkt4);

    elrs_sync_data_t sync_out = {};
    assert(elrs_ota_unpack_sync(&rx4, pkt4, &sync_out));
    assert(sync_out.fhss_index == 42U);
    assert(sync_out.nonce == 17U);
    assert(sync_out.rf_rate_enum == ELRS_RATE_LORA_900_200HZ);
    assert(sync_out.gemini_mode == 1U);
    assert(sync_out.uid4 == uid[4] && sync_out.uid5 == uid[5]);

    /* Flip 1 bit and verify CRC rejects it */
    pkt4[3] ^= 0x04U;
    assert(!elrs_ota_unpack_sync(&rx4, pkt4, &sync_out));

    /* RC Channels roundtrip over 16 nonces (OTA4 Wide) */
    uint16_t ch_in[16] = {
        172, 580, 992, 1400, 1792, 191, 992, 1792,
        400, 800, 1200, 1600, 992, 1792, 992, 992
    };
    uint16_t ch_out[16] = {0};
    for (uint8_t n = 0; n < 16U; n++) {
        tx4.ota_nonce = n;
        rx4.ota_nonce = n;
        elrs_ota_pack_rc(&tx4, ch_in, true, (n & 1U) != 0U, pkt4);
        bool armed = false, ack = false;
        assert(elrs_ota_unpack_rc(&rx4, pkt4, ch_out, &armed, &ack));
        assert(armed == true);
        assert(ack == ((n & 1U) != 0U));
    }
    for (int i = 0; i < 4; i++) {
        int diff = (int)ch_in[i] - (int)ch_out[i];
        if (diff < 0) diff = -diff;
        assert(diff <= 2);
    }

    /* Downlink LinkStats roundtrip (OTA4) */
    elrs_link_stats_data_t ls_in = {
        .uplink_rssi_1  = 65U,
        .uplink_rssi_2  = 68U,
        .antenna        = 1U,
        .model_match    = 1U,
        .lq             = 100U,
        .true_diversity = 1U,
        .snr_scaled     = 32,
    };
    tx4.ota_nonce = 99U;
    rx4.ota_nonce = 99U;
    elrs_ota_pack_linkstats(&rx4, &ls_in, 5U, true, pkt4);
    elrs_link_stats_data_t ls_out = {};
    uint8_t pkg_idx = 0U;
    bool ack_out = false;
    assert(elrs_ota_unpack_linkstats(&tx4, pkt4, &ls_out, &pkg_idx, &ack_out));
    assert(ls_out.uplink_rssi_1 == 65U && ls_out.uplink_rssi_2 == 68U);
    assert(ls_out.lq == 100U && ls_out.snr_scaled == 32);
    assert(pkg_idx == 5U && ack_out == true);

    /* 2. OTA8 (13-byte packet, 16-bit CRC, 16ch FullRes) */
    elrs_ota_state_t tx8, rx8;
    elrs_ota_init(&tx8, uid, ELRS_OTA8_PACKET_SIZE, ELRS_SW_HYBRID_OR_16CH);
    elrs_ota_init(&rx8, uid, ELRS_OTA8_PACKET_SIZE, ELRS_SW_HYBRID_OR_16CH);

    uint8_t pkt8[ELRS_OTA8_PACKET_SIZE];
    memset(ch_out, 0, sizeof(ch_out));
    for (uint8_t n = 0; n < 4U; n++) {
        tx8.ota_nonce = n;
        rx8.ota_nonce = n;
        elrs_ota_pack_rc(&tx8, ch_in, false, false, pkt8);
        bool armed = true, ack = true;
        assert(elrs_ota_unpack_rc(&rx8, pkt8, ch_out, &armed, &ack));
        assert(armed == false && ack == false);
    }
    for (int i = 0; i < 16; i++) {
        int diff = (int)ch_in[i] - (int)ch_out[i];
        if (diff < 0) diff = -diff;
        assert(diff <= 2);
    }

    printf("[PASS] test_ota4_and_ota8_roundtrip (SYNC, RC4, RC8, LinkStats, CRC14/CRC16)\n");
}

static void test_crsf_framing(void) {
    uint16_t ch_in[16];
    for (int i = 0; i < 16; i++) {
        ch_in[i] = (uint16_t)(172 + i * 100);
    }
    uint8_t frame[26];
    assert(crsf_build_rc_channels_frame(ch_in, frame) == 26U);

    crsf_parser_t parser;
    crsf_parser_init(&parser);
    bool got_frame = false;
    for (size_t i = 0; i < 26U; i++) {
        if (crsf_parser_feed_byte(&parser, frame[i])) {
            assert(i == 25U);
            got_frame = true;
        }
    }
    assert(got_frame);

    uint16_t ch_out[16] = {0};
    assert(crsf_decode_rc_channels_frame(parser.buf, 26U, ch_out));
    assert(memcmp(ch_in, ch_out, sizeof(ch_in)) == 0);

    crsf_link_stats_t ls_in = {
        .uplink_rssi_1   = 52,
        .uplink_rssi_2   = 55,
        .uplink_lq       = 100,
        .uplink_snr      = 9,
        .active_antenna  = 0,
        .rf_mode         = 2,
        .uplink_tx_power = 3,
        .downlink_rssi   = 60,
        .downlink_lq     = 99,
        .downlink_snr    = 8,
    };
    uint8_t ls_frame[14];
    assert(crsf_build_link_stats_frame(&ls_in, ls_frame) == 14U);
    crsf_link_stats_t ls_out = {};
    assert(crsf_decode_link_stats_frame(ls_frame, 14U, &ls_out));
    assert(memcmp(&ls_in, &ls_out, sizeof(ls_in)) == 0);

    assert(crsf_us_to_ticks(1500) == 992);
    assert(crsf_ticks_to_us(992) == 1500);
    printf("[PASS] test_crsf_framing (RC_CHANNELS 0x16, LINK_STATISTICS 0x14, stream parser)\n");
}

static void test_demo_live_packets(void) {
    /* Verify OTA packets with binding phrase "lr2021-elrs-demo" -> UID = 6C:40:CF:45:F5:08
     * Air rate: 2G4_K1000 (ELRS_RATE_FSK_2G4_1000HZ = 0x24 = 36), CE_LBT domain
     */
    uint8_t uid[6] = {0};
    elrs_uid_from_bind_phrase("lr2021-elrs-demo", uid);
    const uint8_t expected_uid[6] = {0x6C, 0x40, 0xCF, 0x45, 0xF5, 0x08};
    assert(memcmp(uid, expected_uid, 6) == 0);

    elrs_ota_state_t ota;
    elrs_ota_init(&ota, uid, ELRS_OTA4_PACKET_SIZE, ELRS_SW_HYBRID_OR_16CH);
    assert(ota.uid_seed == 0xCF45F50CUL);
    assert(ota.crc_init == 0xF108U);

    /* 1. SYNC packet at hop=160, nonce=193 */
    elrs_sync_data_t sync_in = {};
    sync_in.fhss_index      = 160U;
    sync_in.nonce           = 193U;
    sync_in.rf_rate_enum    = ELRS_RATE_FSK_2G4_1000HZ;
    sync_in.switch_enc_mode = ELRS_SW_HYBRID_OR_16CH;
    sync_in.new_tlm_ratio   = 7U; /* 1:2 telemetry ratio */
    sync_in.uid4            = uid[4];
    sync_in.uid5            = uid[5];
    uint8_t sync_pkt_160[8] = {0};
    elrs_ota_pack_sync(&ota, &sync_in, sync_pkt_160);

    elrs_sync_data_t sync = {};
    assert(elrs_ota_unpack_sync(&ota, sync_pkt_160, &sync));
    assert(sync.fhss_index == 160U);
    assert(sync.nonce == 193U);
    assert(sync.rf_rate_enum == ELRS_RATE_FSK_2G4_1000HZ);
    assert(sync.switch_enc_mode == ELRS_SW_HYBRID_OR_16CH);
    assert(sync.new_tlm_ratio == 7U);
    assert(sync.uid4 == 0xF5U && sync.uid5 == 0x08U);

    /* 2. SYNC packet at hop=0, nonce=193 */
    sync_in.fhss_index = 0U;
    uint8_t sync_pkt_0[8] = {0};
    elrs_ota_pack_sync(&ota, &sync_in, sync_pkt_0);
    assert(elrs_ota_unpack_sync(&ota, sync_pkt_0, &sync));
    assert(sync.fhss_index == 0U);
    assert(sync.nonce == 193U);

    /* 3. LINKSTATS telemetry packet at hop=1, nonce=194 */
    ota.ota_nonce = 194U;
    elrs_link_stats_data_t ls_in = {};
    ls_in.uplink_rssi_1  = 74U;
    ls_in.uplink_rssi_2  = 57U;
    ls_in.lq             = 96U;
    ls_in.true_diversity = 1U;
    uint8_t tlm_pkt[8] = {0};
    elrs_ota_pack_linkstats(&ota, &ls_in, 0U, false, tlm_pkt);

    elrs_link_stats_data_t ls = {};
    uint8_t pkg_idx = 0U;
    bool ack = false;
    assert(elrs_ota_unpack_linkstats(&ota, tlm_pkt, &ls, &pkg_idx, &ack));
    assert(ls.uplink_rssi_1 == 74U);
    assert(ls.uplink_rssi_2 == 57U);
    assert(ls.lq == 96U);
    assert(ls.true_diversity == 1U);

    /* 4. RCDATA packet at hop=1, nonce=195 */
    ota.ota_nonce = 195U;
    uint16_t ch_in[16] = {0};
    ch_in[0] = crsf_us_to_ticks(1500U);
    ch_in[1] = crsf_us_to_ticks(1500U);
    ch_in[2] = crsf_us_to_ticks(989U);
    ch_in[3] = crsf_us_to_ticks(1498U);
    uint8_t rc_pkt[8] = {0};
    elrs_ota_pack_rc(&ota, ch_in, false, false, rc_pkt);

    uint16_t ch[16] = {0};
    bool armed = true;
    assert(elrs_ota_unpack_rc(&ota, rc_pkt, ch, &armed, &ack));
    assert(!armed);
    assert(crsf_ticks_to_us(ch[0]) == 1500U);
    assert(crsf_ticks_to_us(ch[1]) == 1500U);
    assert(abs((int)crsf_ticks_to_us(ch[2]) - 989) <= 2);
    assert(abs((int)crsf_ticks_to_us(ch[3]) - 1498) <= 2);

    printf("[PASS] test_demo_live_packets (TX16SMK3 SYNC + RCDATA, XR4 LINKSTATS)\n");
}

struct KissTestCtx {
    uint8_t last_cmd;
    uint8_t last_buf[RNODE_KISS_BUF_SIZE];
    size_t  last_len;
    int     frame_count;
};

static void on_test_kiss_frame(uint8_t cmd, const uint8_t *data, size_t len, void *user_ctx) {
    KissTestCtx *ctx = (KissTestCtx *)user_ctx;
    ctx->last_cmd = cmd;
    ctx->last_len = len;
    if (len > 0U && len <= sizeof(ctx->last_buf)) {
        memcpy(ctx->last_buf, data, len);
    }
    ctx->frame_count++;
}

static void test_rnode_kiss_framing(void) {
    KissTestCtx ctx = {};
    rnode_kiss_parser_t parser;
    rnode_kiss_init(&parser, on_test_kiss_frame, &ctx);

    /* Payload containing FEND (0xC0) and FESC (0xDB) to verify byte-stuffing */
    const uint8_t raw_payload[] = {0x08, 0x00, 0xC0, 0xDB, 0xDC, 0xDD, 0x42, 0xFF};
    uint8_t frame[64];
    size_t flen = rnode_kiss_build_frame(frame, sizeof(frame), RNODE_CMD_DATA, raw_payload, sizeof(raw_payload));
    assert(flen == sizeof(raw_payload) + 3U + 2U); /* +3 FEND/CMD/FEND, +2 escaped bytes */

    for (size_t i = 0; i < flen; i++) {
        rnode_kiss_feed_byte(&parser, frame[i]);
    }
    assert(ctx.frame_count == 1);
    assert(ctx.last_cmd == RNODE_CMD_DATA);
    assert(ctx.last_len == sizeof(raw_payload));
    assert(memcmp(ctx.last_buf, raw_payload, sizeof(raw_payload)) == 0);

    /* Verify PHYPRM calculation for 868 MHz, BW=125 kHz, SF7, preamble=16 */
    uint16_t slot_ms = 0, difs_ms = 0;
    uint8_t phy_buf[12] = {0};
    assert(rnode_build_phyprm(125000UL, 7U, 16U, &slot_ms, &difs_ms, phy_buf) == 12U);
    assert(slot_ms == 2U); /* sym_us = 1024 us -> ceil = 2 ms */
    assert(difs_ms == 6U);
    printf("[PASS] test_rnode_kiss_framing (FEND/FESC escaping, CMD_DATA, CMD_STAT_PHYPRM)\n");
}

static void test_rns_relay_dedup_and_ifac(void) {
    rns_relay_table_t relay;
    rns_relay_init(&relay);

    /* 1. Unmasked Reticulum packet (HEADER_1, PLAIN, DATA, hops=0) */
    uint8_t pkt[32];
    size_t len = rns_build_drone_command_pkt("RTL", pkt, sizeof(pkt));
    assert(len == 22U);
    assert(pkt[0] == 0x08U && pkt[1] == 0x00U);

    /* First reception at t=1000ms: should forward and increment hops to 1 */
    assert(rns_relay_should_forward(&relay, pkt, len, 1000UL));
    assert(pkt[1] == 0x01U);
    assert(relay.relayed_count == 1U);

    /* Immediate echo of the relayed packet (with hops=1) at t=1200ms: must be dropped! */
    assert(!rns_relay_should_forward(&relay, pkt, len, 1200UL));
    assert(relay.dropped_dup_count == 1U);

    /* After dedup window expires (t=12000ms > 1000 + 10000ms): allowed again */
    assert(rns_relay_should_forward(&relay, pkt, len, 12000UL));
    assert(pkt[1] == 0x02U);

    /* 2. IFAC-protected Reticulum packet (bit 7 = 0x80 set): must NOT mutate byte 1 (hops) */
    uint8_t ifac_pkt[28] = {0x88, 0x3A, 0x11, 0x22, 0x33, 0x44};
    uint8_t orig_ifac[28];
    memcpy(orig_ifac, ifac_pkt, sizeof(ifac_pkt));
    assert(rns_relay_should_forward(&relay, ifac_pkt, sizeof(ifac_pkt), 2000UL));
    assert(memcmp(ifac_pkt, orig_ifac, sizeof(ifac_pkt)) == 0);
    assert(!rns_relay_should_forward(&relay, ifac_pkt, sizeof(ifac_pkt), 2500UL));
    printf("[PASS] test_rns_relay_dedup_and_ifac (invariant hash ring, hop increment, IFAC preservation)\n");
}

static void test_fc_mavlink_and_beacons(void) {
    rns_fc_telemetry_t tlm = {};
    tlm.lat_deg_e7  = -353632620L; /* -35.36326 deg N */
    tlm.lon_deg_e7  = 1491652370L;  /* 0.00000 deg W */
    tlm.rel_alt_mm  = 85000L;     /* 85 m AGL */
    tlm.vbatt_mv    = 16800U;     /* 4S 16.8V */
    tlm.custom_mode = 5U;         /* Loiter */
    tlm.armed       = true;
    tlm.has_gps_fix = true;

    uint8_t bcn[255];
    size_t blen = rns_build_telemetry_beacon(&tlm, 99U, -52, bcn, sizeof(bcn));
    assert(blen > 19U);
    assert(bcn[0] == 0x08U && bcn[1] == 0x00U);
    assert(memcmp(&bcn[2], RNS_DEST_RNLOG_BEACON, 16U) == 0);
    assert(strstr((const char *)&bcn[19], "\"lat\":-35.36326") != NULL);
    assert(strstr((const char *)&bcn[19], "\"lon\":149.16523") != NULL);
    assert(strstr((const char *)&bcn[19], "\"vbatt_mv\":16800") != NULL);

    /* Verify Reticulum drone command packet round-trip & MAVLink v2 frame generation */
    uint8_t cmd_pkt[64];
    size_t clen = rns_build_drone_command_pkt("GOTO -353632620 1491652370 60", cmd_pkt, sizeof(cmd_pkt));
    assert(clen > 19U);

    rns_drone_cmd_t parsed = {};
    assert(rns_parse_drone_command_pkt(cmd_pkt, clen, &parsed));
    assert(parsed.type == RNS_DRONE_CMD_GUIDED_GOTO);
    assert(parsed.lat_deg_e7 == -353632620L);
    assert(parsed.lon_deg_e7 == 1491652370L);
    assert(parsed.alt_m == 60L);

    uint8_t mav_frame[64];
    size_t mlen = rns_build_mavlink_command(&parsed, 7U, mav_frame, sizeof(mav_frame));
    assert(mlen == 49U); /* 10B header + 37B MISSION_ITEM_INT + 2B CRC */
    assert(mav_frame[0] == 0xFDU && mav_frame[1] == 37U && mav_frame[7] == 73U);

    /* Verify MODE 5 -> COMMAND_LONG (76) MAV_CMD_DO_SET_MODE (176) */
    clen = rns_build_drone_command_pkt("MODE 5", cmd_pkt, sizeof(cmd_pkt));
    assert(rns_parse_drone_command_pkt(cmd_pkt, clen, &parsed));
    assert(parsed.type == RNS_DRONE_CMD_SET_MODE && parsed.mode_num == 5U);
    mlen = rns_build_mavlink_command(&parsed, 8U, mav_frame, sizeof(mav_frame));
    assert(mlen == 45U && mav_frame[0] == 0xFDU && mav_frame[1] == 33U && mav_frame[7] == 76U);

    /* Verify ARM / DISARM -> COMMAND_LONG (76) MAV_CMD_COMPONENT_ARM_DISARM (400) */
    clen = rns_build_drone_command_pkt("ARM", cmd_pkt, sizeof(cmd_pkt));
    assert(rns_parse_drone_command_pkt(cmd_pkt, clen, &parsed));
    assert(parsed.type == RNS_DRONE_CMD_ARM);
    mlen = rns_build_mavlink_command(&parsed, 9U, mav_frame, sizeof(mav_frame));
    assert(mlen == 45U && mav_frame[7] == 76U);

    clen = rns_build_drone_command_pkt("DISARM", cmd_pkt, sizeof(cmd_pkt));
    assert(rns_parse_drone_command_pkt(cmd_pkt, clen, &parsed));
    assert(parsed.type == RNS_DRONE_CMD_DISARM);

    /* Verify RC 1600 1400 1100 1500 -> RC_CHANNELS_OVERRIDE (70, 30B frame) */
    clen = rns_build_drone_command_pkt("RC 1600 1400 1100 1500", cmd_pkt, sizeof(cmd_pkt));
    assert(rns_parse_drone_command_pkt(cmd_pkt, clen, &parsed));
    assert(parsed.type == RNS_DRONE_CMD_RC_OVERRIDE);
    assert(parsed.rc_us[0] == 1600U && parsed.rc_us[1] == 1400U && parsed.rc_us[2] == 1100U && parsed.rc_us[3] == 1500U);
    mlen = rns_build_mavlink_command(&parsed, 10U, mav_frame, sizeof(mav_frame));
    assert(mlen == 30U && mav_frame[0] == 0xFDU && mav_frame[1] == 18U && mav_frame[7] == 70U);
    printf("[PASS] test_fc_mavlink_and_beacons (rnlog.beacon, elrs.drone.cmd, MAVLink v2 MISSION_ITEM_INT/COMMAND_LONG/RC_OVERRIDE)\n");
}

static void test_elrs_data_tunnel(void) {
    uint8_t uid[6] = {0};
    elrs_uid_from_bind_phrase("lr2021-elrs-demo", uid);
    elrs_ota_state_t ota4, ota8;
    elrs_ota_init(&ota4, uid, ELRS_OTA4_PACKET_SIZE, ELRS_SW_HYBRID_OR_16CH);
    elrs_ota_init(&ota8, uid, ELRS_OTA8_PACKET_SIZE, ELRS_SW_HYBRID_OR_16CH);

    /* Build a 110-byte Reticulum telemetry beacon packet */
    rns_fc_telemetry_t tlm = {};
    tlm.lat_deg_e7 = -353632620L;
    tlm.lon_deg_e7 = 1491652370L;
    tlm.rel_alt_mm = 120000L;
    tlm.vbatt_mv   = 24800U;
    tlm.armed      = true;
    uint8_t rns_pkt[255];
    uint16_t rns_len = (uint16_t)rns_build_telemetry_beacon(&tlm, 100U, -48, rns_pkt, sizeof(rns_pkt));
    assert(rns_len > 90U);

    /* 1. Tunnel over OTA8 (13-byte 2G4_K1000 frames) with repeats=2 and simulated 30% packet loss */
    elrs_data_tunnel_t tx8, rx8;
    elrs_tunnel_init(&tx8, 2U);
    elrs_tunnel_init(&rx8, 2U);
    assert(elrs_tunnel_enqueue_tx(&tx8, rns_pkt, rns_len, true));

    uint8_t out_msg[ELRS_TUNNEL_MAX_MTU] = {0};
    uint16_t out_len = 0U;
    bool completed = false;
    int frame_idx = 0;
    while (elrs_tunnel_has_pending_tx(&tx8)) {
        uint8_t ota_pkt[13];
        assert(elrs_tunnel_next_tx_frame(&tx8, &ota8, ota_pkt));
        /* Verify OTA8 CRC16 is valid */
        assert(elrs_ota_validate_crc(&ota8, ota_pkt, ota8.ota_nonce));

        /* Drop every odd frame (50% loss — first copy of each fragment passes, second drops) */
        if ((frame_idx % 2) == 0) {
            if (elrs_tunnel_process_rx_frame(&rx8, ota_pkt, true, out_msg, &out_len)) {
                completed = true;
            }
        }
        frame_idx++;
    }
    assert(completed);
    assert(out_len == rns_len);
    assert(memcmp(out_msg, rns_pkt, rns_len) == 0);

    /* 2. Tunnel over OTA4 (8-byte 900_200HZ / 900_K1000 frames) */
    elrs_data_tunnel_t tx4, rx4;
    elrs_tunnel_init(&tx4, 2U);
    elrs_tunnel_init(&rx4, 2U);
    assert(elrs_tunnel_enqueue_tx(&tx4, rns_pkt, rns_len, false));

    memset(out_msg, 0, sizeof(out_msg));
    out_len = 0U;
    completed = false;
    while (elrs_tunnel_has_pending_tx(&tx4)) {
        uint8_t ota_pkt[8];
        assert(elrs_tunnel_next_tx_frame(&tx4, &ota4, ota_pkt));
        /* Verify OTA4 CRC14 is valid */
        assert(elrs_ota_validate_crc(&ota4, ota_pkt, ota4.ota_nonce));

        if (elrs_tunnel_process_rx_frame(&rx4, ota_pkt, false, out_msg, &out_len)) {
            completed = true;
        }
    }
    assert(completed);
    assert(out_len == rns_len);
    assert(memcmp(out_msg, rns_pkt, rns_len) == 0);
    printf("[PASS] test_elrs_data_tunnel (OTA8 13B + OTA4 8B fragmentation/reassembly with loss recovery)\n");
}

static void test_rns_identity_pairing_and_lua_crsf(void) {
    /* 1. Verify FIPS 180-4 SHA-256 test vector ("abc") */
    const uint8_t abc[3] = {'a', 'b', 'c'};
    const uint8_t expected_abc_sha[32] = {
        0xba, 0x78, 0x16, 0xbf, 0x8f, 0x01, 0xcf, 0xea,
        0x41, 0x41, 0x40, 0xde, 0x5d, 0xae, 0x22, 0x23,
        0xb0, 0x03, 0x61, 0xa3, 0x96, 0x17, 0x7a, 0x9c,
        0xb4, 0x10, 0xff, 0x61, 0xf2, 0x00, 0x15, 0xad
    };
    uint8_t sha[32] = {0};
    rns_sha256(abc, 3U, sha);
    assert(memcmp(sha, expected_abc_sha, 32U) == 0);

    /* 2. Verify two hardware boards with identical ELRS binding UID derive distinct 16B Reticulum SINGLE addresses */
    uint8_t uid[6];
    elrs_uid_from_bind_phrase("lr2021-elrs-demo", uid);
    const uint8_t hw_board1[8] = {0xFC, 0x6D, 0x1A, 0x37, 0x11, 0x22, 0x33, 0x44};
    const uint8_t hw_board2[8] = {0xD5, 0x6B, 0x69, 0x41, 0x55, 0x66, 0x77, 0x88};

    rns_node_identity_t tx_id, rx_id;
    rns_identity_init(&tx_id, hw_board1, uid, RNS_NODE_ROLE_TX, "LR2021-TX");
    rns_identity_init(&rx_id, hw_board2, uid, RNS_NODE_ROLE_RX, "LR2021-RX");
    assert(memcmp(tx_id.dest_hash, rx_id.dest_hash, 16U) != 0);
    assert(memcmp(tx_id.identity_hash, rx_id.identity_hash, 16U) != 0);

    /* Verify determinism: re-running with same hw_id + uid produces identical dest_hash */
    rns_node_identity_t rx_id_again;
    rns_identity_init(&rx_id_again, hw_board2, uid, RNS_NODE_ROLE_RX, "LR2021-RX");
    assert(memcmp(rx_id.dest_hash, rx_id_again.dest_hash, 16U) == 0);

    /* 3. Verify RNSA Node Announce & Peer Table auto-discovery */
    rns_peer_table_t tx_peers;
    rns_peer_table_init(&tx_peers);
    assert(tx_peers.count == 0U && tx_peers.selected_idx == -1);

    rns_fc_telemetry_t rx_tlm = {};
    rx_tlm.vbatt_mv    = 25100U;
    rx_tlm.custom_mode = 5U; /* Loiter */
    rx_tlm.armed       = true;

    uint8_t ann_pkt[128];
    size_t ann_len = rns_build_node_announce(&rx_id, 868000000UL, 125000UL, 7U, 5U, &rx_tlm, ann_pkt, sizeof(ann_pkt));
    assert(ann_len == 68U);
    assert(ann_pkt[0] == 0x01U); /* HEADER_1, SINGLE, ANNOUNCE */

    assert(rns_parse_node_announce(ann_pkt, ann_len, -42, 11, 5000UL, &tx_peers));
    assert(tx_peers.count == 1U);
    assert(tx_peers.selected_idx == 0); /* Automatically selected first RX peer */
    assert(memcmp(tx_peers.peers[0].dest_hash, rx_id.dest_hash, 16U) == 0);
    assert(tx_peers.peers[0].role == RNS_NODE_ROLE_RX);
    assert(tx_peers.peers[0].vbatt_mv == 25100U);
    assert(tx_peers.peers[0].custom_mode == 5U);
    assert(strcmp(tx_peers.peers[0].name, "LR2021-RX") == 0);

    /* Tampered announce authenticator must be rejected */
    ann_pkt[ann_len - 1U] ^= 0xFFU;
    assert(!rns_parse_node_announce(ann_pkt, ann_len, -42, 11, 5100UL, &tx_peers));

    /* 4. Verify Over-the-Air Reticulum Pairing & Config Sync (RNSP) */
    rns_pairing_cfg_t p_in = {};
    memcpy(p_in.tx_dest, tx_id.dest_hash, 16U);
    p_in.freq_hz      = 869525000UL;
    p_in.bw_hz        = 250000UL;
    p_in.sf           = 8U;
    p_in.cr           = 6U;
    p_in.txpower_dbm  = 14;
    p_in.ifac_enabled = true;
    strcpy(p_in.ifac_pass, "mesh-key-42");

    uint8_t pair_pkt[128];
    size_t pair_len = rns_build_pairing_pkt(&p_in, pair_pkt, sizeof(pair_pkt));
    assert(pair_len == 68U);

    rns_pairing_cfg_t p_out = {};
    assert(rns_parse_pairing_pkt(pair_pkt, pair_len, rx_id.dest_hash, &p_out));
    assert(memcmp(p_out.tx_dest, tx_id.dest_hash, 16U) == 0);
    assert(p_out.freq_hz == 869525000UL);
    assert(p_out.bw_hz == 250000UL);
    assert(p_out.sf == 8U && p_out.cr == 6U && p_out.txpower_dbm == 14);
    assert(p_out.ifac_enabled && strcmp(p_out.ifac_pass, "mesh-key-42") == 0);

    /* Tampered pairing authenticator must be rejected */
    pair_pkt[pair_len - 1U] ^= 0xFFU;
    assert(!rns_parse_pairing_pkt(pair_pkt, pair_len, rx_id.dest_hash, &p_out));

    /* 5. Verify Targeted Receiver Command (RNSC) addressed to RX's 16-byte dest_hash */
    uint8_t tcmd_pkt[128];
    size_t tcmd_len = rns_build_targeted_cmd_pkt(rx_id.dest_hash, tx_id.dest_hash, 42U, "LOITER", tcmd_pkt, sizeof(tcmd_pkt));
    assert(tcmd_len > 40U);

    uint8_t sender_dest[16] = {0};
    uint8_t rx_seq = 0U;
    char rx_cmd[32] = {0};
    /* Another node (e.g., TX itself) must NOT accept a command addressed to rx_id.dest_hash */
    assert(!rns_parse_targeted_cmd_pkt(tcmd_pkt, tcmd_len, tx_id.dest_hash, sender_dest, &rx_seq, rx_cmd, sizeof(rx_cmd)));
    /* The targeted RX node MUST accept it and recover sender_dest == tx_id.dest_hash */
    assert(rns_parse_targeted_cmd_pkt(tcmd_pkt, tcmd_len, rx_id.dest_hash, sender_dest, &rx_seq, rx_cmd, sizeof(rx_cmd)));
    assert(memcmp(sender_dest, tx_id.dest_hash, 16U) == 0);
    assert(rx_seq == 42U);
    assert(strcmp(rx_cmd, "LOITER") == 0);

    /* 6. Verify CRSF 0x7A/0x7B EdgeTX Lua telemetry frames fit within 64B and round-trip */
    crsf_rns_status_t st_in = {};
    st_in.app_mode         = 2U;
    st_in.lock_state       = 2U;
    st_in.elrs_rate_idx    = 3U;
    st_in.rnode_txpower    = 10;
    st_in.rnode_freq_hz    = 868000000UL;
    st_in.rnode_bw_khz     = 125U;
    st_in.rnode_sf         = 7U;
    st_in.rnode_cr         = 5U;
    st_in.ifac_enabled     = true;
    st_in.peer_count       = 1U;
    st_in.selected_peer    = 0U;
    st_in.last_ping_rtt_ms = 28U;
    st_in.tx_msgs          = 12U;
    st_in.rx_msgs          = 11U;
    st_in.relayed_msgs     = 4U;
    memcpy(st_in.tx_dest, tx_id.dest_hash, 16U);
    memcpy(st_in.rx_dest, rx_id.dest_hash, 16U);

    uint8_t crsf_buf[64] = {0};
    size_t st_flen = crsf_build_rns_status_frame(&st_in, crsf_buf);
    assert(st_flen == 62U && st_flen <= CRSF_MAX_FRAME_LEN);

    crsf_rns_status_t st_out = {};
    assert(crsf_decode_rns_status_frame(crsf_buf, st_flen, &st_out));
    assert(st_out.rnode_freq_hz == 868000000UL && st_out.last_ping_rtt_ms == 28U);
    assert(memcmp(st_out.tx_dest, tx_id.dest_hash, 16U) == 0);
    assert(memcmp(st_out.rx_dest, rx_id.dest_hash, 16U) == 0);

    crsf_rns_peer_info_t pi_in = {};
    pi_in.peer_idx    = 0U;
    pi_in.total_peers = 1U;
    pi_in.is_selected = true;
    pi_in.role        = RNS_NODE_ROLE_RX;
    pi_in.hops        = 1U;
    pi_in.rssi_dbm    = -45;
    pi_in.snr_db      = 10;
    pi_in.armed       = true;
    pi_in.custom_mode = 5U;
    pi_in.vbatt_mv    = 25100U;
    pi_in.age_s       = 2U;
    memcpy(pi_in.dest_hash, rx_id.dest_hash, 16U);
    strcpy(pi_in.name, "LR2021-RX");

    size_t pi_flen = crsf_build_rns_peer_frame(&pi_in, crsf_buf);
    assert(pi_flen == 48U && pi_flen <= CRSF_MAX_FRAME_LEN);

    crsf_rns_peer_info_t pi_out = {};
    assert(crsf_decode_rns_peer_frame(crsf_buf, pi_flen, &pi_out));
    assert(pi_out.vbatt_mv == 25100U && pi_out.hops == 1U && pi_out.rssi_dbm == -45);
    assert(memcmp(pi_out.dest_hash, rx_id.dest_hash, 16U) == 0);
    assert(strcmp(pi_out.name, "LR2021-RX") == 0);

    /* 7. Verify CRSF 0x21 FLIGHT_MODE telemetry string round-trip & 0xEC receiver address parsing */
    uint8_t fm_buf[64] = {0};
    size_t fm_len = crsf_build_flight_mode_frame("RNS:4cef..c149 1p", fm_buf);
    assert(fm_len == 22U);
    char fm_dec[32] = {0};
    assert(crsf_decode_flight_mode_frame(fm_buf, fm_len, fm_dec, sizeof(fm_dec)));
    assert(strcmp(fm_dec, "RNS:4cef..c149 1p") == 0);

    uint8_t cmd_buf[32] = {0};
    size_t cmd_len = crsf_build_rns_cmd_frame(CRSF_RNS_SUBCMD_GET_STATUS, NULL, 0U, cmd_buf);
    cmd_buf[0] = CRSF_ADDRESS_CRSF_RECEIVER;
    cmd_buf[3] = CRSF_ADDRESS_CRSF_RECEIVER;
    cmd_buf[cmd_len - 1U] = crsf_crc8(&cmd_buf[2], cmd_len - 3U);
    crsf_parser_t rx_p;
    crsf_parser_init(&rx_p);
    bool got_rx_cmd = false;
    for (size_t i = 0U; i < cmd_len; i++) {
        if (crsf_parser_feed_byte(&rx_p, cmd_buf[i])) {
            got_rx_cmd = true;
        }
    }
    assert(got_rx_cmd && rx_p.buf[0] == CRSF_ADDRESS_CRSF_RECEIVER && rx_p.buf[2] == CRSF_FRAMETYPE_RNS_CMD);

    /* 8. Verify CRSF 0x7A SET_CFG & 0x7B RESP_ACK round-trip + CRC tamper rejection */
    uint8_t cfg_pay[12] = {
        (uint8_t)((2440000000UL >> 24) & 0xFFU),
        (uint8_t)((2440000000UL >> 16) & 0xFFU),
        (uint8_t)((2440000000UL >> 8)  & 0xFFU),
        (uint8_t)(2440000000UL & 0xFFU),
        0x00, 0x07, 0xA1, 0x20, /* 500000 Hz */
        8U,                     /* SF8 */
        6U,                     /* CR 4/6 */
        (uint8_t)((int8_t)-5),  /* -5 dBm */
        1U                      /* IFAC ON */
    };
    size_t cfg_flen = crsf_build_rns_cmd_frame(CRSF_RNS_SUBCMD_SET_CFG, cfg_pay, sizeof(cfg_pay), cmd_buf);
    assert(cfg_flen == 19U);
    crsf_rns_cfg_cmd_t cfg_dec = {};
    assert(crsf_decode_rns_set_cfg_frame(cmd_buf, cfg_flen, &cfg_dec));
    assert(cfg_dec.freq_hz == 2440000000UL && cfg_dec.bw_hz == 500000UL);
    assert(cfg_dec.sf == 8U && cfg_dec.cr == 6U && cfg_dec.txpower_dbm == -5 && cfg_dec.ifac_enabled);
    cmd_buf[cfg_flen - 1U] ^= 0xFFU;
    assert(!crsf_decode_rns_set_cfg_frame(cmd_buf, cfg_flen, &cfg_dec));

    uint8_t ack_buf[16] = {0};
    size_t ack_len = crsf_build_rns_ack_frame(CRSF_RNS_SUBCMD_SET_CFG, 0U, ack_buf);
    assert(ack_len == 9U);
    uint8_t ack_sub = 0U, ack_st = 99U;
    assert(crsf_decode_rns_ack_frame(ack_buf, ack_len, &ack_sub, &ack_st));
    assert(ack_sub == CRSF_RNS_SUBCMD_SET_CFG && ack_st == 0U);
    ack_buf[ack_len - 1U] ^= 0xFFU;
    assert(!crsf_decode_rns_ack_frame(ack_buf, ack_len, &ack_sub, &ack_st));

    printf("[PASS] test_rns_identity_pairing_and_lua_crsf (SHA-256, 16B SINGLE dest_hash, RNSA/RNSP/RNSC, CRSF 0x7A/0x7B/0x21)\n");
}

/* ============================================================================
 * End-to-End EdgeTX Lua 5.2 VM Integration Test for tools/rns_elrs.lua
 * ============================================================================ */
struct LuaCrsfHarness {
    std::vector<std::vector<uint8_t>> tx_frames; /* Handset Lua -> Module/RX (0x7A) */
    std::vector<std::vector<uint8_t>> rx_frames; /* Module/RX -> Handset Lua (0x7B / 0x21) */
    std::vector<std::string>          lcd_lines;
    crsf_rns_status_t                 sim_status;
    crsf_rns_peer_info_t              sim_peer;
};

static LuaCrsfHarness g_lua_harness;

typedef struct lua_State lua_State;
typedef int (*lua_CFunction)(lua_State *L);
typedef ptrdiff_t lua_Integer;

static lua_State *(*p_luaL_newstate)(void) = NULL;
static void (*p_luaL_openlibs)(lua_State *L) = NULL;
static int (*p_luaL_loadstring)(lua_State *L, const char *s) = NULL;
static int (*p_lua_pcallk)(lua_State *L, int nargs, int nresults, int errfunc, int ctx, lua_CFunction k) = NULL;
static void (*p_lua_pushcclosure)(lua_State *L, lua_CFunction fn, int n) = NULL;
static void (*p_lua_setglobal)(lua_State *L, const char *name) = NULL;
static void (*p_lua_pushlstring)(lua_State *L, const char *s, size_t len) = NULL;
static void (*p_lua_pushnil)(lua_State *L) = NULL;
static const char *(*p_lua_tolstring)(lua_State *L, int idx, size_t *len) = NULL;
static void (*p_lua_close)(lua_State *L) = NULL;

static int host_crsf_tx_from_lua(lua_State *L) {
    size_t len = 0;
    const char *bytes = p_lua_tolstring(L, 1, &len);
    if (bytes && len >= 3U) {
        /* Wrap into full CRSF wire frame: [0]=dest, [1]=len(type+payload+crc), [2]=cmd, [3..]=payload, [last]=crc8 */
        uint8_t cmd = (uint8_t)bytes[0];
        const uint8_t *pay = (const uint8_t *)&bytes[1];
        size_t pay_len = len - 1U;
        std::vector<uint8_t> frame(pay_len + 4U);
        frame[0] = pay[0]; /* dest_addr (0xEE or 0xEC) */
        frame[1] = (uint8_t)(pay_len + 2U);
        frame[2] = cmd;
        memcpy(&frame[3], pay, pay_len);
        frame[3U + pay_len] = crsf_crc8(&frame[2], pay_len + 1U);

        /* Verify byte-by-byte stream parsing via crsf_parser_feed_byte */
        crsf_parser_t parser;
        crsf_parser_init(&parser);
        bool parsed_ok = false;
        for (size_t i = 0; i < frame.size(); i++) {
            if (crsf_parser_feed_byte(&parser, frame[i])) {
                parsed_ok = true;
            }
        }
        assert(parsed_ok);
        g_lua_harness.tx_frames.push_back(frame);

        /* Automatically generate the real C++ 0x7B response frame just like firmware process_crsf_rns_frame() */
        uint8_t subcmd = 0U;
        const uint8_t *cmd_pay = NULL;
        size_t cmd_pay_len = 0U;
        if (crsf_decode_rns_cmd_frame(frame.data(), frame.size(), NULL, NULL, &subcmd, &cmd_pay, &cmd_pay_len)) {
            uint8_t resp[64];
            size_t rlen = 0U;
            if (subcmd == CRSF_RNS_SUBCMD_GET_STATUS) {
                rlen = crsf_build_rns_status_frame(&g_lua_harness.sim_status, resp);
            } else if (subcmd == CRSF_RNS_SUBCMD_GET_PEER) {
                rlen = crsf_build_rns_peer_frame(&g_lua_harness.sim_peer, resp);
            } else {
                rlen = crsf_build_rns_ack_frame(subcmd, 0U, resp);
            }
            if (rlen > 0U) {
                g_lua_harness.rx_frames.push_back(std::vector<uint8_t>(resp, resp + rlen));
            }
        }
    }
    return 0;
}

static int host_crsf_rx_to_lua(lua_State *L) {
    if (g_lua_harness.rx_frames.empty()) {
        p_lua_pushnil(L);
        return 1;
    }
    std::vector<uint8_t> frame = g_lua_harness.rx_frames.front();
    g_lua_harness.rx_frames.erase(g_lua_harness.rx_frames.begin());
    /* Validate frame CRC before passing [cmd, payload...] to Lua */
    uint8_t flen = frame[1];
    assert((size_t)(flen + 2U) == frame.size());
    assert(crsf_crc8(&frame[2], (size_t)(flen - 1U)) == frame.back());
    /* Pass [type, payload...] (excluding trailing CRC8) */
    p_lua_pushlstring(L, (const char *)&frame[2], frame.size() - 3U);
    return 1;
}

static int host_lcd_clear(lua_State *) {
    g_lua_harness.lcd_lines.clear();
    return 0;
}

static int host_lcd_draw_text(lua_State *L) {
    size_t len = 0;
    const char *s = p_lua_tolstring(L, 1, &len);
    if (s) {
        g_lua_harness.lcd_lines.push_back(std::string(s, len));
    }
    return 0;
}

static bool lcd_contains(const char *substr) {
    for (const auto &line : g_lua_harness.lcd_lines) {
        if (line.find(substr) != std::string::npos) {
            return true;
        }
    }
    return false;
}

static void test_edgetx_lua_script_end_to_end(void) {
    void *dl = dlopen("liblua5.2.so.0", RTLD_LAZY);
    assert(dl != NULL && "liblua5.2.so.0 required for EdgeTX Lua 5.2 integration test");

    p_luaL_newstate   = (lua_State *(*)(void))dlsym(dl, "luaL_newstate");
    p_luaL_openlibs   = (void (*)(lua_State *))dlsym(dl, "luaL_openlibs");
    p_luaL_loadstring = (int (*)(lua_State *, const char *))dlsym(dl, "luaL_loadstring");
    p_lua_pcallk      = (int (*)(lua_State *, int, int, int, int, lua_CFunction))dlsym(dl, "lua_pcallk");
    p_lua_pushcclosure= (void (*)(lua_State *, lua_CFunction, int))dlsym(dl, "lua_pushcclosure");
    p_lua_setglobal   = (void (*)(lua_State *, const char *))dlsym(dl, "lua_setglobal");
    p_lua_pushlstring = (void (*)(lua_State *, const char *, size_t))dlsym(dl, "lua_pushlstring");
    p_lua_pushnil     = (void (*)(lua_State *))dlsym(dl, "lua_pushnil");
    p_lua_tolstring   = (const char *(*)(lua_State *, int, size_t *))dlsym(dl, "lua_tolstring");
    p_lua_close       = (void (*)(lua_State *))dlsym(dl, "lua_close");
    assert(p_luaL_newstate && p_luaL_openlibs && p_luaL_loadstring && p_lua_pcallk);

    g_lua_harness.tx_frames.clear();
    g_lua_harness.rx_frames.clear();
    g_lua_harness.lcd_lines.clear();

    /* Build real C++ Reticulum identities, status frame, peer frame, ACK frame, and FLIGHT_MODE frame */
    uint8_t uid[6];
    elrs_uid_from_bind_phrase("lr2021-elrs-demo", uid);
    const uint8_t hw_tx[8] = {0xFC, 0x6D, 0x1A, 0x37, 0x11, 0x22, 0x33, 0x44};
    const uint8_t hw_rx[8] = {0xD5, 0x6B, 0x69, 0x41, 0x55, 0x66, 0x77, 0x88};
    rns_node_identity_t tx_id, rx_id;
    rns_identity_init(&tx_id, hw_tx, uid, RNS_NODE_ROLE_TX, "LR2021-TX");
    rns_identity_init(&rx_id, hw_rx, uid, RNS_NODE_ROLE_RX, "LR2021-RX");

    crsf_rns_status_t st = {};
    memcpy(st.tx_dest, tx_id.dest_hash, 16U);
    memcpy(st.rx_dest, rx_id.dest_hash, 16U);
    st.app_mode         = 2U; /* ELRS_TX */
    st.lock_state       = 2U; /* LOCKED */
    st.elrs_rate_idx    = 3U;
    st.ifac_enabled     = true;
    st.rnode_freq_hz    = 915000000UL; /* Preset #3 (915.000 MHz) */
    st.rnode_bw_khz     = 250U;        /* Preset #2 (250.0 kHz) */
    st.rnode_sf         = 8U;
    st.rnode_cr         = 6U;
    st.rnode_txpower    = 17;
    st.peer_count       = 1U;
    st.selected_peer    = 0U;
    st.last_ping_rtt_ms = 34U;
    st.tx_msgs          = 19U;
    st.rx_msgs          = 18U;
    st.relayed_msgs     = 5U;
    g_lua_harness.sim_status = st;

    crsf_rns_peer_info_t pi = {};
    pi.peer_idx    = 0U;
    pi.total_peers = 1U;
    pi.is_selected = true;
    pi.role        = RNS_NODE_ROLE_RX;
    memcpy(pi.dest_hash, rx_id.dest_hash, 16U);
    strcpy(pi.name, "LR2021-RX");
    pi.hops        = 2U;
    pi.rssi_dbm    = -58;
    pi.snr_db      = 9;
    pi.armed       = true;
    pi.custom_mode = 5U;
    pi.vbatt_mv    = 25200U;
    pi.age_s       = 1U;
    g_lua_harness.sim_peer = pi;

    uint8_t buf[64];
    size_t len = crsf_build_flight_mode_frame("RNS:4cef..c149 1p", buf);
    g_lua_harness.rx_frames.push_back(std::vector<uint8_t>(buf, buf + len));

    lua_State *L = p_luaL_newstate();
    p_luaL_openlibs(L);
    p_lua_pushcclosure(L, host_crsf_tx_from_lua, 0);
    p_lua_setglobal(L, "__host_crsf_tx");
    p_lua_pushcclosure(L, host_crsf_rx_to_lua, 0);
    p_lua_setglobal(L, "__host_crsf_rx");
    p_lua_pushcclosure(L, host_lcd_clear, 0);
    p_lua_setglobal(L, "__host_lcd_clear");
    p_lua_pushcclosure(L, host_lcd_draw_text, 0);
    p_lua_setglobal(L, "__host_lcd_draw_text");

    const char *lua_driver = R"LUA(
      LCD_W = 480
      LCD_H = 272
      BOLD = 1
      INVERS = 2
      BLINK = 4
      SOLID = 0
      EVT_VIRTUAL_NEXT_PAGE = 101
      EVT_VIRTUAL_PREV_PAGE = 102
      EVT_PAGE_BREAK        = 103
      EVT_VIRTUAL_NEXT      = 104
      EVT_VIRTUAL_PREV      = 105
      EVT_VIRTUAL_ENTER     = 106
      EVT_VIRTUAL_EXIT      = 107

      local sim_time_10ms = 100
      function getTime()
        return sim_time_10ms
      end
      function advanceTimeMs(ms)
        sim_time_10ms = sim_time_10ms + math.floor(ms / 10)
      end

      lcd = {
        clear = function() __host_lcd_clear() end,
        drawText = function(x, y, s, attr) __host_lcd_draw_text(tostring(s)) end,
        drawLine = function(x1, y1, x2, y2, pat, flg) end
      }

      function crossfireTelemetryPush(cmd, payload)
        local chars = { string.char(cmd) }
        for i = 1, #payload do
          chars[#chars + 1] = string.char(payload[i])
        end
        __host_crsf_tx(table.concat(chars))
        return true
      end

      function crossfireTelemetryPop()
        local raw = __host_crsf_rx()
        if not raw or #raw < 1 then return nil, nil end
        local cmd = string.byte(raw, 1)
        local data = {}
        for i = 2, #raw do
          data[#data + 1] = string.byte(raw, i)
        end
        return cmd, data
      end

      tool = dofile("tools/rns_elrs.lua")
      tool.init()
      -- 1. Page 1: Pop RESP_STATUS + FLIGHT_MODE and render Page 1
      tool.run(0)
    )LUA";

    int rc = p_luaL_loadstring(L, lua_driver);
    assert(rc == 0);
    rc = p_lua_pcallk(L, 0, 0, 0, 0, NULL);
    if (rc != 0) {
        size_t elen = 0;
        const char *err = p_lua_tolstring(L, -1, &elen);
        fprintf(stderr, "Lua error: %s\n", err ? err : "unknown");
        assert(rc == 0);
    }

    /* Verify init() sent SUBCMD_GET_STATUS (0x01) to 0xEE (TX Module) */
    assert(!g_lua_harness.tx_frames.empty());
    uint8_t dest = 0, orig = 0, subcmd = 0;
    const uint8_t *pay = NULL;
    size_t pay_len = 0;
    assert(crsf_decode_rns_cmd_frame(g_lua_harness.tx_frames[0].data(), g_lua_harness.tx_frames[0].size(),
                                     &dest, &orig, &subcmd, &pay, &pay_len));
    assert(dest == CRSF_ADDRESS_CRSF_TRANSMITTER && orig == CRSF_ADDRESS_HANDSET);
    assert(subcmd == CRSF_RNS_SUBCMD_GET_STATUS && pay_len == 0U);

    /* Verify Page 1 rendered the decoded C++ status & 0x21 FLIGHT_MODE string */
    assert(lcd_contains("Mode:ELRS_TX(LOCKED) Peers:1 RTT:34ms"));
    assert(lcd_contains("RNS:4cef..c149 1p"));
    assert(lcd_contains("[TX-MOD(0xEE)]"));

    /* 2. Test Page 1 buttons: [Pair & Sync RX], [Announce], and [OTA-RX(0xEC)] toggle */
    g_lua_harness.tx_frames.clear();
    const char *lua_page1_actions = R"LUA(
      -- Cursor 1: [Pair & Sync RX]
      tool.run(EVT_VIRTUAL_ENTER)
      -- Cursor 2: [Announce]
      tool.run(EVT_VIRTUAL_NEXT)
      tool.run(EVT_VIRTUAL_ENTER)
      -- Cursor 3: Toggle [OTA-RX(0xEC)] and trigger Announce addressed to 0xEC
      tool.run(EVT_VIRTUAL_NEXT)
      tool.run(EVT_VIRTUAL_ENTER)
      tool.run(EVT_VIRTUAL_PREV)
      tool.run(EVT_VIRTUAL_ENTER)
    )LUA";
    assert(p_luaL_loadstring(L, lua_page1_actions) == 0);
    assert(p_lua_pcallk(L, 0, 0, 0, 0, NULL) == 0);
    assert(g_lua_harness.tx_frames.size() == 3U);

    /* Frame 0: SUBCMD_PAIR_RX to 0xEE */
    assert(crsf_decode_rns_cmd_frame(g_lua_harness.tx_frames[0].data(), g_lua_harness.tx_frames[0].size(),
                                     &dest, &orig, &subcmd, &pay, &pay_len));
    assert(dest == CRSF_ADDRESS_CRSF_TRANSMITTER && subcmd == CRSF_RNS_SUBCMD_PAIR_RX);

    /* Frame 1: SUBCMD_ACTION(ACT_ANNOUNCE) to 0xEE */
    assert(crsf_decode_rns_cmd_frame(g_lua_harness.tx_frames[1].data(), g_lua_harness.tx_frames[1].size(),
                                     &dest, &orig, &subcmd, &pay, &pay_len));
    assert(dest == CRSF_ADDRESS_CRSF_TRANSMITTER && subcmd == CRSF_RNS_SUBCMD_ACTION && pay_len == 1U && pay[0] == CRSF_RNS_ACT_ANNOUNCE);

    /* Frame 2: SUBCMD_ACTION(ACT_ANNOUNCE) tunneled over-the-air to 0xEC (CRSF_ADDRESS_CRSF_RECEIVER) */
    assert(crsf_decode_rns_cmd_frame(g_lua_harness.tx_frames[2].data(), g_lua_harness.tx_frames[2].size(),
                                     &dest, &orig, &subcmd, &pay, &pay_len));
    assert(dest == CRSF_ADDRESS_CRSF_RECEIVER && subcmd == CRSF_RNS_SUBCMD_ACTION && pay_len == 1U && pay[0] == CRSF_RNS_ACT_ANNOUNCE);
    assert(lcd_contains("[OTA-RX(0xEC)]"));

    /* 3. Switch to Page 2: Verify initial sync from C++ status (915.000 MHz, 250.0 kHz, SF8, CR 4/6, 17dBm, IFAC ON),
     * then edit Freq -> 2440.000 MHz, BW -> 500.0 kHz, SF -> 9, CR -> 4/7, Pwr -> 18 dBm, IFAC -> OFF, and Apply!
     */
    g_lua_harness.tx_frames.clear();
    const char *lua_page2 = R"LUA(
      tool.run(EVT_VIRTUAL_NEXT_PAGE)
    )LUA";
    assert(p_luaL_loadstring(L, lua_page2) == 0);
    assert(p_lua_pcallk(L, 0, 0, 0, 0, NULL) == 0);
    assert(lcd_contains("1. Freq: 915.000 MHz"));
    assert(lcd_contains("2. BW:   250.0 kHz"));
    assert(lcd_contains("3. SF: SF8"));
    assert(lcd_contains("4. CR: 4/6"));
    assert(lcd_contains("5. Pwr: 17dBm"));
    assert(lcd_contains("6. IFAC: ON"));

    const char *lua_page2_edit = R"LUA(
      -- 1. Edit Freq: 915 MHz (#3) -> 2440 MHz (#4)
      tool.run(EVT_VIRTUAL_ENTER)
      tool.run(EVT_VIRTUAL_NEXT)
      tool.run(EVT_VIRTUAL_ENTER)
      -- 2. Edit BW: 250 kHz (#2) -> 500 kHz (#3)
      tool.run(EVT_VIRTUAL_NEXT)
      tool.run(EVT_VIRTUAL_ENTER)
      tool.run(EVT_VIRTUAL_NEXT)
      tool.run(EVT_VIRTUAL_ENTER)
      -- 3. Edit SF: 8 -> 9
      tool.run(EVT_VIRTUAL_NEXT)
      tool.run(EVT_VIRTUAL_ENTER)
      tool.run(EVT_VIRTUAL_NEXT)
      tool.run(EVT_VIRTUAL_ENTER)
      -- 4. Edit CR: 6 -> 7
      tool.run(EVT_VIRTUAL_NEXT)
      tool.run(EVT_VIRTUAL_ENTER)
      tool.run(EVT_VIRTUAL_NEXT)
      tool.run(EVT_VIRTUAL_ENTER)
      -- 5. Edit Pwr: 17 -> 18 dBm
      tool.run(EVT_VIRTUAL_NEXT)
      tool.run(EVT_VIRTUAL_ENTER)
      tool.run(EVT_VIRTUAL_NEXT)
      tool.run(EVT_VIRTUAL_ENTER)
      -- 6. Edit IFAC: ON -> OFF
      tool.run(EVT_VIRTUAL_NEXT)
      tool.run(EVT_VIRTUAL_ENTER)
      tool.run(EVT_VIRTUAL_NEXT)
      tool.run(EVT_VIRTUAL_ENTER)
      -- 7. [Apply & Push OTA to RX]
      tool.run(EVT_VIRTUAL_NEXT)
      tool.run(EVT_VIRTUAL_ENTER)
    )LUA";
    assert(p_luaL_loadstring(L, lua_page2_edit) == 0);
    assert(p_lua_pcallk(L, 0, 0, 0, 0, NULL) == 0);
    assert(g_lua_harness.tx_frames.size() == 2U);

    crsf_rns_cfg_cmd_t lua_cfg = {};
    assert(crsf_decode_rns_set_cfg_frame(g_lua_harness.tx_frames[0].data(), g_lua_harness.tx_frames[0].size(), &lua_cfg));
    assert(lua_cfg.freq_hz == 2440000000UL);
    assert(lua_cfg.bw_hz == 500000UL);
    assert(lua_cfg.sf == 9U);
    assert(lua_cfg.cr == 7U);
    assert(lua_cfg.txpower_dbm == 18);
    assert(!lua_cfg.ifac_enabled);

    assert(crsf_decode_rns_cmd_frame(g_lua_harness.tx_frames[1].data(), g_lua_harness.tx_frames[1].size(),
                                     &dest, &orig, &subcmd, &pay, &pay_len));
    assert(subcmd == CRSF_RNS_SUBCMD_PAIR_RX);

    /* 4. Switch to Page 3 (Mesh Peers): verify round-robin GET_PEER poll, peer rendering (2h, -58dBm, 25.2V), and SELECT_PEER */
    g_lua_harness.tx_frames.clear();
    const char *lua_page3 = R"LUA(
      LCD_W = 128 -- Also verify B&W 128x64 layout rendering
      advanceTimeMs(400)
      tool.run(EVT_VIRTUAL_NEXT_PAGE)
      tool.run(EVT_VIRTUAL_ENTER)
    )LUA";
    assert(p_luaL_loadstring(L, lua_page3) == 0);
    assert(p_lua_pcallk(L, 0, 0, 0, 0, NULL) == 0);
    assert(lcd_contains("2h -58dBm 25.2V"));
    assert(g_lua_harness.tx_frames.size() == 2U);
    /* First frame is periodic SUBCMD_GET_PEER(0), second is SUBCMD_SELECT_PEER(0) */
    assert(crsf_decode_rns_cmd_frame(g_lua_harness.tx_frames[0].data(), g_lua_harness.tx_frames[0].size(),
                                     &dest, &orig, &subcmd, &pay, &pay_len));
    assert(subcmd == CRSF_RNS_SUBCMD_GET_PEER && pay_len == 1U && pay[0] == 0U);
    assert(crsf_decode_rns_cmd_frame(g_lua_harness.tx_frames[1].data(), g_lua_harness.tx_frames[1].size(),
                                     &dest, &orig, &subcmd, &pay, &pay_len));
    assert(subcmd == CRSF_RNS_SUBCMD_SELECT_PEER && pay_len == 1U && pay[0] == 0U);

    /* 5. Switch to Page 4 (Targeted RX Cmd): press all 9 action buttons (Ping RX..DISARM + Form:VEE/LINE/HOLD) */
    g_lua_harness.tx_frames.clear();
    const char *lua_page4 = R"LUA(
      LCD_W = 480
      tool.run(EVT_VIRTUAL_NEXT_PAGE)
      for i = 1, 9 do
        tool.run(EVT_VIRTUAL_ENTER)
        tool.run(EVT_VIRTUAL_NEXT)
      end
    )LUA";
    assert(p_luaL_loadstring(L, lua_page4) == 0);
    assert(p_lua_pcallk(L, 0, 0, 0, 0, NULL) == 0);
    assert(lcd_contains("Ping RTT:34ms  TunTX:19 TunRX:18 Rly:5"));
    assert(lcd_contains("[Form:VEE]"));
    assert(lcd_contains("[Form:LINE]"));
    assert(lcd_contains("[Form:HOLD]"));
    assert(lcd_contains("OK (0x06)"));
    assert(g_lua_harness.tx_frames.size() == 9U);
    const uint8_t expected_acts[9] = {
        CRSF_RNS_ACT_PING,     CRSF_RNS_ACT_LOITER,    CRSF_RNS_ACT_RTL,
        CRSF_RNS_ACT_LAND,     CRSF_RNS_ACT_ARM,       CRSF_RNS_ACT_DISARM,
        CRSF_RNS_ACT_FORM_VEE, CRSF_RNS_ACT_FORM_LINE, CRSF_RNS_ACT_FORM_HOLD
    };
    for (int i = 0; i < 9; i++) {
        assert(crsf_decode_rns_cmd_frame(g_lua_harness.tx_frames[i].data(), g_lua_harness.tx_frames[i].size(),
                                         &dest, &orig, &subcmd, &pay, &pay_len));
        assert(subcmd == CRSF_RNS_SUBCMD_ACTION && pay_len == 1U && pay[0] == expected_acts[i]);
    }

    p_lua_close(L);
    dlclose(dl);
    printf("[PASS] test_edgetx_lua_script_end_to_end (Lua 5.2 VM executing tools/rns_elrs.lua Pages 1..4 <-> C++ CRSF 0x7A/0x7B/0x21)\n");
}

/* ---------------------------------------------------------------------------
 * 12. Multi-Drone Reticulum Mesh Control Distribution, Relay & Formation Test
 * --------------------------------------------------------------------------- */
static void test_multi_drone_mesh_and_formation(void) {
    /* 1. Verify VEL (LOCAL_NED) and BVL (BODY_OFFSET_NED) command parsing + MAVLink #84 generation */
    uint8_t pkt[128];
    size_t len = rns_build_drone_command_pkt("BVL 250 -100 0 1500", pkt, sizeof(pkt));
    assert(len > RNS_MIN_PACKET_LEN);

    rns_drone_cmd_t dcmd;
    assert(rns_parse_drone_command_pkt(pkt, len, &dcmd));
    assert(dcmd.type == RNS_DRONE_CMD_VEL_NED);
    assert(dcmd.use_body_frame == true);
    assert(dcmd.vx_cms == 250);
    assert(dcmd.vy_cms == -100);
    assert(dcmd.vz_cms == 0);
    assert(dcmd.has_yaw_rate && dcmd.yaw_rate_cdps == 1500);

    uint8_t mav[72];
    size_t mav_len = rns_build_mavlink_command(&dcmd, 7U, mav, sizeof(mav));
    /* MAVLink v2 SET_POSITION_TARGET_LOCAL_NED (#84): 10B hdr + 53B payload + 2B CRC = 65B */
    assert(mav_len == 65U);
    assert(mav[0] == 0xFDU && mav[1] == 53U && mav[4] == 7U);
    assert(mav[7] == 84U && mav[8] == 0U && mav[9] == 0U);
    assert(mav[10 + 52] == 9U); /* MAV_FRAME_BODY_OFFSET_NED = 9 */
    /* type_mask at payload[48..49] should be 0x05C7 when yaw_rate is non-zero */
    uint16_t mask = (uint16_t)mav[10 + 48] | ((uint16_t)mav[10 + 49] << 8U);
    assert(mask == 0x05C7U);

    /* 2. Verify 3-Drone Multi-Hop Relay + Vee Formation Slot Computation (Leader -> Drone 1 Relay -> Drone 2) */
    uint8_t leader_hw[8] = {0x10, 0x00, 0x00, 0x01, 0xAA, 0xBB, 0xCC, 0xDD};
    uint8_t uid[6]       = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66};
    rns_node_identity_t leader_id;
    rns_identity_init(&leader_id, leader_hw, uid, RNS_NODE_ROLE_TX, "LEADER-0");

    rns_formation_pkt_t fp = {};
    memcpy(fp.leader_dest, leader_id.dest_hash, 16U);
    fp.seq           = 42U;
    fp.shape         = RNS_FORM_VEE;
    fp.spacing_dm    = 100U;         /* 10.0 m spacing */
    fp.leader_lat_e7 = -353632620L;  /* -35.3632620 deg (Canberra SITL default) */
    fp.leader_lon_e7 = 1491652370L;  /* 149.1652370 deg */
    fp.leader_alt_mm = 25000L;       /* 25.0 m relative alt */
    fp.vx_cms        = 400;          /* 4.0 m/s North */
    fp.vy_cms        = 0;            /* 0.0 m/s East */
    fp.vz_cms        = 0;
    fp.heading_cdeg  = 0U;           /* Heading 0 deg (due North) */

    uint8_t form_pkt[64];
    size_t flen = rns_build_formation_pkt(&fp, form_pkt, sizeof(form_pkt));
    assert(flen == 64U);
    assert(form_pkt[1] == 0U); /* hops = 0 */

    /* Drone 1 (Slot 1: Left Wingman) receives packet via ELRS DATA Tunnel, parses it, and relays it over Reticulum LoRa */
    elrs_ota_state_t ota_tx, ota_rx;
    elrs_ota_init(&ota_tx, uid, ELRS_OTA8_PACKET_SIZE, ELRS_SW_HYBRID_OR_16CH);
    elrs_ota_init(&ota_rx, uid, ELRS_OTA8_PACKET_SIZE, ELRS_SW_HYBRID_OR_16CH);
    elrs_data_tunnel_t tun_tx, tun_rx;
    elrs_tunnel_init(&tun_tx, 1U);
    elrs_tunnel_init(&tun_rx, 1U);

    assert(elrs_tunnel_enqueue_tx(&tun_tx, form_pkt, (uint16_t)flen, true));
    uint8_t reassembled[ELRS_TUNNEL_MAX_MTU];
    uint16_t reassembled_len = 0U;
    bool got_msg = false;
    while (elrs_tunnel_has_pending_tx(&tun_tx)) {
        uint8_t ota_frame[ELRS_OTA8_PACKET_SIZE];
        ota_tx.ota_nonce++;
        ota_rx.ota_nonce = ota_tx.ota_nonce;
        assert(elrs_tunnel_next_tx_frame(&tun_tx, &ota_tx, ota_frame));
        if (elrs_tunnel_process_rx_frame(&tun_rx, ota_frame, true, reassembled, &reassembled_len)) {
            got_msg = true;
        }
    }
    assert(got_msg && reassembled_len == 64U);

    /* Drone 1 relays the packet into the multi-drone Reticulum mesh (hops: 0 -> 1) */
    rns_relay_table_t drone1_relay, drone2_relay;
    rns_relay_init(&drone1_relay);
    rns_relay_init(&drone2_relay);
    assert(rns_relay_should_forward(&drone1_relay, reassembled, reassembled_len, 1000U));
    assert(reassembled[1] == 1U); /* hops incremented to 1 */

    /* Drone 2 relays the packet further down the chain (hops: 1 -> 2), while Drone 1 drops the echo */
    assert(rns_relay_should_forward(&drone2_relay, reassembled, reassembled_len, 1150U));
    assert(reassembled[1] == 2U); /* hops incremented to 2 */
    assert(!rns_relay_should_forward(&drone1_relay, reassembled, reassembled_len, 1200U)); /* Loop prevented! */

    /* Verify SHA-256 integrity tag still validates after 2 relay hops! */
    rns_formation_pkt_t rx_fp;
    assert(rns_parse_formation_pkt(reassembled, reassembled_len, &rx_fp));
    assert(rx_fp.seq == 42U && rx_fp.shape == RNS_FORM_VEE && rx_fp.spacing_dm == 100U);

    /* Compute Slot 1 (Left Wingman: 10m behind, 10m West) and Slot 2 (Right Wingman: 10m behind, 10m East) */
    rns_drone_cmd_t slot1_cmd, slot2_cmd;
    assert(rns_compute_formation_slot_cmd(&rx_fp, 1U, &slot1_cmd));
    assert(rns_compute_formation_slot_cmd(&rx_fp, 2U, &slot2_cmd));
    assert(slot1_cmd.type == RNS_DRONE_CMD_GLOBAL_POS_VEL);
    assert(slot2_cmd.type == RNS_DRONE_CMD_GLOBAL_POS_VEL);

    /* Heading is 0 deg (North): both Slot 1 and Slot 2 must be ~10m South of Leader (lat < leader_lat),
     * Slot 1 is West (lon < leader_lon) and Slot 2 is symmetric East (lon > leader_lon) */
    assert(slot1_cmd.lat_deg_e7 < fp.leader_lat_e7);
    assert(slot1_cmd.lat_deg_e7 == slot2_cmd.lat_deg_e7);
    assert(slot1_cmd.lon_deg_e7 < fp.leader_lon_e7);
    assert(slot2_cmd.lon_deg_e7 > fp.leader_lon_e7);
    assert(abs((slot2_cmd.lon_deg_e7 - fp.leader_lon_e7) + (slot1_cmd.lon_deg_e7 - fp.leader_lon_e7)) <= 1);
    assert(slot1_cmd.vx_cms == 400);

    /* Verify MAVLink #86 SET_POSITION_TARGET_GLOBAL_INT frame (65 bytes, CRC_EXTRA=5) */
    size_t m86_len = rns_build_mavlink_command(&slot1_cmd, 9U, mav, sizeof(mav));
    assert(m86_len == 65U);
    assert(mav[0] == 0xFDU && mav[1] == 53U && mav[7] == 86U);
    assert(mav[10 + 52] == 6U); /* MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6 */

    /* Verify HOLD_BRAKE emergency formation command produces zero-velocity body brake (#84) */
    fp.shape = RNS_FORM_HOLD_BRAKE;
    flen = rns_build_formation_pkt(&fp, form_pkt, sizeof(form_pkt));
    assert(rns_parse_formation_pkt(form_pkt, flen, &rx_fp));
    rns_drone_cmd_t brake_cmd;
    assert(rns_compute_formation_slot_cmd(&rx_fp, 2U, &brake_cmd));
    assert(brake_cmd.type == RNS_DRONE_CMD_VEL_NED);
    assert(brake_cmd.vx_cms == 0 && brake_cmd.vy_cms == 0 && brake_cmd.vz_cms == 0);

    /* 3. Verify 2-node multi-hop ping-pong bounce chain with delay and hop/TTL expiration */
    rns_relay_table_t node_a, node_b;
    rns_relay_init(&node_a);
    rns_relay_init(&node_b);
    rns_relay_configure_chain_test(&node_a, true, 4U, 150U, 1000UL);
    rns_relay_configure_chain_test(&node_b, true, 4U, 150U, 1000UL);

    fp.seq   = 77U;
    fp.shape = RNS_FORM_VEE;
    flen = rns_build_formation_pkt(&fp, form_pkt, sizeof(form_pkt));
    assert(flen == 64U && form_pkt[1] == 0U);

    /* Node A originates packet at t=100ms (hops=0) */
    rns_relay_record_sent(&node_a, form_pkt, flen, 100U);
    /* Node A must NOT re-relay its own hops=0 transmission */
    assert(!rns_relay_should_forward(&node_a, form_pkt, flen, 105U));

    /* Hop 0 -> 1 on Node B at t=140ms */
    assert(rns_relay_should_forward(&node_b, form_pkt, flen, 140U));
    assert(form_pkt[1] == 1U);
    /* Duplicate at hop 1 on Node B must be dropped */
    assert(!rns_relay_should_forward(&node_b, form_pkt, flen, 145U));

    /* Hop 1 -> 2 back on Node A at t=300ms */
    assert(rns_relay_should_forward(&node_a, form_pkt, flen, 300U));
    assert(form_pkt[1] == 2U);

    /* Hop 2 -> 3 back on Node B at t=460ms */
    assert(rns_relay_should_forward(&node_b, form_pkt, flen, 460U));
    assert(form_pkt[1] == 3U);

    /* Hop 3 -> 4 back on Node A at t=620ms */
    assert(rns_relay_should_forward(&node_a, form_pkt, flen, 620U));
    assert(form_pkt[1] == 4U);

    /* At t=780ms, Node B receives hops=4 == max_hops(4): MUST EXPIRE! */
    assert(!rns_relay_should_forward(&node_b, form_pkt, flen, 780U));
    assert(node_b.last_rx_expired && node_b.expired_hop_count == 1U);

    /* Verify wall-clock TTL expiration (`ttl_window_ms = 500ms`) before `max_hops` is reached */
    rns_relay_configure_chain_test(&node_a, true, 6U, 200U, 500UL);
    rns_relay_configure_chain_test(&node_b, true, 6U, 200U, 500UL);
    fp.seq = 78U;
    flen = rns_build_formation_pkt(&fp, form_pkt, sizeof(form_pkt));
    rns_relay_record_sent(&node_a, form_pkt, flen, 1000U);
    assert(rns_relay_should_forward(&node_b, form_pkt, flen, 1100U)); /* hops 0 -> 1 */
    assert(rns_relay_should_forward(&node_a, form_pkt, flen, 1350U)); /* hops 1 -> 2 (age 350ms <= 500ms) */
    /* Next bounce arrives at Node B at t=1650ms (age 550ms > 500ms on Node B): MUST EXPIRE on TTL! */
    assert(!rns_relay_should_forward(&node_b, form_pkt, flen, 1650U));
    assert(node_b.last_rx_expired && node_b.expired_hop_count == 1U);

    printf("[PASS] test_multi_drone_mesh_and_formation (RNSF 64B formation + ping-pong hop/TTL chain + MAVLink #84/#86)\n");
}

/* ---------------------------------------------------------------------------
 * 13. Deterministic Slot-TDMA Relay & 64B Multi-Drone Bundle ("RNSM") Test
 * --------------------------------------------------------------------------- */
static void test_slot_tdma_and_multi_drone_bundle(void) {
    /* 1. Build a 64B RNSM packet with 4 independent drone slot commands */
    rns_multi_drone_pkt_t mp = {};
    mp.seq          = 19U;
    mp.slot_mask    = 0x0FU;         /* Slots 1..4 all active */
    mp.ref_lat_e7   = -353632620L;
    mp.ref_lon_e7   = 1491652370L;
    mp.ref_alt_mm   = 20000L;        /* 20.0 m reference altitude */
    mp.heading_cdeg = 9000U;         /* 90.0 deg heading */

    /* Slot 1: 3D NED offset (-10.0m N, -15.0m E, +5m Alt) -> #86 GLOBAL_POS_VEL */
    mp.slots[0].action   = RNS_MULTI_ACT_OFFSET_NED;
    mp.slots[0].n_val_dm = -100;
    mp.slots[0].e_val_dm = -150;
    mp.slots[0].d_val    = 5;

    /* Slot 2: NED velocity vector (+5.0 m/s N, -2.0 m/s E, -1.0 m/s D) -> #84 VEL_NED */
    mp.slots[1].action   = RNS_MULTI_ACT_VEL_NED;
    mp.slots[1].n_val_dm = 50;
    mp.slots[1].e_val_dm = -20;
    mp.slots[1].d_val    = -10;

    /* Slot 3: LOITER -> #76 COMMAND_LONG (MAV_CMD_NAV_LOITER_UNLIM) */
    mp.slots[2].action   = RNS_MULTI_ACT_LOITER;

    /* Slot 4: RTL -> #76 COMMAND_LONG (MAV_CMD_NAV_RETURN_TO_LAUNCH) */
    mp.slots[3].action   = RNS_MULTI_ACT_RTL;

    uint8_t raw[64];
    size_t len = rns_build_multi_drone_pkt(&mp, raw, sizeof(raw));
    assert(len == 64U);
    assert(raw[18] == 0xA8U && memcmp(&raw[19], "RNSM", 4U) == 0);

    /* 2. Verify Deterministic Slot-TDMA Relay Scheduling (`Slots 1..4` -> 0, 45, 90, 135 ms) */
    rns_relay_table_t rt;
    rns_relay_init(&rt);
    assert(rns_relay_compute_slot_delay_ms(&rt, raw, len, 1U, 0x5AU) == 0UL);
    assert(rns_relay_compute_slot_delay_ms(&rt, raw, len, 2U, 0x5AU) == 45UL);
    assert(rns_relay_compute_slot_delay_ms(&rt, raw, len, 3U, 0x5AU) == 90UL);
    assert(rns_relay_compute_slot_delay_ms(&rt, raw, len, 4U, 0x5AU) == 135UL);
    /* Slot 0 (Leader) or non-swarm packets fall back to CSMA jitter (120 + (0x5A & 0x7F) = 210 ms) */
    assert(rns_relay_compute_slot_delay_ms(&rt, raw, len, 0U, 0x5AU) == 210UL);

    /* 3. Forward RNSM across 2 relay hops and verify SHA-256 tag still validates */
    assert(rns_relay_should_forward(&rt, raw, len, 500U));
    assert(raw[1] == 1U);

    rns_multi_drone_pkt_t rx_mp = {};
    assert(rns_parse_multi_drone_pkt(raw, len, &rx_mp));
    assert(rx_mp.seq == 19U && rx_mp.slot_mask == 0x0FU);
    assert(rx_mp.ref_lat_e7 == -353632620L && rx_mp.ref_alt_mm == 20000L);

    /* Extract per-slot commands for Slots 1..4 */
    rns_drone_cmd_t c1, c2, c3, c4;
    assert(rns_extract_multi_slot_cmd(&rx_mp, 1U, &c1));
    assert(c1.type == RNS_DRONE_CMD_GLOBAL_POS_VEL);
    assert(c1.lat_deg_e7 < rx_mp.ref_lat_e7 && c1.lon_deg_e7 < rx_mp.ref_lon_e7);
    assert(c1.alt_m == 25); /* 20m ref + 5m offset */

    assert(rns_extract_multi_slot_cmd(&rx_mp, 2U, &c2));
    assert(c2.type == RNS_DRONE_CMD_VEL_NED);
    assert(c2.vx_cms == 500 && c2.vy_cms == -200 && c2.vz_cms == -100);

    assert(rns_extract_multi_slot_cmd(&rx_mp, 3U, &c3));
    assert(c3.type == RNS_DRONE_CMD_LOITER);

    assert(rns_extract_multi_slot_cmd(&rx_mp, 4U, &c4));
    assert(c4.type == RNS_DRONE_CMD_RTL);

    /* Corrupt 1 byte in payload and verify SHA-256 tag rejects it */
    raw[40] ^= 0x01U;
    assert(!rns_parse_multi_drone_pkt(raw, len, &rx_mp));

    printf("[PASS] test_slot_tdma_and_multi_drone_bundle (Deterministic Slot-TDMA + 64B RNSM multi-slot bundle)\n");
}

int main(void) {
    test_uid_md5();
    test_fhss_invariants();
    test_ota4_and_ota8_roundtrip();
    test_crsf_framing();
    test_demo_live_packets();
    test_rnode_kiss_framing();
    test_rns_relay_dedup_and_ifac();
    test_fc_mavlink_and_beacons();
    test_elrs_data_tunnel();
    test_rns_identity_pairing_and_lua_crsf();
    test_edgetx_lua_script_end_to_end();
    test_multi_drone_mesh_and_formation();
    test_slot_tdma_and_multi_drone_bundle();
    printf("All host unit tests PASSED.\n");
    return 0;
}
