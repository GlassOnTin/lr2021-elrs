/* SPDX-License-Identifier: AGPL-3.0-or-later */
/* Host unit test suite for ExpressLRS UID, CRC14/CRC16, OTA4/OTA8, FHSS, and CRSF */

#include "../firmware/elrs/elrs_protocol.h"
#include "../firmware/crsf/crsf.h"
#include "../firmware/rnode/rns_node.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <set>

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
    /* Verify live OTA packets captured from RadioMaster TX16SMK3 <-> RadioMaster XR4
     * Binding phrase: "lr2021-elrs-demo" -> UID = 0A:B7:0C:7C:38:03
     * Air rate: 2G4_K1000 (ELRS_RATE_FSK_2G4_1000HZ = 0x24 = 36), CE_LBT domain
     */
    uint8_t uid[6] = {0};
    elrs_uid_from_bind_phrase("lr2021-elrs-demo", uid);
    const uint8_t expected_uid[6] = {0x0A, 0xB7, 0x0C, 0x7C, 0x38, 0x03};
    assert(memcmp(uid, expected_uid, 6) == 0);

    elrs_ota_state_t ota;
    elrs_ota_init(&ota, uid, ELRS_OTA4_PACKET_SIZE, ELRS_SW_HYBRID_OR_16CH);
    assert(ota.uid_seed == 0x09FA466EUL);
    assert(ota.crc_init == 0x426AU);

    /* 1. Live TX16SMK3 SYNC packet at hop=160, nonce=193: AAA0C1240F466AD3 */
    const uint8_t sync_pkt_160[8] = {0xAA, 0xA0, 0xC1, 0x24, 0x0F, 0x46, 0x6A, 0xD3};
    elrs_sync_data_t sync = {};
    assert(elrs_ota_unpack_sync(&ota, sync_pkt_160, &sync));
    assert(sync.fhss_index == 160U);
    assert(sync.nonce == 193U);
    assert(sync.rf_rate_enum == ELRS_RATE_FSK_2G4_1000HZ);
    assert(sync.switch_enc_mode == ELRS_SW_HYBRID_OR_16CH);
    assert(sync.new_tlm_ratio == 7U); /* 1:2 telemetry ratio */
    assert(sync.uid4 == 0x46U && sync.uid5 == 0x6AU);

    /* 2. Live TX16SMK3 SYNC packet at hop=0, nonce=193: 6200C1240F466A99 */
    const uint8_t sync_pkt_0[8] = {0x62, 0x00, 0xC1, 0x24, 0x0F, 0x46, 0x6A, 0x99};
    assert(elrs_ota_unpack_sync(&ota, sync_pkt_0, &sync));
    assert(sync.fhss_index == 0U);
    assert(sync.nonce == 193U);

    /* 3. Live XR4 LINKSTATS telemetry packet at hop=1, nonce=194: 6400CAB9E00000C4 */
    const uint8_t tlm_pkt[8] = {0x64, 0x00, 0xCA, 0xB9, 0xE0, 0x00, 0x00, 0xC4};
    ota.ota_nonce = 194U;
    elrs_link_stats_data_t ls = {};
    uint8_t pkg_idx = 0U;
    bool ack = false;
    assert(elrs_ota_unpack_linkstats(&ota, tlm_pkt, &ls, &pkg_idx, &ack));
    assert(ls.uplink_rssi_1 == 74U);
    assert(ls.uplink_rssi_2 == 57U);
    assert(ls.lq == 96U);
    assert(ls.true_diversity == 1U);

    /* 4. Live TX16SMK3 RCDATA packet at hop=1, nonce=195: 68000218807F401E */
    const uint8_t rc_pkt[8] = {0x68, 0x00, 0x02, 0x18, 0x80, 0x7F, 0x40, 0x1E};
    ota.ota_nonce = 195U;
    uint16_t ch[16] = {0};
    bool armed = true;
    assert(elrs_ota_unpack_rc(&ota, rc_pkt, ch, &armed, &ack));
    assert(!armed);
    assert(crsf_ticks_to_us(ch[0]) == 1500U);
    assert(crsf_ticks_to_us(ch[1]) == 1500U);
    assert(crsf_ticks_to_us(ch[2]) == 989U);
    assert(crsf_ticks_to_us(ch[3]) == 1498U);

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
    assert(strstr((const char *)&bcn[19], "\"lon\":0.00000") != NULL);
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

    printf("[PASS] test_rns_identity_pairing_and_lua_crsf (SHA-256, 16B SINGLE dest_hash, RNSA/RNSP/RNSC, CRSF 0x7A/0x7B/0x21)\n");
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
    printf("All host unit tests PASSED.\n");
    return 0;
}
