/* SPDX-License-Identifier: AGPL-3.0-or-later */
/* Host unit test suite for ExpressLRS UID, CRC14/CRC16, OTA4/OTA8, FHSS, and CRSF */

#include "../firmware/elrs/elrs_protocol.h"
#include "../firmware/crsf/crsf.h"

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

static void test_winniepico_live_packets(void) {
    /* Verify live OTA packets captured from RadioMaster TX16SMK3 <-> RadioMaster XR4
     * Binding phrase: "WinniePico" -> UID = EB:71:09:FA:46:6A
     * Air rate: 2G4_K1000 (ELRS_RATE_FSK_2G4_1000HZ = 0x24 = 36), CE_LBT domain
     */
    uint8_t uid[6] = {0};
    elrs_uid_from_bind_phrase("WinniePico", uid);
    const uint8_t expected_uid[6] = {0xEB, 0x71, 0x09, 0xFA, 0x46, 0x6A};
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

    printf("[PASS] test_winniepico_live_packets (TX16SMK3 SYNC + RCDATA, XR4 LINKSTATS)\n");
}

int main(void) {
    test_uid_md5();
    test_fhss_invariants();
    test_ota4_and_ota8_roundtrip();
    test_crsf_framing();
    test_winniepico_live_packets();
    printf("All host unit tests PASSED.\n");
    return 0;
}
