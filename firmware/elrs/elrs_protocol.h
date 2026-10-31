/* SPDX-License-Identifier: AGPL-3.0-or-later */
/*
 * Portable ExpressLRS v3/v4 Over-The-Air (OTA) protocol, UID MD5 derivation,
 * GenerableCRC (14-bit & 16-bit), and FHSS sequence generator for LR2021.
 *
 * Adapted from ExpressLRS (GPL-3.0-or-later, src/lib/OTA, src/lib/FHSS, src/lib/CRC).
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ELRS_UID_LEN            6U
#define ELRS_OTA_VERSION_ID     4U
#define ELRS_OTA4_PACKET_SIZE   8U
#define ELRS_OTA8_PACKET_SIZE   13U
#define ELRS_FHSS_SEQUENCE_LEN  256U
#define ELRS_NUM_CHANNELS       16U

/* CRSF standard channel values (11-bit: 172..1811, center 992) */
#define ELRS_CRSF_CH_MIN        172U
#define ELRS_CRSF_CH_MID        992U
#define ELRS_CRSF_CH_MAX        1811U

/* OTA Packet Header Types (low 2 bits of byte 0) */
#define ELRS_PKT_TYPE_RCDATA    0x00U /* Uplink RC channels */
#define ELRS_PKT_TYPE_LINKSTATS 0x00U /* Downlink LinkStatistics */
#define ELRS_PKT_TYPE_DATA      0x01U /* Uplink/Downlink MSP/MAVLink/Telemetry data */
#define ELRS_PKT_TYPE_SYNC      0x02U /* Uplink SYNC packet */

/* Switch encoding modes */
typedef enum {
    ELRS_SW_WIDE_OR_8CH    = 0,
    ELRS_SW_HYBRID_OR_16CH = 1,
    ELRS_SW_12CH           = 2,
} elrs_switch_mode_t;

/* Regulatory domains */
typedef enum {
    ELRS_DOMAIN_AU915  = 0,
    ELRS_DOMAIN_FCC915 = 1,
    ELRS_DOMAIN_EU868  = 2,
    ELRS_DOMAIN_IN866  = 3,
    ELRS_DOMAIN_AU433  = 4,
    ELRS_DOMAIN_EU433  = 5,
    ELRS_DOMAIN_US433  = 6,
    ELRS_DOMAIN_US433W = 7,
    ELRS_DOMAIN_TH920  = 8,
    ELRS_DOMAIN_ISM2G4 = 9,
    ELRS_DOMAIN_CE2G4  = 10,
    ELRS_DOMAIN_COUNT
} elrs_domain_t;

/* ExpressLRS RF Rate Enum (matches upstream expresslrs_RFrates_e wire values in SYNC packets) */
typedef enum {
    ELRS_RATE_LORA_900_25HZ       = 0,
    ELRS_RATE_LORA_900_50HZ       = 1,
    ELRS_RATE_LORA_900_100HZ      = 2,
    ELRS_RATE_LORA_900_100HZ_8CH  = 3,
    ELRS_RATE_LORA_900_150HZ      = 4,
    ELRS_RATE_LORA_900_200HZ      = 5,
    ELRS_RATE_LORA_900_200HZ_8CH  = 6,
    ELRS_RATE_LORA_900_250HZ      = 7,
    ELRS_RATE_LORA_900_333HZ_8CH  = 8,
    ELRS_RATE_LORA_900_500HZ      = 9,
    ELRS_RATE_LORA_900_50HZ_DVDA  = 10,
    ELRS_RATE_FSK_900_1000HZ_8CH  = 11,

    ELRS_RATE_LORA_2G4_25HZ       = 20,
    ELRS_RATE_LORA_2G4_50HZ       = 21,
    ELRS_RATE_LORA_2G4_100HZ      = 22,
    ELRS_RATE_LORA_2G4_100HZ_8CH  = 23,
    ELRS_RATE_LORA_2G4_150HZ      = 24,
    ELRS_RATE_LORA_2G4_200HZ      = 25,
    ELRS_RATE_LORA_2G4_200HZ_8CH  = 26,
    ELRS_RATE_LORA_2G4_250HZ      = 27,
    ELRS_RATE_LORA_2G4_333HZ_8CH  = 28,
    ELRS_RATE_LORA_2G4_500HZ      = 29,
    ELRS_RATE_FLRC_2G4_250HZ_DVDA = 30,
    ELRS_RATE_FLRC_2G4_500HZ_DVDA = 31,
    ELRS_RATE_FLRC_2G4_500HZ      = 32,
    ELRS_RATE_FLRC_2G4_1000HZ     = 33,
    ELRS_RATE_FSK_2G4_250HZ_DVDA  = 34,
    ELRS_RATE_FSK_2G4_500HZ_DVDA  = 35,
    ELRS_RATE_FSK_2G4_1000HZ      = 36,

    ELRS_RATE_LORA_DUAL_100HZ_8CH = 100,
    ELRS_RATE_LORA_DUAL_150HZ     = 101,
} elrs_rf_rate_enum_t;

typedef enum {
    ELRS_PHY_LORA_900 = 0,
    ELRS_PHY_FSK_900  = 1,
    ELRS_PHY_LORA_2G4 = 2,
    ELRS_PHY_FLRC_2G4 = 3,
    ELRS_PHY_FSK_2G4  = 4,
} elrs_phy_type_t;

typedef struct {
    const char         *name;
    elrs_rf_rate_enum_t rate_enum;
    elrs_phy_type_t     phy_type;
    uint8_t             bw;
    uint8_t             sf;
    uint8_t             cr;
    uint16_t            preamble_len;
    uint32_t            interval_us;
    uint8_t             fhss_hop_interval;
    uint8_t             tlm_denom_default;
    uint8_t             payload_len; /* 8 (OTA4) or 13 (OTA8) */
    uint8_t             num_of_sends;
} elrs_air_rate_config_t;

typedef struct {
    const char *domain_name;
    uint32_t    freq_start_hz;
    uint32_t    freq_stop_hz;
    uint32_t    freq_count;
    uint32_t    freq_center_hz;
} elrs_fhss_domain_info_t;

typedef struct {
    elrs_domain_t                  domain;
    const elrs_fhss_domain_info_t *info;
    uint32_t                       freq_spread_hz;
    uint8_t                        sync_channel;
    uint16_t                       sequence_count;
    uint16_t                       hop_ptr;
    uint8_t                        sequence[ELRS_FHSS_SEQUENCE_LEN];
} elrs_fhss_state_t;

typedef struct {
    uint8_t fhss_index;
    uint8_t nonce;
    uint8_t rf_rate_enum;
    uint8_t switch_enc_mode;
    uint8_t new_tlm_ratio;
    uint8_t gemini_mode;
    uint8_t ota_protocol;
    uint8_t uid4;
    uint8_t uid5;
} elrs_sync_data_t;

typedef struct {
    uint8_t uplink_rssi_1; /* Positive dBm magnitude (0..127 -> -0..-127 dBm) */
    uint8_t uplink_rssi_2;
    uint8_t antenna;
    uint8_t model_match;
    uint8_t lq;
    uint8_t true_diversity;
    int8_t  snr_scaled;    /* SNR * 4 */
} elrs_link_stats_data_t;

typedef struct {
    uint8_t            uid[ELRS_UID_LEN];
    uint16_t           crc_init;
    uint32_t           uid_seed;
    bool               is_full_res; /* false = OTA4 (8 bytes, 14-bit CRC), true = OTA8 (13 bytes, 16-bit CRC) */
    elrs_switch_mode_t switch_mode;
    uint8_t            ota_nonce;
    uint8_t            hybrid8_next_sw_idx;
    bool               fullres_is_high_aux;
    uint8_t            uplink_tx_power_enum;
    uint16_t           crc_table[256];
    uint8_t            crc_bits;
    uint16_t           crc_poly;
    uint16_t           crc_mask;
} elrs_ota_state_t;

/* 1. Compute 6-byte ExpressLRS UID from binding phrase using MD5("-DMY_BINDING_PHRASE=\"<phrase>\"") */
void elrs_uid_from_bind_phrase(const char *phrase, uint8_t out_uid[ELRS_UID_LEN]);

/* Parse 6-byte UID from hex/comma string (e.g. "12:34:56:78:9A:BC" or "123456789abc" or "18,52,86,120,154,188") */
bool elrs_uid_parse_string(const char *str, uint8_t out_uid[ELRS_UID_LEN]);

/* 2. Initialize OTA state for a given 6-byte UID, packet size (8 or 13), and switch mode */
void elrs_ota_init(elrs_ota_state_t *st,
                   const uint8_t uid[ELRS_UID_LEN],
                   uint8_t packet_size,
                   elrs_switch_mode_t switch_mode);

/* Update packet size / switch mode while keeping UID */
void elrs_ota_set_mode(elrs_ota_state_t *st, uint8_t packet_size, elrs_switch_mode_t switch_mode);

/* Generate and validate OTA packet CRC (14-bit for OTA4, 16-bit for OTA8) */
void elrs_ota_generate_crc(const elrs_ota_state_t *st, uint8_t *pkt, uint8_t ota_nonce);
bool elrs_ota_validate_crc(const elrs_ota_state_t *st, const uint8_t *pkt, uint8_t ota_nonce);

/* Pack & unpack SYNC packet */
void elrs_ota_pack_sync(const elrs_ota_state_t *st,
                        const elrs_sync_data_t *sync,
                        uint8_t *out_pkt);
bool elrs_ota_unpack_sync(const elrs_ota_state_t *st,
                          const uint8_t *pkt,
                          elrs_sync_data_t *out_sync);

/* Pack & unpack RC Channels packet (ch_crsf[16] in 11-bit CRSF 172..1811 range) */
void elrs_ota_pack_rc(elrs_ota_state_t *st,
                      const uint16_t ch_crsf[ELRS_NUM_CHANNELS],
                      bool is_armed,
                      bool stubborn_ack,
                      uint8_t *out_pkt);
bool elrs_ota_unpack_rc(const elrs_ota_state_t *st,
                        const uint8_t *pkt,
                        uint16_t ch_crsf[ELRS_NUM_CHANNELS],
                        bool *out_is_armed,
                        bool *out_stubborn_ack);

/* Pack & unpack Downlink LinkStats telemetry packet */
void elrs_ota_pack_linkstats(const elrs_ota_state_t *st,
                             const elrs_link_stats_data_t *stats,
                             uint8_t pkg_index,
                             bool stubborn_ack,
                             uint8_t *out_pkt);
bool elrs_ota_unpack_linkstats(const elrs_ota_state_t *st,
                               const uint8_t *pkt,
                               elrs_link_stats_data_t *out_stats,
                               uint8_t *out_pkg_index,
                               bool *out_stubborn_ack);

/* 3. FHSS Sequence Generation & Hopping */
const elrs_fhss_domain_info_t *elrs_fhss_get_domain_info(elrs_domain_t domain);
void elrs_fhss_init(elrs_fhss_state_t *fhss, elrs_domain_t domain, uint32_t uid_seed);
uint32_t elrs_fhss_get_initial_freq(const elrs_fhss_state_t *fhss);
uint32_t elrs_fhss_get_curr_freq(const elrs_fhss_state_t *fhss);
uint32_t elrs_fhss_get_gemini_freq(const elrs_fhss_state_t *fhss);
uint32_t elrs_fhss_advance_and_get_freq(elrs_fhss_state_t *fhss);
void elrs_fhss_set_index(elrs_fhss_state_t *fhss, uint8_t idx);
bool elrs_fhss_on_sync_channel(const elrs_fhss_state_t *fhss);

/* 4. Air Rate Table Lookup (22 supported LR2021 Sub-GHz + 2.4 GHz rates) */
uint8_t elrs_get_air_rate_count(void);
const elrs_air_rate_config_t *elrs_get_air_rate_by_index(uint8_t idx);
const elrs_air_rate_config_t *elrs_get_air_rate_by_enum(uint8_t rate_enum);

#ifdef __cplusplus
}
#endif
