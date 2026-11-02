/* SPDX-License-Identifier: AGPL-3.0-or-later */
/*
 * Dual Semtech LR1121 driver and simultaneous ELRS + Reticulum LoRa mesh
 * coordinator for RadioMaster XR4 (ESP32-PICO-D4 + 2x LR1121).
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "../elrs/elrs_protocol.h"
#include "../crsf/crsf.h"
#include "../rnode/rns_node.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * RadioMaster XR4 (ESP32-PICO-D4 + 2x Semtech LR1121) Hardware Pinout
 * Matches ExpressLRS Generic LR1121 True Diversity.json + radiomaster.rx_dual.xr4
 * ============================================================================ */
#define XR4_PIN_SPI_SCK         25
#define XR4_PIN_SPI_MISO        33
#define XR4_PIN_SPI_MOSI        32

#define XR4_PIN_RADIO1_NSS      27
#define XR4_PIN_RADIO1_RST      26
#define XR4_PIN_RADIO1_BUSY     36
#define XR4_PIN_RADIO1_DIO1     37

#define XR4_PIN_RADIO2_NSS      13
#define XR4_PIN_RADIO2_RST      21
#define XR4_PIN_RADIO2_BUSY     39
#define XR4_PIN_RADIO2_DIO1     34

#define XR4_PIN_UART0_RX        3   /* Primary CRSF UART to FC SERIAL6 */
#define XR4_PIN_UART0_TX        1
#define XR4_PIN_UART1_RX        5   /* Auxiliary X1/X2 UART pads (MAVLink / KISS / CLI) */
#define XR4_PIN_UART1_TX        18

#define XR4_PIN_LED_RGB         22
#define XR4_PIN_BUTTON          0

/* Official RadioMaster XR4 DIO5..DIO8 RF Switch Matrix (SetDioAsRfSwitch 0x0112) */
extern const uint8_t k_xr4_rfsw_ctrl[8];

/* ============================================================================
 * Semtech LR1121 SPI Opcodes & Constants
 * ============================================================================ */
#define LR1121_OC_GET_STATUS                0x0100U
#define LR1121_OC_GET_VERSION               0x0101U
#define LR1121_OC_WRITE_REGMEM32            0x0105U
#define LR1121_OC_READ_REGMEM32             0x0106U
#define LR1121_OC_WRITE_BUFFER8             0x0109U
#define LR1121_OC_READ_BUFFER8              0x010AU
#define LR1121_OC_CLEAR_RXBUFFER            0x010BU
#define LR1121_OC_WRITE_REGMEM32_MASK       0x010CU
#define LR1121_OC_GET_ERRORS                0x010DU
#define LR1121_OC_CLEAR_ERRORS              0x010EU
#define LR1121_OC_CALIBRATE                 0x010FU
#define LR1121_OC_SET_REGMODE               0x0110U
#define LR1121_OC_CALIBRATE_IMAGE           0x0111U
#define LR1121_OC_SET_DIO_AS_RF_SWITCH      0x0112U
#define LR1121_OC_SET_DIO_IRQ_PARAMS        0x0113U
#define LR1121_OC_CLEAR_IRQ                 0x0114U
#define LR1121_OC_SET_TCXO_MODE             0x0117U
#define LR1121_OC_REBOOT                    0x0118U
#define LR1121_OC_SET_SLEEP                 0x011BU
#define LR1121_OC_SET_STANDBY               0x011CU
#define LR1121_OC_SET_FS                    0x011DU

#define LR1121_OC_RESET_STATS               0x0200U
#define LR1121_OC_GET_STATS                 0x0201U
#define LR1121_OC_GET_PKT_TYPE              0x0202U
#define LR1121_OC_GET_RXBUFFER_STATUS       0x0203U
#define LR1121_OC_GET_PKT_STATUS            0x0204U
#define LR1121_OC_GET_RSSI_INST             0x0205U
#define LR1121_OC_SET_GFSK_SYNC_WORD        0x0206U
#define LR1121_OC_SET_LORA_PUBLIC_NETWORK   0x0208U
#define LR1121_OC_SET_RX                    0x0209U
#define LR1121_OC_SET_TX                    0x020AU
#define LR1121_OC_SET_RF_FREQUENCY          0x020BU
#define LR1121_OC_SET_PKT_TYPE              0x020EU
#define LR1121_OC_SET_MODULATION_PARAM      0x020FU
#define LR1121_OC_SET_PKT_PARAM             0x0210U
#define LR1121_OC_SET_TX_PARAMS             0x0211U
#define LR1121_OC_SET_RX_TX_FALLBACK_MODE   0x0213U
#define LR1121_OC_SET_PA_CFG                0x0215U
#define LR1121_OC_SET_RX_BOOSTED            0x0227U
#define LR1121_OC_SET_LORA_SYNC_WORD        0x022BU

/* IRQ Bitmasks */
#define LR1121_IRQ_TX_DONE                  (1UL << 2)
#define LR1121_IRQ_RX_DONE                  (1UL << 3)
#define LR1121_IRQ_CRC_ERROR                (1UL << 6)
#define LR1121_IRQ_TIMEOUT                  (1UL << 9)

/* Packet Types */
#define LR1121_PKT_TYPE_GFSK                0x01U
#define LR1121_PKT_TYPE_LORA                0x02U

/* LoRa Bandwidth Codes */
#define LR1121_LORA_BW_62                   0x03U
#define LR1121_LORA_BW_125                  0x04U
#define LR1121_LORA_BW_250                  0x05U
#define LR1121_LORA_BW_500                  0x06U
#define LR1121_LORA_BW_200_2G4              0x0DU
#define LR1121_LORA_BW_400_2G4              0x0EU
#define LR1121_LORA_BW_800_2G4              0x0FU

/* LoRa Coding Rates */
#define LR1121_LORA_CR_4_5                  0x01U
#define LR1121_LORA_CR_4_6                  0x02U
#define LR1121_LORA_CR_4_7                  0x03U
#define LR1121_LORA_CR_4_8                  0x04U
#define LR1121_LORA_CR_LI_4_5               0x05U
#define LR1121_LORA_CR_LI_4_6               0x06U
#define LR1121_LORA_CR_LI_4_8               0x07U

/* Power Amplifier Selections */
#define LR1121_PA_SEL_LP                    0x00U /* Sub-GHz Low Power PA (-17..+14 dBm) */
#define LR1121_PA_SEL_HP                    0x01U /* Sub-GHz High Power PA (-9..+22 dBm) */
#define LR1121_PA_SEL_HF                    0x02U /* 2.4 GHz High Freq PA (-18..+13 dBm) */

#define LR1121_RAMP_48_US                   0x02U
#define LR1121_GFSK_BWF_600KHZ              0x1AU

typedef enum {
    LR1121_RADIO_1   = 0, /* XR4 Radio 1: Dedicated to 2.4 GHz / Sub-GHz ExpressLRS */
    LR1121_RADIO_2   = 1, /* XR4 Radio 2: Dedicated to 868 MHz Reticulum LoRa Mesh */
    LR1121_RADIO_MAX = 2,
} lr1121_radio_idx_t;

typedef enum {
    LR1121_MODE_SLEEP      = 0,
    LR1121_MODE_STDBY_RC   = 1,
    LR1121_MODE_STDBY_XOSC = 2,
    LR1121_MODE_FS         = 3,
    LR1121_MODE_RX_CONT    = 4,
    LR1121_MODE_TX         = 5,
} lr1121_op_mode_t;

typedef struct {
    uint8_t  hw_version;
    uint8_t  chip_type;     /* 0x03 / 0xF3 for LR1121 */
    uint16_t fw_version;
} lr1121_version_t;

typedef struct {
    int8_t  rssi_dbm;
    int8_t  snr_q2_db;
    uint8_t raw_len;
} lr1121_pkt_stats_t;

/* Hardware Abstraction Callbacks (ESP32 SPI/GPIO or Host Unit Test Mock) */
typedef struct {
    void (*reset_radio)(lr1121_radio_idx_t radio);
    bool (*wait_busy)(lr1121_radio_idx_t radio, uint32_t timeout_us);
    void (*write_cmd)(lr1121_radio_idx_t radio, uint16_t opcode, const uint8_t *params, uint16_t len);
    void (*read_cmd)(lr1121_radio_idx_t radio, uint16_t opcode, const uint8_t *args, uint16_t arg_len, uint8_t *out, uint16_t out_len);
} lr1121_hal_ops_t;

void lr1121_set_hal_ops(const lr1121_hal_ops_t *ops);

/* Core LR1121 Driver API (per radio instance) */
bool lr1121_begin(lr1121_radio_idx_t radio, uint32_t low_freq_hz, uint32_t high_freq_hz, lr1121_version_t *out_ver);
bool lr1121_get_version(lr1121_radio_idx_t radio, lr1121_version_t *out_ver);
void lr1121_set_mode(lr1121_radio_idx_t radio, lr1121_op_mode_t mode);
void lr1121_set_rf_switch(lr1121_radio_idx_t radio, const uint8_t rfsw_ctrl[8]);
void lr1121_set_frequency(lr1121_radio_idx_t radio, uint32_t freq_hz, bool enter_rx);
void lr1121_set_pa_and_power_dbm(lr1121_radio_idx_t radio, int8_t power_dbm, bool is_sub_ghz);
uint8_t lr1121_lora_bw_from_hz(uint32_t bw_hz);

void lr1121_config_lora(lr1121_radio_idx_t radio,
                        uint8_t bw_code,
                        uint8_t sf,
                        uint8_t cr_code,
                        uint32_t freq_hz,
                        uint16_t preamble_len,
                        bool invert_iq,
                        uint8_t payload_len,
                        bool explicit_header,
                        bool crc_en);

void lr1121_config_fsk_k1000(lr1121_radio_idx_t radio,
                             uint32_t freq_hz,
                             uint8_t sync_word1,
                             uint8_t sync_word2,
                             bool use_fec_14b);

void lr1121_tx_nb(lr1121_radio_idx_t radio, const uint8_t *data, uint8_t len);
bool lr1121_rx_read(lr1121_radio_idx_t radio, uint8_t *out_buf, uint8_t max_len, lr1121_pkt_stats_t *out_stats);
uint32_t lr1121_get_and_clear_irq(lr1121_radio_idx_t radio);

/* ============================================================================
 * Hamming(7,4) 8B <-> 14B FEC Codec for 2G4_K1000 / 900_K1000 GFSK
 * ============================================================================ */
void lr1121_fec_encode_8_to_14(const uint8_t in8[8], uint8_t out14[14]);
void lr1121_fec_decode_14_to_8(const uint8_t in14[14], uint8_t out8[8]);

/* ============================================================================
 * RadioMaster XR4 Simultaneous Dual-Radio Coordinator (Radio 1 ELRS + Radio 2 RNS)
 * ============================================================================ */
typedef struct {
    /* Radio 1: Full-time ExpressLRS (e.g. 2G4_K1000 CE2G4 or EU868) */
    elrs_domain_t        elrs_domain;
    uint8_t              elrs_rate_idx;
    uint8_t              uid[ELRS_UID_LEN];
    elrs_fhss_state_t    fhss;
    elrs_ota_state_t     ota;
    uint16_t             rc_channels[ELRS_NUM_CHANNELS];
    bool                 is_armed;
    bool                 elrs_locked;
    uint32_t             elrs_rx_ok_cnt;
    uint32_t             elrs_crc_err_cnt;
    uint32_t             elrs_tlm_tx_cnt;
    uint32_t             elrs_fhss_ticks;

    /* Reticulum-over-ELRS DATA Tunnel on Radio 1 */
    elrs_data_tunnel_t   elrs_tunnel;

    /* Radio 2: Full-time 868 MHz Reticulum LoRa Mesh (RNODE / RELAY / RNSF / RNSM) */
    uint32_t             mesh_freq_hz;
    uint32_t             mesh_bw_hz;
    uint8_t              mesh_sf;
    uint8_t              mesh_cr;
    int8_t               mesh_power_dbm;
    rns_relay_table_t    relay;
    rns_node_identity_t  identity;
    rns_peer_table_t     peers;
    rns_fc_telemetry_t   fc_tlm;
    uint8_t              drone_slot_id; /* 1..4 */
    uint8_t              mav_seq;
    uint32_t             mesh_tx_cnt;
    uint32_t             mesh_rx_cnt;
    uint32_t             mesh_bridge_cnt;

    /* Deferred Slot-TDMA Relay Queue on Radio 2 */
    bool                 pending_relay_valid;
    uint8_t              pending_relay_pkt[255];
    uint8_t              pending_relay_len;
    uint32_t             pending_relay_due_ms;

    /* Latest MAVLink v2 frame emitted to Flight Controller */
    uint8_t              last_mavlink_frame[72];
    uint8_t              last_mavlink_len;
} xr4_dual_radio_t;

/* Initialize XR4 dual-radio state machine and configure Radio 1 (ELRS) + Radio 2 (868 MHz RNS Mesh) */
bool xr4_dual_radio_init(xr4_dual_radio_t *xr4,
                         const char *bind_phrase,
                         elrs_domain_t elrs_domain,
                         uint8_t elrs_rate_idx,
                         uint32_t mesh_freq_hz,
                         uint32_t mesh_bw_hz,
                         uint8_t mesh_sf,
                         uint8_t mesh_cr,
                         int8_t mesh_power_dbm,
                         uint8_t drone_slot_id,
                         const uint8_t hw_id[8]);

/* Advance 1 FHSS tick on Radio 1 (2.4 GHz ELRS stays 100% continuous while Radio 2 transmits/receives 868 MHz) */
void xr4_dual_radio_elrs_tock(xr4_dual_radio_t *xr4);

/* Process an incoming 8-byte OTA4 packet decoded on Radio 1 (ELRS) */
bool xr4_dual_radio_on_elrs_rx(xr4_dual_radio_t *xr4,
                               const uint8_t pkt8[8],
                               int8_t rssi_dbm,
                               int8_t snr_q2,
                               uint32_t now_ms);

/* Process an incoming 868 MHz LoRa mesh packet decoded on Radio 2 */
bool xr4_dual_radio_on_mesh_rx(xr4_dual_radio_t *xr4,
                               const uint8_t *lora_pkt,
                               uint8_t len,
                               int8_t rssi_dbm,
                               int8_t snr_q2,
                               uint32_t now_ms);

/* Service deferred Slot-TDMA relay transmissions on Radio 2 without touching Radio 1 */
bool xr4_dual_radio_service_tdma(xr4_dual_radio_t *xr4, uint32_t now_ms);

#ifdef __cplusplus
}
#endif
