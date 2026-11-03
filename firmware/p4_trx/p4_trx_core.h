/* SPDX-License-Identifier: AGPL-3.0-or-later */
/*
 * LilyGo T-Display P4 (ESP32-P4NRW32 + Semtech LR2021 + MIPI-DSI Touch)
 * Standalone Drone TRX Unit Core:
 *   - Multi-touch dual virtual gimbals (Mode 1..4, trims, expo, IMU tilt-to-steer)
 *   - ExpressLRS 3.x / 4.0 TX & RX state machine + Reticulum Swarm Command Center
 *   - L76K GNSS NMEA parser ($GNGGA / $GNRMC) for Follow-Me Formation Leader
 *   - T-Display P4 LR2021 front-end config (32 MHz TCXO 3.0V, AT2401C 2.4 GHz FEM,
 *     SKY13453 INT/MMCX1 antenna switch, +5 dBm HF PA cap / +22 dBm LF PA)
 *   - Landscape RGB565 UI renderer (1168x540 HI8561 / 1232x568 RM69A10)
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

/* ---------------------------------------------------------------------------
 * 1. T-Display P4 Hardware Constants (XL9535, LR2021, Display, Peripherals)
 * --------------------------------------------------------------------------- */
#define P4_I2C1_SDA_PIN                 7U
#define P4_I2C1_SCL_PIN                 8U
#define P4_I2C1_INT_PIN                 5U

#define P4_I2C2_SDA_PIN                 20U
#define P4_I2C2_SCL_PIN                 21U

#define P4_XL9535_I2C_ADDR              0x20U
#define P4_SGM38121_I2C_ADDR            0x28U
#define P4_BQ27220_I2C_ADDR             0x55U
#define P4_PCF8563_I2C_ADDR             0x51U
#define P4_ES8311_I2C_ADDR              0x18U
#define P4_AW86224_I2C_ADDR             0x58U
#define P4_ICM20948_I2C_ADDR            0x68U
#define P4_QMI8658_I2C_ADDR             0x6AU
#define P4_QMC6309_I2C_ADDR             0x7CU
#define P4_GT9895_I2C_ADDR              0x5DU
#define P4_HI8561_TOUCH_I2C_ADDR        0x68U

/* XL9535 16-bit I/O expander pin definitions */
#define P4_XL_PIN_3V3_POWER_EN          0U   /* Active-LOW: MUST remain 0 at all times! */
#define P4_XL_PIN_SKY13453_VCTL         1U   /* 1 = Internal antenna, 0 = External MMCX1 */
#define P4_XL_PIN_SCREEN_RST            2U   /* Active-LOW display reset */
#define P4_XL_PIN_TOUCH_RST             3U   /* Active-LOW touch reset */
#define P4_XL_PIN_TOUCH_INT             4U   /* Input */
#define P4_XL_PIN_ETH_RST               5U   /* Output */
#define P4_XL_PIN_5V0_AUDIO_PWR_EN      6U   /* 1 = Enable 5V0 & RT9080-33 audio rail */
#define P4_XL_PIN_IMU_INT               7U   /* Input */
#define P4_XL_PIN_USB_PHY_PWR_EN        10U  /* 1 = Enable USB PHY / VCCA rail */
#define P4_XL_PIN_GPS_WAKE_UP           11U  /* 1 = Wake L76K GNSS */
#define P4_XL_PIN_RTC_INT               12U  /* Input */
#define P4_XL_PIN_C6_WAKE_UP            13U  /* Input */
#define P4_XL_PIN_C6_EN                 14U  /* 0 = Disable unused ESP32-C6 co-processor */
#define P4_XL_PIN_SD_POWER_EN           15U  /* Active-LOW SD power */
#define P4_XL_PIN_LORA_RST              16U  /* Active-LOW LR2021 NRESET (Port 1 bit 6 -> bit 14 of 16-bit word) */
#define P4_XL_PIN_LORA_DIO1             17U  /* Input: LR2021 DIO11 IRQ (Port 1 bit 7 -> bit 15 of 16-bit word) */

/* Note on octal XL9535 pin numbering from schematic (IO0_0..IO0_7 = 0..7, IO1_0..IO1_7 = 8..15): */
#define P4_XL_BIT_3V3_POWER_EN          (1U << 0U)
#define P4_XL_BIT_SKY13453_VCTL         (1U << 1U)
#define P4_XL_BIT_SCREEN_RST            (1U << 2U)
#define P4_XL_BIT_TOUCH_RST             (1U << 3U)
#define P4_XL_BIT_TOUCH_INT             (1U << 4U)
#define P4_XL_BIT_ETH_RST               (1U << 5U)
#define P4_XL_BIT_5V0_AUDIO_PWR_EN      (1U << 6U)
#define P4_XL_BIT_IMU_INT               (1U << 7U)
#define P4_XL_BIT_USB_PHY_PWR_EN        (1U << 8U)   /* IO1_0 (octal 10) */
#define P4_XL_BIT_GPS_WAKE_UP           (1U << 9U)   /* IO1_1 (octal 11) */
#define P4_XL_BIT_RTC_INT               (1U << 10U)  /* IO1_2 (octal 12) */
#define P4_XL_BIT_C6_WAKE_UP            (1U << 11U)  /* IO1_3 (octal 13) */
#define P4_XL_BIT_C6_EN                 (1U << 12U)  /* IO1_4 (octal 14) */
#define P4_XL_BIT_SD_POWER_EN           (1U << 13U)  /* IO1_5 (octal 15) */
#define P4_XL_BIT_LORA_RST              (1U << 14U)  /* IO1_6 (octal 16) */
#define P4_XL_BIT_LORA_DIO1             (1U << 15U)  /* IO1_7 (octal 17) */

/* Onboard Semtech LR2021 SPI2 pins on T-Display P4 */
#define P4_LR2021_SCLK_PIN              2U
#define P4_LR2021_MOSI_PIN              3U
#define P4_LR2021_MISO_PIN              4U
#define P4_LR2021_CS_PIN                24U
#define P4_LR2021_BUSY_PIN              6U

/* L76K GNSS UART pins on T-Display P4 */
#define P4_GPS_UART_TX_PIN              22U
#define P4_GPS_UART_RX_PIN              23U

/* ES8311 I2S0 pins on T-Display P4 */
#define P4_I2S_MCLK_PIN                 13U
#define P4_I2S_BCLK_PIN                 12U
#define P4_I2S_WS_PIN                   9U
#define P4_I2S_DOUT_PIN                 10U
#define P4_I2S_DIN_PIN                  11U

/* HI8561 Backlight PWM & BOOT button */
#define P4_BL_PWM_PIN                   51U
#define P4_BOOT_BTN_PIN                 35U

/* T-Display P4 LR2021 RF Front-End limits (32MHz TCXO + AT2401C 2.4GHz FEM) */
#define P4_LR2021_TCXO_TUNE_3V0         0x06U
#define P4_LR2021_TCXO_TIMEOUT_TICKS    32768UL
#define P4_LR2021_MAX_HF_DBM            5     /* Must not exceed +5 dBm into external AT2401C PA */
#define P4_LR2021_MAX_LF_DBM            22    /* Sub-GHz LF PA supports up to +22 dBm */

/* LR2021 DIO function opcodes for T-Display P4 AT2401C + Sub-GHz RF switch */
#define P4_LR2021_OC_SET_TCXO_MODE      0x0120U
#define P4_LR2021_OC_SET_DIO_FUNC       0x0112U
#define P4_LR2021_OC_SET_DIO_RF_SW_CFG  0x0113U
#define P4_LR2021_DIO_FUNC_IRQ          0x01U
#define P4_LR2021_DIO_FUNC_RF_SWITCH    0x03U

typedef enum {
    P4_ANTENNA_INTERNAL = 0, /* SKY13453_VCTL = 1 */
    P4_ANTENNA_MMCX1    = 1  /* SKY13453_VCTL = 0 */
} p4_antenna_path_t;

typedef enum {
    P4_PANEL_HI8561_TFT     = 0, /* 1168x540 landscape (540x1168 native portrait) */
    P4_PANEL_RM69A10_AMOLED = 1  /* 1232x568 landscape (568x1232 native portrait) */
} p4_panel_variant_t;

typedef enum {
    P4_GIMBAL_MODE_1 = 1, /* Left: Pitch/Yaw,      Right: Throttle/Roll */
    P4_GIMBAL_MODE_2 = 2, /* Left: Throttle/Yaw,   Right: Pitch/Roll (Default) */
    P4_GIMBAL_MODE_3 = 3, /* Left: Pitch/Roll,     Right: Throttle/Yaw */
    P4_GIMBAL_MODE_4 = 4  /* Left: Throttle/Roll,  Right: Pitch/Yaw */
} p4_gimbal_mode_t;

typedef enum {
    P4_PAGE_HUD      = 0, /* Flight HUD, Artificial Horizon, 16ch CRSF, MAVLink Telemetry */
    P4_PAGE_ELRS     = 1, /* ExpressLRS RF Config: Rate, Domain, Power, Antenna, Mode */
    P4_PAGE_SWARM    = 2, /* Reticulum & Swarm Control: Pair, Peers, Formation, Follow-Me, RNSM */
    P4_PAGE_SPECTRUM = 3, /* RF Spectrum Waterfall/Bars, IMU, L76K GNSS, BQ27220 Battery, Audio/Haptics */
    P4_PAGE_COUNT    = 4
} p4_ui_page_t;

typedef enum {
    P4_FEEDBACK_NONE     = 0,
    P4_FEEDBACK_CLICK    = 1, /* Short crisp haptic click + 1.8 kHz blip */
    P4_FEEDBACK_DETENT   = 2, /* Center stick crossing tick */
    P4_FEEDBACK_ARM      = 3, /* Rising two-tone arming chime + strong pulse */
    P4_FEEDBACK_DISARM   = 4, /* Falling disarm tone */
    P4_FEEDBACK_ALERT    = 5, /* Link loss / low battery alarm */
    P4_FEEDBACK_SWARM_TX = 6  /* Swarm formation / multi-drone command transmitted */
} p4_feedback_event_t;

/* ---------------------------------------------------------------------------
 * 2. Multi-Touch, Gimbal, IMU, GNSS & Battery Telemetry Structures
 * --------------------------------------------------------------------------- */
#define P4_MAX_TOUCH_POINTS             10U
#define P4_SPECTRUM_BINS                32U

typedef struct {
    bool     valid;
    uint8_t  id;
    uint16_t x;        /* Landscape X: 0 .. width-1 */
    uint16_t y;        /* Landscape Y: 0 .. height-1 */
    uint8_t  pressure;
} p4_touch_point_t;

typedef struct {
    uint8_t          count;
    p4_touch_point_t points[P4_MAX_TOUCH_POINTS];
} p4_touch_frame_t;

typedef struct {
    bool    imu_present;
    bool    is_icm20948;      /* true = ICM20948 (0x68), false = QMI8658 (0x6A) */
    float   roll_deg;         /* -90 .. +90 deg */
    float   pitch_deg;        /* -90 .. +90 deg */
    float   yaw_deg;          /* 0 .. 360 deg */
    float   accel_g[3];
    float   gyro_dps[3];
} p4_imu_state_t;

typedef struct {
    bool     fix_valid;
    uint8_t  fix_quality;     /* 0=Invalid, 1=GPS, 2=DGPS */
    uint8_t  satellites;
    int32_t  lat_deg_e7;      /* Latitude * 1e7 */
    int32_t  lon_deg_e7;      /* Longitude * 1e7 */
    int32_t  alt_mm;          /* Altitude MSL in mm */
    uint16_t groundspeed_cms; /* Groundspeed in cm/s */
    uint16_t heading_cdeg;    /* Course over ground in centi-degrees (0..35999) */
    uint32_t sentences_ok;
    char     line_buf[128];
    uint8_t  line_pos;
} p4_gnss_state_t;

typedef struct {
    bool     gauge_present;
    uint16_t voltage_mv;      /* e.g. 4120 mV */
    int16_t  current_ma;      /* signed mA (+discharging / -charging) */
    uint8_t  soc_pct;         /* 0..100 % */
    int16_t  temp_deci_c;     /* 0.1 deg C */
} p4_battery_state_t;

typedef struct {
    uint8_t  pa_sel;          /* 0 = LF, 1 = HF */
    uint8_t  pa_lf_mode;
    uint8_t  pa_lf_duty_cycle;
    uint8_t  pa_lf_slices;
    uint8_t  pa_hf_duty_cycle;
    int8_t   power_half_dbm;  /* 2 * dBm */
    uint8_t  dio_rf_sw_map[5];/* DIO6, DIO7, DIO8, DIO10, DIO11 */
    uint8_t  tcxo_tune;       /* 0x06 = 3.0V */
    uint32_t tcxo_timeout;
} p4_lr2021_rf_setup_t;

/* ---------------------------------------------------------------------------
 * 3. Main T-Display P4 TRX State Machine Structure
 * --------------------------------------------------------------------------- */
typedef struct {
    /* Display geometry (Landscape) */
    p4_panel_variant_t panel_variant;
    uint16_t           screen_w;       /* 1168 (HI8561) or 1232 (RM69A10) */
    uint16_t           screen_h;       /* 540  (HI8561) or 568  (RM69A10) */
    uint8_t            brightness_pct; /* 10..100 % */

    /* UI Navigation & Feedback */
    p4_ui_page_t       active_page;
    p4_feedback_event_t last_feedback;
    uint32_t           feedback_count;
    bool               haptics_enabled;
    bool               audio_enabled;
    bool               dfu_requested;

    /* Virtual Gimbals & Controls */
    p4_gimbal_mode_t   gimbal_mode;
    bool               throttle_spring_center; /* false = sticky throttle (default), true = 3D/rover spring center */
    uint8_t            rate_pct;               /* 50, 75, or 100 % */
    uint8_t            expo_pct;               /* 0, 20, 40 % */

    /* Normalized gimbal positions (-1.0f .. +1.0f) */
    float              left_x;
    float              left_y;                 /* Throttle in Mode 2 (-1.0 = bottom/idle, +1.0 = full) */
    float              right_x;
    float              right_y;

    /* Active touch finger tracking (-1 if not held) */
    int8_t             left_touch_id;
    int8_t             right_touch_id;
    bool               ui_tap_latched;

    /* Sub-trims in CRSF ticks (-100 .. +100) */
    int16_t            trim_roll;
    int16_t            trim_pitch;
    int16_t            trim_thr;
    int16_t            trim_yaw;

    /* Switches & Aux States */
    bool               sw_armed;               /* Ch5: ARM (1792) / DISARM (191) */
    uint8_t            sw_flight_mode;         /* Ch6: 0=Stabilize(191), 1=Loiter(992), 2=Guided/Auto(1792) */
    bool               sw_loiter;              /* Ch7 */
    bool               sw_rtl;                 /* Ch8 */
    uint8_t            sw_formation;           /* Ch9: 0=HOLD, 1=LINE, 2=VEE, 3=COLUMN */
    bool               sw_follow_me;           /* Ch10: Broadcast L76K GPS as RNSF Leader */
    bool               sw_imu_tilt_steer;      /* Ch11: Blend IMU Roll/Pitch into Aileron/Elevator */
    bool               sw_beeper;              /* Ch12 */

    /* Output 16-channel CRSF array (172 .. 1811, center 992) */
    uint16_t           crsf_channels[CRSF_NUM_CHANNELS];

    /* ExpressLRS RF Configuration & Link State */
    uint8_t            app_mode;               /* 0=IDLE, 1=ELRS_RX, 2=ELRS_TX, 3=RNS_RELAY, 4=RNODE_KISS */
    uint8_t            elrs_rate_idx;          /* 0..22 */
    elrs_domain_t      elrs_domain;
    int8_t             tx_power_dbm;
    p4_antenna_path_t  antenna_path;           /* P4_ANTENNA_INTERNAL or P4_ANTENNA_MMCX1 */
    char               bind_phrase[32];
    uint8_t            uid[6];
    elrs_fhss_state_t  fhss;
    elrs_ota_state_t   ota;
    elrs_data_tunnel_t tunnel;

    bool               elrs_locked;
    uint8_t            uplink_lq;
    int8_t             uplink_rssi_dbm;
    int8_t             uplink_snr_db;
    uint32_t           tx_pkt_count;
    uint32_t           rx_pkt_count;
    uint32_t           sync_pkt_count;

    /* Reticulum & Swarm Command Center State */
    rns_node_identity_t rns_id;
    rns_peer_table_t    rns_peers;
    rns_relay_table_t   rns_relay;
    rns_fc_telemetry_t  drone_tlm;
    uint32_t            rns_freq_hz;
    uint32_t            rns_bw_hz;
    uint8_t             rns_sf;
    uint8_t             rns_cr;
    int8_t              rns_txpower_dbm;
    uint16_t            formation_spacing_dm;  /* e.g. 100 = 10.0 m */
    uint8_t             formation_seq;
    uint8_t             multi_seq;
    uint8_t             targeted_seq;
    uint32_t            last_follow_me_ms;
    char                last_swarm_action[40];

    /* Last generated OTA Reticulum / Swarm packet (for TX queuing & test verification) */
    uint8_t             pending_rns_pkt[128];
    size_t              pending_rns_len;

    /* Onboard Sensors & Spectrum */
    p4_imu_state_t      imu;
    p4_gnss_state_t     gnss;
    p4_battery_state_t  battery;
    int8_t              spectrum_dbm[P4_SPECTRUM_BINS];
    uint32_t            spectrum_center_hz;
    uint32_t            spectrum_span_hz;
} p4_trx_state_t;

/* ---------------------------------------------------------------------------
 * 4. Public API
 * --------------------------------------------------------------------------- */

/* Initialize T-Display P4 TRX state with panel variant and binding phrase */
void p4_trx_init(p4_trx_state_t *st,
                 p4_panel_variant_t panel,
                 const char *bind_phrase,
                 const uint8_t hw_id[8]);

/* Compute exact XL9535 16-bit output register word (ensuring IO0 3.3V_POWER_EN=0 and IO10 USB_PHY=1) */
uint16_t p4_trx_build_xl9535_output_word(const p4_trx_state_t *st,
                                         bool screen_rst_high,
                                         bool touch_rst_high,
                                         bool lora_rst_high);

/* Compute T-Display P4 LR2021 RF setup (3.0V TCXO, AT2401C DIO6/7/8/10/11 matrix, +5 dBm HF cap) */
void p4_trx_compute_lr2021_rf_setup(uint32_t freq_hz,
                                    int8_t requested_dbm,
                                    p4_lr2021_rf_setup_t *out_setup);

/* Map raw touch controller coordinates to Landscape (screen_w x screen_h) */
void p4_trx_map_raw_touch_to_landscape(p4_panel_variant_t panel,
                                       uint16_t raw_x,
                                       uint16_t raw_y,
                                       uint16_t *out_lx,
                                       uint16_t *out_ly);

/* Process a multi-touch frame (simultaneous left/right thumb gimbals + UI buttons) */
void p4_trx_process_touch(p4_trx_state_t *st,
                          const p4_touch_frame_t *touch,
                          uint32_t now_ms);

/* Recompute all 16 CRSF channels from gimbals, Mode 1..4, trims, expo, IMU tilt, and switches */
void p4_trx_update_crsf_channels(p4_trx_state_t *st);

/* Step one ExpressLRS TX frame tick (packs OTA4/OTA8 RC, SYNC, or DATA tunnel frame) */
size_t p4_trx_step_elrs_tx_frame(p4_trx_state_t *st,
                                 uint32_t *out_rf_freq_hz,
                                 uint8_t out_ota_pkt[16]);

/* Feed a byte from the L76K GNSS UART ($GNGGA / $GNRMC); returns true when a sentence completes */
bool p4_trx_gnss_feed_byte(p4_gnss_state_t *gnss, char c);

/* Trigger a Follow-Me RNSF formation broadcast using the T-Display P4's onboard L76K GNSS fix */
size_t p4_trx_build_follow_me_formation_pkt(p4_trx_state_t *st,
                                            uint8_t out_pkt[64]);

/* Trigger a 4-Drone RNSM swarm command bundle from the Swarm UI / CLI */
size_t p4_trx_build_default_swarm_bundle_pkt(p4_trx_state_t *st,
                                             uint8_t out_pkt[64]);

/* Render the full Landscape TX16S-style UI into an RGB565 framebuffer (screen_w * screen_h) */
void p4_trx_render_landscape_rgb565(const p4_trx_state_t *st,
                                    uint16_t *fb_rgb565);

/* Rotate a Landscape RGB565 buffer (W x H) into the MIPI-DSI native Portrait buffer (H x W) */
void p4_trx_blit_landscape_to_portrait_rgb565(const uint16_t *landscape_fb,
                                              uint16_t *portrait_fb,
                                              uint16_t landscape_w,
                                              uint16_t landscape_h);

#ifdef __cplusplus
}
#endif
