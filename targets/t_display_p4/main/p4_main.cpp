/* SPDX-License-Identifier: AGPL-3.0-or-later */
/*
 * LilyGo T-Display P4 (ESP32-P4NRW32 + Semtech LR2021 + MIPI-DSI Touch)
 * Standalone Drone TRX Unit Firmware (ExpressLRS 3.x/4.0 + Reticulum Swarm GCS).
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "esp_efuse.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_commands.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_ldo_regulator.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "soc/usb_serial_jtag_struct.h"

#include "p4_trx_core.h"
#include "lr2021_driver.h"
#include "LR2021PRAM.h"

static const char *TAG = "p4_trx";

/* Global T-Display P4 TRX State & Framebuffers in 32MB PSRAM */
static p4_trx_state_t       g_p4;
static SemaphoreHandle_t    g_p4_mutex     = NULL;
static uint16_t            *g_fb_landscape = NULL;
static uint16_t            *g_fb_portrait  = NULL;
static esp_lcd_panel_handle_t g_dpi_panel  = NULL;
static esp_lcd_panel_io_handle_t g_dbi_io  = NULL;

/* I2C Bus & Device Handles */
static i2c_master_bus_handle_t g_i2c1_bus  = NULL;
static i2c_master_bus_handle_t g_i2c2_bus  = NULL;
static i2c_master_dev_handle_t g_dev_xl9535    = NULL;
static i2c_master_dev_handle_t g_dev_bq27220   = NULL;
static i2c_master_dev_handle_t g_dev_touch     = NULL;
static i2c_master_dev_handle_t g_dev_sgm38121  = NULL;
static i2c_master_dev_handle_t g_dev_imu       = NULL;
static i2c_master_dev_handle_t g_dev_aw86224   = NULL;

static uint32_t g_hi8561_touch_info_addr = 0x200111A8UL;
static bool     g_touch_ready            = false;
static uint16_t g_lr2021_fw_ver          = 0U;
static bool     g_lr2021_ready           = false;
static bool     g_rf_reconfig_needed     = true;

/* SPI2 Handle for Onboard Semtech LR2021 */
static spi_device_handle_t g_lr2021_spi    = NULL;

/* ---------------------------------------------------------------------------
 * 1. I2C Helper & XL9535 I/O Expander Driver
 * --------------------------------------------------------------------------- */
static bool i2c_write_bytes(i2c_master_dev_handle_t dev, const uint8_t *data, size_t len) {
    if (!dev) return false;
    return i2c_master_transmit(dev, data, len, 50) == ESP_OK;
}

static bool i2c_write_read(i2c_master_dev_handle_t dev,
                           const uint8_t *wbuf, size_t wlen,
                           uint8_t *rbuf, size_t rlen) {
    if (!dev) return false;
    return i2c_master_transmit_receive(dev, wbuf, wlen, rbuf, rlen, 50) == ESP_OK;
}

static bool xl9535_write_reg16(uint8_t reg, uint16_t val) {
    const uint8_t buf[3] = {
        reg,
        (uint8_t)(val & 0xFFU),
        (uint8_t)((val >> 8U) & 0xFFU)
    };
    return i2c_write_bytes(g_dev_xl9535, buf, sizeof(buf));
}

static bool xl9535_sync_outputs(bool screen_rst_high, bool touch_rst_high, bool lora_rst_high) {
    uint16_t out_word = p4_trx_build_xl9535_output_word(&g_p4, screen_rst_high, touch_rst_high, lora_rst_high);
    return xl9535_write_reg16(0x02U, out_word);
}

static bool init_xl9535_and_power_rails(void) {
    /* Configure direction mask:
     * Outputs (0): IO0(3V3_EN), IO1(SKY13453), IO2(LCD_RST), IO3(TP_RST), IO5(ETH_RST),
     *              IO6(5V0_AUDIO_EN), IO10(bit8 USB_PHY), IO11(bit9 GPS_WAKE),
     *              IO14(bit12 C6_EN), IO15(bit13 SD_PWR), IO16(bit14 LORA_RST)
     * Inputs  (1): IO4(TP_INT), IO7(IMU_INT), IO12(bit10 RTC_INT), IO13(bit11 C6_WAKE), IO17(bit15 LORA_DIO1)
     */
    const uint16_t dir_mask = P4_XL_BIT_TOUCH_INT |
                              P4_XL_BIT_IMU_INT   |
                              P4_XL_BIT_RTC_INT   |
                              P4_XL_BIT_C6_WAKE_UP|
                              P4_XL_BIT_LORA_DIO1;

    /* Preload output latches BEFORE driving pins as outputs so IO0 (3.3V_POWER_EN) never glitches high */
    if (!xl9535_sync_outputs(false, false, false)) {
        ESP_LOGE(TAG, "XL9535 output preload failed");
        return false;
    }
    if (!xl9535_write_reg16(0x06U, dir_mask)) {
        ESP_LOGE(TAG, "XL9535 direction config failed");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(25));

    /* Release SCREEN_RST, TOUCH_RST, and LORA_RST high */
    if (!xl9535_sync_outputs(true, true, true)) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(120));

    /* Ensure SGM38121 camera LDO channels stay off to save power */
    if (g_dev_sgm38121) {
        const uint8_t disable_cmd[2] = {0x0EU, 0x00U};
        (void)i2c_write_bytes(g_dev_sgm38121, disable_cmd, 2U);
    }
    return true;
}

/* ---------------------------------------------------------------------------
 * 2. Semtech LR2021 Onboard Dual-Band Radio Driver (SPI2 + 3.0V TCXO + AT2401C)
 * --------------------------------------------------------------------------- */
static bool p4_lr2021_wait_busy(uint32_t timeout_us) {
    int64_t t0 = esp_timer_get_time();
    while (gpio_get_level((gpio_num_t)P4_LR2021_BUSY_PIN) != 0) {
        if ((uint32_t)(esp_timer_get_time() - t0) > timeout_us) {
            return false;
        }
    }
    return true;
}

static void p4_lr2021_spi_xfer(uint8_t *buf, size_t len) {
    gpio_set_level((gpio_num_t)P4_LR2021_CS_PIN, 0);
    spi_transaction_t t = {};
    t.length    = len * 8U;
    t.tx_buffer = buf;
    t.rx_buffer = buf;
    (void)spi_device_polling_transmit(g_lr2021_spi, &t);
    gpio_set_level((gpio_num_t)P4_LR2021_CS_PIN, 1);
}

static uint16_t p4_lr2021_write_cmd(uint16_t opcode, const uint8_t *args, uint16_t arg_len, uint32_t wait_us) {
    uint8_t buf[260];
    if (arg_len > 255U) arg_len = 255U;
    buf[0] = (uint8_t)(opcode >> 8U);
    buf[1] = (uint8_t)(opcode & 0xFFU);
    if (arg_len > 0U && args) {
        memcpy(&buf[2], args, arg_len);
    }
    (void)p4_lr2021_wait_busy(wait_us);
    p4_lr2021_spi_xfer(buf, (size_t)arg_len + 2U);
    (void)p4_lr2021_wait_busy(wait_us);
    return (uint16_t)(((uint16_t)buf[0] << 8U) | buf[1]);
}

static uint16_t p4_lr2021_read_resp(uint8_t *buf, uint16_t len, uint32_t wait_us) {
    (void)p4_lr2021_wait_busy(wait_us);
    p4_lr2021_spi_xfer(buf, len);
    (void)p4_lr2021_wait_busy(wait_us);
    return (uint16_t)(((uint16_t)buf[0] << 8U) | buf[1]);
}

static uint16_t p4_lr2021_write_regmem32(uint32_t address, const uint32_t *words, uint8_t word_count) {
    uint8_t buf[5 + 32 * 4];
    if (word_count > 32U) word_count = 32U;
    buf[0] = (uint8_t)(LR2021_REGMEM_WRITE_REGMEM32_OC >> 8U);
    buf[1] = (uint8_t)(LR2021_REGMEM_WRITE_REGMEM32_OC & 0xFFU);
    buf[2] = (uint8_t)(address >> 16U);
    buf[3] = (uint8_t)(address >> 8U);
    buf[4] = (uint8_t)(address & 0xFFU);
    for (uint8_t i = 0U; i < word_count; i++) {
        uint32_t w = words[i];
        uint8_t *p = &buf[5U + i * 4U];
        p[0] = (uint8_t)(w >> 24U);
        p[1] = (uint8_t)(w >> 16U);
        p[2] = (uint8_t)(w >> 8U);
        p[3] = (uint8_t)(w & 0xFFU);
    }
    (void)p4_lr2021_wait_busy(20000U);
    p4_lr2021_spi_xfer(buf, 5U + (size_t)word_count * 4U);
    (void)p4_lr2021_wait_busy(20000U);
    return (uint16_t)(((uint16_t)buf[0] << 8U) | buf[1]);
}

static bool p4_lr2021_load_pram(void) {
    const uint32_t total_words = (uint32_t)(sizeof(pram_lr2021) / sizeof(uint32_t));
    const uint8_t  chunk_words = 12U;
    const uint32_t n_chunks    = total_words / chunk_words;
    for (uint32_t c = 0U; c < n_chunks; c++) {
        const uint32_t *src = &pram_lr2021[c * chunk_words];
        uint32_t addr = 0x00801000UL + c * chunk_words * 4U;
        if ((p4_lr2021_write_regmem32(addr, src, chunk_words) & 0x0400U) == 0U) return false;
    }
    uint8_t rem = (uint8_t)(total_words - n_chunks * chunk_words);
    if (rem > 0U) {
        const uint32_t *src = &pram_lr2021[n_chunks * chunk_words];
        uint32_t addr = 0x00801000UL + n_chunks * chunk_words * 4U;
        if ((p4_lr2021_write_regmem32(addr, src, rem) & 0x0400U) == 0U) return false;
    }
    uint8_t enable = 0x00U;
    return (p4_lr2021_write_cmd(LR2021_PATCH_ENABLE_PRAM_OC, &enable, 1U, 20000U) & 0x0400U) != 0U;
}

/* Hamming(7,4) + Interleaving for 2.4 GHz FSK 1000 Hz mode (8B -> 14B) */
static const uint8_t k_hamming_enc[16] = {
    0x00, 0x71, 0x62, 0x13, 0x54, 0x25, 0x36, 0x47,
    0x38, 0x49, 0x5A, 0x2B, 0x6C, 0x1D, 0x0E, 0x7F
};
static const uint8_t k_hamming_dec[128] = {
    0x00, 0x00, 0x00, 0x03, 0x00, 0x05, 0x0E, 0x07,
    0x00, 0x09, 0x0E, 0x0B, 0x0E, 0x0D, 0x0E, 0x0E,
    0x00, 0x03, 0x03, 0x03, 0x04, 0x0D, 0x06, 0x03,
    0x08, 0x0D, 0x0A, 0x03, 0x0D, 0x0D, 0x0E, 0x0D,
    0x00, 0x05, 0x02, 0x0B, 0x05, 0x05, 0x06, 0x05,
    0x08, 0x0B, 0x0B, 0x0B, 0x0C, 0x05, 0x0E, 0x0B,
    0x08, 0x01, 0x06, 0x03, 0x06, 0x05, 0x06, 0x06,
    0x08, 0x08, 0x08, 0x0B, 0x08, 0x0D, 0x06, 0x0F,
    0x00, 0x09, 0x02, 0x07, 0x04, 0x07, 0x07, 0x07,
    0x09, 0x09, 0x0A, 0x09, 0x0C, 0x09, 0x0E, 0x07,
    0x04, 0x01, 0x0A, 0x03, 0x04, 0x04, 0x04, 0x07,
    0x0A, 0x09, 0x0A, 0x0A, 0x04, 0x0D, 0x0A, 0x0F,
    0x02, 0x01, 0x02, 0x02, 0x0C, 0x05, 0x02, 0x07,
    0x0C, 0x09, 0x02, 0x0B, 0x0C, 0x0C, 0x0C, 0x0F,
    0x01, 0x01, 0x02, 0x01, 0x04, 0x01, 0x06, 0x0F,
    0x08, 0x01, 0x0A, 0x0F, 0x0C, 0x0F, 0x0F, 0x0F
};

static void fec_encode_8_to_14(const uint8_t *in8, uint8_t *out14) {
    uint8_t enc[16] = {0};
    memset(out14, 0, 14);
    for (uint8_t i = 0; i < 8; i++) {
        enc[i * 2 + 0] = k_hamming_enc[in8[i] & 0x0FU];
        enc[i * 2 + 1] = k_hamming_enc[in8[i] >> 4U];
    }
    for (uint8_t i = 0; i < 7; i++) {
        for (uint8_t j = 0; j < 8; j++) {
            out14[i * 2 + 0] |= (uint8_t)(((enc[j + 0] >> i) & 0x01U) << j);
            out14[i * 2 + 1] |= (uint8_t)(((enc[j + 8] >> i) & 0x01U) << j);
        }
    }
}

static void __attribute__((unused)) fec_decode_14_to_8(const uint8_t *in14, uint8_t *out8) {
    uint8_t enc[16] = {0};
    for (uint8_t i = 0; i < 8; i++) {
        for (uint8_t j = 0; j < 7; j++) {
            enc[i + 0] |= (uint8_t)(((in14[j * 2 + 0] >> i) & 0x01U) << j);
            enc[i + 8] |= (uint8_t)(((in14[j * 2 + 1] >> i) & 0x01U) << j);
        }
    }
    for (uint8_t i = 0; i < 8; i++) {
        out8[i]  = k_hamming_dec[enc[i * 2 + 0] & 0x7FU];
        out8[i] |= (uint8_t)(k_hamming_dec[enc[i * 2 + 1] & 0x7FU] << 4U);
    }
}

static void p4_lr2021_set_rf_freq(uint32_t freq_hz) {
    const uint8_t buf[4] = {
        (uint8_t)(freq_hz >> 24U),
        (uint8_t)(freq_hz >> 16U),
        (uint8_t)(freq_hz >> 8U),
        (uint8_t)(freq_hz & 0xFFU)
    };
    (void)p4_lr2021_write_cmd(LR2021_RADIO_SET_RF_FREQUENCY_OC, buf, 4U, 5000U);
}

static void p4_lr2021_apply_pa_and_path(uint32_t freq_hz, int8_t requested_dbm) {
    p4_lr2021_rf_setup_t setup = {};
    p4_trx_compute_lr2021_rf_setup(freq_hz, requested_dbm, &setup);

    bool is_sub_ghz = (freq_hz < 1000000000UL);
    p4_lr2021_set_rf_freq(freq_hz);

    const uint16_t fe_step = (uint16_t)(((freq_hz + 3999999UL) / 4000000UL) |
                                        (is_sub_ghz ? 0x0000U : 0x8000U));
    const uint8_t cal_fe[2] = {(uint8_t)(fe_step >> 8U), (uint8_t)(fe_step & 0xFFU)};
    const uint8_t rx_path[2] = {is_sub_ghz ? (uint8_t)0x00U : (uint8_t)0x01U,
                                is_sub_ghz ? (uint8_t)0x00U : (uint8_t)0x07U};
    (void)p4_lr2021_write_cmd(LR2021_RADIO_SET_RX_PATH_OC, rx_path, 2U, 10000U);
    (void)p4_lr2021_write_cmd(LR2021_SYSTEM_CALIBRATE_FRONTEND_OC, cal_fe, 2U, 50000U);
    p4_lr2021_set_rf_freq(freq_hz);

    const uint8_t pa_cfg[3] = {
        (uint8_t)((setup.pa_sel << 7U) | (setup.pa_lf_mode & 0x03U)),
        (uint8_t)((setup.pa_lf_duty_cycle << 4U) | (setup.pa_lf_slices & 0x0FU)),
        setup.pa_hf_duty_cycle
    };
    (void)p4_lr2021_write_cmd(LR2021_RADIO_SET_PA_CFG_OC, pa_cfg, 3U, 10000U);

    const uint8_t tx_params[2] = {(uint8_t)setup.power_half_dbm, LR2021_RADIO_RAMP_48_US};
    (void)p4_lr2021_write_cmd(LR2021_RADIO_SET_TX_PARAMS_OC, tx_params, 2U, 10000U);
}

static bool p4_lr2021_init(void) {
    /* Hardware reset LR2021 via XL9535 IO16 */
    (void)xl9535_sync_outputs(true, true, false);
    vTaskDelay(pdMS_TO_TICKS(15));
    (void)xl9535_sync_outputs(true, true, true);
    vTaskDelay(pdMS_TO_TICKS(40));

    uint8_t vresp[4] = {0};
    (void)p4_lr2021_write_cmd(LR2021_SYSTEM_GET_VERSION_OC, NULL, 0U, 10000U);
    (void)p4_lr2021_read_resp(vresp, 4U, 10000U);
    g_lr2021_fw_ver = (uint16_t)(((uint16_t)vresp[2] << 8U) | vresp[3]);
    ESP_LOGI(TAG, "LR2021 GET_VERSION = 0x%04X", (unsigned)g_lr2021_fw_ver);
    if (g_lr2021_fw_ver != 0x0118U) {
        return false;
    }

    if (!p4_lr2021_load_pram()) {
        ESP_LOGE(TAG, "LR2021 PRAM load failed");
        return false;
    }

    uint8_t stdby = 0x00U;
    (void)p4_lr2021_write_cmd(LR2021_SYSTEM_SET_STANDBY_OC, &stdby, 1U, 10000U);

    /* Configure T-Display P4 32 MHz TCXO at 3.0V (0x06), timeout = 32768 */
    const uint8_t tcxo_cmd[5] = {
        P4_LR2021_TCXO_TUNE_3V0,
        0x00U, 0x00U, 0x80U, 0x00U
    };
    (void)p4_lr2021_write_cmd(P4_LR2021_OC_SET_TCXO_MODE, tcxo_cmd, 5U, 50000U);

    const uint32_t sys_reg_val = 0x002CCCCCUL;
    (void)p4_lr2021_write_regmem32(0x0080004CUL, &sys_reg_val, 1U);
    const uint8_t lfclk_cfg = 0x00U;
    (void)p4_lr2021_write_cmd(0x0118U, &lfclk_cfg, 1U, 10000U);

    const uint8_t fb_mode = LR2021_RADIO_FALLBACK_FS;
    (void)p4_lr2021_write_cmd(LR2021_RADIO_SET_RX_TX_FALLBACK_MODE_OC, &fb_mode, 1U, 10000U);

    const uint8_t timeouts[6] = {0xFFU, 0xFFU, 0xFFU, 0x00U, 0x00U, 0x00U};
    (void)p4_lr2021_write_cmd(LR2021_RADIO_SET_DEFAULT_RX_TX_TIMEOUT_OC, timeouts, 6U, 10000U);

    /* Configure DIO6, DIO7, DIO8, DIO10 as RF switches for AT2401C + Sub-GHz diplexer, and DIO11 as IRQ */
    const uint8_t dio_pins[4] = {6U, 7U, 8U, 10U};
    const uint8_t dio_sw_cfg[4] = {0x04U, 0x08U, 0x03U, 0x0CU};
    for (int i = 0; i < 4; i++) {
        const uint8_t fn[2] = {dio_pins[i], 0x30U}; /* Function 0x3 = RF_SWITCH */
        (void)p4_lr2021_write_cmd(LR2021_SYSTEM_SET_DIO_FUNCTION_OC, fn, 2U, 10000U);
        const uint8_t sw[2] = {dio_pins[i], dio_sw_cfg[i]};
        (void)p4_lr2021_write_cmd(P4_LR2021_OC_SET_DIO_RF_SW_CFG, sw, 2U, 10000U);
    }

    const uint8_t dio11_fn[2] = {11U, 0x10U}; /* DIO11 = IRQ */
    (void)p4_lr2021_write_cmd(LR2021_SYSTEM_SET_DIO_FUNCTION_OC, dio11_fn, 2U, 10000U);
    const uint32_t irq_mask = (uint32_t)LR2021_IRQ_RX_DONE | (uint32_t)LR2021_IRQ_TX_DONE;
    const uint8_t dio11_irq[5] = {
        11U,
        (uint8_t)(irq_mask >> 24U),
        (uint8_t)(irq_mask >> 16U),
        (uint8_t)(irq_mask >> 8U),
        (uint8_t)(irq_mask & 0xFFU)
    };
    (void)p4_lr2021_write_cmd(LR2021_SYSTEM_SET_DIOIRQPARAMS_OC, dio11_irq, 5U, 10000U);

    const uint8_t reg_mode = 0x02U; /* DCDC enabled */
    (void)p4_lr2021_write_cmd(LR2021_SYSTEM_SET_REGMODE_OC, &reg_mode, 1U, 10000U);

    const uint8_t cal_mask = 0x7FU;
    (void)p4_lr2021_write_cmd(LR2021_SYSTEM_CALIBRATE_OC, &cal_mask, 1U, 50000U);

    (void)p4_lr2021_write_cmd(LR2021_SYSTEM_CLEAR_ERRORS_OC, NULL, 0U, 10000U);
    const uint8_t clr_irq[4] = {0xFFU, 0xFFU, 0xFFU, 0xFFU};
    (void)p4_lr2021_write_cmd(LR2021_SYSTEM_CLEAR_IRQ_OC, clr_irq, 4U, 10000U);

    g_lr2021_ready = true;
    return true;
}

static void p4_lr2021_configure_active_mode(void) {
    if (!g_lr2021_ready) return;
    (void)xl9535_sync_outputs(true, true, true);

    uint8_t stdby = 0x00U;
    (void)p4_lr2021_write_cmd(LR2021_SYSTEM_SET_STANDBY_OC, &stdby, 1U, 10000U);

    if (g_p4.app_mode == 3U || g_p4.app_mode == 4U) {
        /* Reticulum Mesh / RNode LoRa explicit-header mode */
        const uint8_t pkt_type = LR2021_RADIO_PKT_TYPE_LORA;
        (void)p4_lr2021_write_cmd(LR2021_RADIO_SET_PKT_TYPE_OC, &pkt_type, 1U, 10000U);
        p4_lr2021_apply_pa_and_path(g_p4.rns_freq_hz, g_p4.rns_txpower_dbm);

        uint8_t bw_code = (g_p4.rns_bw_hz >= 500000UL) ? LR2021_RADIO_LORA_BW_500 :
                          (g_p4.rns_bw_hz >= 250000UL) ? LR2021_RADIO_LORA_BW_250 : LR2021_RADIO_LORA_BW_125;
        uint8_t cr_code = (g_p4.rns_cr >= 5U && g_p4.rns_cr <= 8U) ? (uint8_t)(g_p4.rns_cr - 4U) : 1U;
        const uint8_t mod[2] = {(uint8_t)((g_p4.rns_sf << 4U) | bw_code), (uint8_t)(cr_code << 4U)};
        (void)p4_lr2021_write_cmd(LR2021_RADIO_SET_LORA_MODULATION_PARAM_OC, mod, 2U, 10000U);

        const uint8_t wrbuf_rns[11] = {0xF3U, 0x0AU, 0x14U, 0x00U, 0x0CU, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
        (void)p4_lr2021_write_cmd(LR2021_REGMEM_WRITE_REGMEM32_MASK_OC, wrbuf_rns, 11U, 10000U);

        const uint8_t pkt[4] = {0x00U, 18U, 64U, (uint8_t)((LR2021_LORA_PACKET_EXPLICIT << 2U) | (1U << 1U))};
        (void)p4_lr2021_write_cmd(LR2021_RADIO_SET_LORA_PACKET_PARAMS_OC, pkt, 4U, 10000U);
        uint8_t sw = 0x12U;
        (void)p4_lr2021_write_cmd(LR2021_RADIO_SET_LORA_SYNCWORD_OC, &sw, 1U, 10000U);
        (void)p4_lr2021_write_cmd(LR2021_RADIO_SET_RX_OC, NULL, 0U, 10000U);
        return;
    }

    const elrs_air_rate_config_t *rf = elrs_get_air_rate_by_index(g_p4.elrs_rate_idx);
    if (!rf) return;

    uint32_t init_freq = elrs_fhss_get_initial_freq(&g_p4.fhss);

    if (rf->phy_type == ELRS_PHY_FSK_2G4 || rf->phy_type == ELRS_PHY_FSK_900) {
        const uint8_t pkt_type = LR2021_RADIO_PKT_TYPE_FSK;
        (void)p4_lr2021_write_cmd(LR2021_RADIO_SET_PKT_TYPE_OC, &pkt_type, 1U, 10000U);
        p4_lr2021_apply_pa_and_path(init_freq, g_p4.tx_power_dbm);

        uint32_t br_bps  = (uint32_t)rf->bw * 10000UL;
        uint32_t fdev_hz = (uint32_t)rf->cr * 1000UL;
        bool use_fec     = (rf->payload_len == ELRS_OTA4_PACKET_SIZE);
        uint8_t wire_len = use_fec ? 14U : rf->payload_len;

        const uint8_t mod_params[9] = {
            (uint8_t)(br_bps >> 24U), (uint8_t)(br_bps >> 16U), (uint8_t)(br_bps >> 8U), (uint8_t)(br_bps & 0xFFU),
            LR2021_RADIO_GFSK_PULSE_SHAPE_OFF, rf->sf,
            (uint8_t)(fdev_hz >> 16U), (uint8_t)(fdev_hz >> 8U), (uint8_t)(fdev_hz & 0xFFU)
        };
        (void)p4_lr2021_write_cmd(LR2021_RADIO_SET_FSK_MODULATION_PARAMS_OC, mod_params, 9U, 10000U);

        const uint8_t pkt_params[7] = {
            (uint8_t)(rf->preamble_len >> 8U), (uint8_t)(rf->preamble_len & 0xFFU),
            LR2021_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_8BITS, 0x00U, 0U, wire_len,
            (uint8_t)(LR2021_RADIO_GFSK_CRC_OFF | LR2021_RADIO_GFSK_DC_FREE_WHITENING)
        };
        (void)p4_lr2021_write_cmd(LR2021_RADIO_SET_FSK_PACKET_PARAMS_OC, pkt_params, 7U, 10000U);

        const uint8_t whitening[2] = {0x01U, 0x00U};
        (void)p4_lr2021_write_cmd(LR2021_RADIO_SET_FSK_WHITENING_PARAMS_OC, whitening, 2U, 10000U);

        const uint8_t sync_buf[9] = {
            0x55U, 0x55U, 0x55U, 0x55U, 0x55U, 0x55U,
            g_p4.ota.uid[5], g_p4.ota.uid[4],
            (uint8_t)(LR2021_GFSK_SYNCWORD_BIT_ORDER_MSB | 16U)
        };
        (void)p4_lr2021_write_cmd(LR2021_RADIO_SET_FSK_SYNC_WORD_OC, sync_buf, 9U, 10000U);
    } else if (rf->phy_type == ELRS_PHY_FLRC_2G4) {
        const uint8_t pkt_type = LR2021_RADIO_PKT_TYPE_FLRC;
        (void)p4_lr2021_write_cmd(LR2021_RADIO_SET_PKT_TYPE_OC, &pkt_type, 1U, 10000U);
        p4_lr2021_apply_pa_and_path(init_freq, g_p4.tx_power_dbm);

        const uint8_t mod_params[2] = {rf->bw, (uint8_t)((rf->cr << 4U) | (rf->sf & 0x0FU))};
        (void)p4_lr2021_write_cmd(LR2021_RADIO_SET_FLRC_MODULATION_PARAMS_OC, mod_params, 2U, 10000U);

        uint8_t pbl_code = (rf->preamble_len >= 4U) ? (uint8_t)((rf->preamble_len / 4U) - 1U) : 0U;
        const uint8_t pkt_params[4] = {
            (uint8_t)((pbl_code << 2U) | LR2021_RADIO_FLRC_SYNC_32BITS),
            (uint8_t)((1U << 6U) | (1U << 3U) | LR2021_RADIO_FLRC_PAYLOAD_FIXED_LEN | LR2021_RADIO_FLRC_CRC_24BITS),
            0U, rf->payload_len
        };
        (void)p4_lr2021_write_cmd(LR2021_RADIO_SET_FLRC_PACKET_PARAMS_OC, pkt_params, 4U, 10000U);

        uint32_t sw32 = g_p4.ota.uid_seed;
        uint16_t crc_seed = g_p4.ota.crc_init;
        const uint32_t poly = 0x005D6DCBUL;
        const uint8_t crc_params[8] = {
            (uint8_t)(poly >> 24U), (uint8_t)(poly >> 16U), (uint8_t)(poly >> 8U), (uint8_t)(poly & 0xFFU),
            0x00U, 0xFFU, (uint8_t)(crc_seed >> 8U), (uint8_t)(crc_seed & 0xFFU)
        };
        (void)p4_lr2021_write_cmd(LR2021_RADIO_SET_FSK_CRC_PARAMS_OC, crc_params, 8U, 10000U);

        uint8_t sw_buf[5] = {1U, (uint8_t)(sw32 >> 24U), (uint8_t)(sw32 >> 16U), (uint8_t)(sw32 >> 8U), (uint8_t)(sw32 & 0xFFU)};
        if ((sw_buf[1] == 0x8CU && sw_buf[2] == 0x38U) || (sw_buf[1] == 0x63U && sw_buf[2] == 0x0EU)) {
            uint8_t tmp = sw_buf[1];
            sw_buf[1] = sw_buf[2];
            sw_buf[2] = tmp;
        }
        (void)p4_lr2021_write_cmd(LR2021_RADIO_SET_FLRC_SYNCWORD_OC, sw_buf, 5U, 10000U);
    } else {
        /* Sub-GHz or 2.4 GHz LoRa */
        const uint8_t pkt_type = LR2021_RADIO_PKT_TYPE_LORA;
        (void)p4_lr2021_write_cmd(LR2021_RADIO_SET_PKT_TYPE_OC, &pkt_type, 1U, 10000U);
        p4_lr2021_apply_pa_and_path(init_freq, g_p4.tx_power_dbm);

        const uint8_t mod_params[2] = {(uint8_t)((rf->sf << 4U) | (rf->bw & 0x0FU)), (uint8_t)(rf->cr << 4U)};
        (void)p4_lr2021_write_cmd(LR2021_RADIO_SET_LORA_MODULATION_PARAM_OC, mod_params, 2U, 10000U);

        bool is_sub = (rf->phy_type == ELRS_PHY_LORA_900);
        uint8_t compat_byte = (is_sub && rf->sf == LR2021_RADIO_LORA_SF6) ? 0x08U : 0x00U;
        const uint8_t wrbuf[11] = {0xF3U, 0x0AU, 0x14U, 0x00U, 0x0CU, 0x00U, 0x00U, 0x00U, compat_byte, 0x00U, 0x00U};
        (void)p4_lr2021_write_cmd(LR2021_REGMEM_WRITE_REGMEM32_MASK_OC, wrbuf, 11U, 10000U);
        uint8_t iq_bit = (!is_sub && (g_p4.uid[5] & 0x01U)) ? LR2021_RADIO_LORA_IQ_INVERTED : LR2021_RADIO_LORA_IQ_STANDARD;
        const uint8_t pkt_params[4] = {
            (uint8_t)(rf->preamble_len >> 8U), (uint8_t)(rf->preamble_len & 0xFFU),
            rf->payload_len, (uint8_t)((LR2021_LORA_PACKET_IMPLICIT << 2U) | iq_bit)
        };
        (void)p4_lr2021_write_cmd(LR2021_RADIO_SET_LORA_PACKET_PARAMS_OC, pkt_params, 4U, 10000U);
        uint8_t sw = 0x12U;
        (void)p4_lr2021_write_cmd(LR2021_RADIO_SET_LORA_SYNCWORD_OC, &sw, 1U, 10000U);
    }

    (void)p4_lr2021_write_cmd(LR2021_SYSTEM_CLEAR_RX_FIFO_OC, NULL, 0U, 5000U);
    (void)p4_lr2021_write_cmd(LR2021_SYSTEM_CLEAR_TX_FIFO_OC, NULL, 0U, 5000U);
}

static void p4_lr2021_transmit_frame(const uint8_t *data, uint8_t len, bool is_fsk_2g4_fec) {
    uint8_t wire_buf[64];
    uint8_t wire_len = len;
    if (is_fsk_2g4_fec && len == 8U) {
        fec_encode_8_to_14(data, wire_buf);
        wire_len = 14U;
    } else {
        memcpy(wire_buf, data, wire_len);
    }
    (void)p4_lr2021_write_cmd(LR2021_SYSTEM_SET_FS_OC, NULL, 0U, 5000U);
    (void)p4_lr2021_write_cmd(LR2021_RADIO_WRITE_TX_FIFO, wire_buf, wire_len, 5000U);
    (void)p4_lr2021_write_cmd(LR2021_RADIO_SET_TX_OC, NULL, 0U, 5000U);
}

/* ---------------------------------------------------------------------------
 * 3. MIPI-DSI Display (HI8561 TFT & RM69A10 AMOLED) & Multi-Touch Drivers
 * --------------------------------------------------------------------------- */
static const uint8_t k_hi8561_init_seq[] = {
    2, 0xDF, 3, 0x90, 0x69, 0xF9,
    1, 0xDE, 0x00,
    2, 0xBB, 7, 0x0F, 0x10, 0x43, 0x50, 0x32, 0x44, 0x44,
    2, 0xBF, 2, 0x46, 0x32,
    2, 0xC0, 4, 0x01, 0xAD, 0x01, 0xAD,
    2, 0xBD, 2, 0x00, 0xB4,
    2, 0xC6, 23, 0x00, 0x7D, 0x00, 0xC8, 0x00, 0x17, 0x1A, 0x82, 0x00, 0x00, 0x00, 0x01,
                 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x01,
    2, 0xC8, 3, 0x23, 0x48, 0x87,
    1, 0xCC, 0x31,
    2, 0xBC, 3, 0x2E, 0x80, 0x84,
    2, 0xC3, 25, 0x3B, 0x01, 0x02, 0x05, 0x0C, 0x0C, 0x75, 0x0A, 0x79, 0x0A, 0x79, 0x02,
                 0x6E, 0x02, 0x6E, 0x02, 0x6E, 0x0A, 0x0D, 0x0A, 0x0F, 0x0A, 0x0F, 0x0A, 0x0F,
    2, 0xC4, 24, 0x01, 0x02, 0x05, 0x0C, 0x0C, 0x75, 0x0A, 0x79, 0x0A, 0x79, 0x02, 0x6E,
                 0x02, 0x6E, 0x02, 0x6E, 0x0A, 0x0D, 0x0A, 0x0F, 0x0A, 0x0F, 0x0A, 0x0F,
    2, 0xC5, 23, 0x03, 0x05, 0x0C, 0x0C, 0x75, 0x0A, 0x79, 0x0A, 0x79, 0x02, 0x6E, 0x02,
                 0x6E, 0x02, 0x6E, 0x0A, 0x0D, 0x0A, 0x0F, 0x0A, 0x0F, 0x0A, 0x0F,
    2, 0xD7, 17, 0x00, 0x0A, 0x63, 0x0A, 0x63, 0x0A, 0x63, 0x0A, 0x63, 0x0A, 0x63, 0x0A,
                 0x63, 0x0A, 0x63, 0x0A, 0x63,
    2, 0xCB, 43, 0x7F, 0x78, 0x71, 0x64, 0x5A, 0x58, 0x4B, 0x51, 0x3A, 0x53, 0x51, 0x4F,
                 0x6A, 0x54, 0x57, 0x46, 0x3F, 0x2F, 0x1B, 0x0F, 0x08, 0x7F, 0x78, 0x71,
                 0x64, 0x5A, 0x58, 0x4B, 0x51, 0x3A, 0x53, 0x51, 0x4F, 0x6A, 0x54, 0x57,
                 0x46, 0x3F, 0x2F, 0x1B, 0x0F, 0x08, 0x00,
    2, 0xCE, 23, 0x00, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C,
                 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C,
    2, 0xCF, 45, 0x00, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30,
                 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x00,
                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    2, 0xD0, 29, 0x00, 0x1F, 0x1F, 0x11, 0x1E, 0x1F, 0x0F, 0x0F, 0x0D, 0x0D, 0x0B, 0x0B,
                 0x09, 0x09, 0x07, 0x07, 0x05, 0x05, 0x01, 0x1F, 0x1F, 0x1F, 0x1F, 0x00,
                 0x00, 0x00, 0x00, 0x00, 0x00,
    2, 0xD1, 29, 0x00, 0x1F, 0x1F, 0x10, 0x1E, 0x1F, 0x0E, 0x0E, 0x0C, 0x0C, 0x0A, 0x0A,
                 0x08, 0x08, 0x06, 0x06, 0x04, 0x04, 0x00, 0x1F, 0x1F, 0x1F, 0x1F, 0x00,
                 0x00, 0x00, 0x00, 0x00, 0x00,
    2, 0xD2, 29, 0x00, 0x5F, 0x1F, 0x10, 0x1F, 0x1E, 0x08, 0x08, 0x4A, 0x0A, 0x0C, 0x0C,
                 0x0E, 0x0E, 0x04, 0x04, 0x06, 0x06, 0x00, 0x1F, 0x1F, 0x1F, 0x1F, 0x00,
                 0x00, 0x00, 0x00, 0x00, 0x00,
    2, 0xD3, 29, 0x00, 0x1F, 0x1F, 0x11, 0x1F, 0x1E, 0x09, 0x09, 0x0B, 0x0B, 0x0D, 0x0D,
                 0x0F, 0x0F, 0x05, 0x05, 0x07, 0x07, 0x01, 0x1F, 0x1F, 0x1F, 0x1F, 0x00,
                 0x00, 0x00, 0x00, 0x00, 0x00,
    2, 0xD4, 87, 0x00, 0x20, 0x0B, 0x00, 0x0D, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00,
                 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x03, 0x00, 0x81, 0x04, 0xAE,
                 0x04, 0xB0, 0x04, 0xB2, 0x04, 0xB4, 0x04, 0xB6, 0x04, 0xB8, 0x00, 0x00,
                 0x00, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x00, 0x06, 0x44, 0x06, 0x46,
                 0x03, 0x03, 0x00, 0x00, 0x07, 0x00, 0x06, 0x04, 0xA7, 0x04, 0xA8, 0x00,
                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00,
                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x01, 0x00,
                 0x00, 0x20, 0x00,
    2, 0xD5, 61, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x00, 0x00,
                 0x00, 0x07, 0x32, 0x5A, 0x00, 0x00, 0x3C, 0x00, 0x1E, 0x00, 0x1E, 0xB3,
                 0x00, 0x0F, 0x06, 0x0C, 0x00, 0x71, 0x20, 0x04, 0x10, 0x04, 0x06, 0x00,
                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x1F,
                 0xFF, 0x00, 0x00, 0x00, 0x1F, 0xFF, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    2, 0xCD, 2, 0x00, 0x00,
    1, 0xDE, 0x01,
    2, 0xB9, 4, 0x00, 0xFF, 0xFF, 0x04,
    2, 0xC7, 3, 0x1F, 0x14, 0x0E,
    1, 0xDE, 0x02,
    2, 0xE5, 24, 0x00, 0x60, 0x60, 0x02, 0x18, 0x60, 0x18, 0x60, 0x09, 0x04, 0x00, 0xC5,
                 0x01, 0x2C, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x04,
    2, 0xE6, 3, 0x10, 0x10, 0x82,
    2, 0xC4, 7, 0x00, 0x11, 0x07, 0x00, 0x11, 0x01, 0x08,
    2, 0xC3, 2, 0x20, 0xFF,
    1, 0xBD, 0x1B,
    2, 0xC6, 2, 0x4A, 0x00,
    2, 0xCD, 4, 0x14, 0x64, 0x11, 0x40,
    2, 0xC1, 10, 0x00, 0x40, 0x00, 0x02, 0x02, 0x02, 0x02, 0x7F, 0x00, 0x00,
    2, 0xB3, 2, 0x00, 0xA8,
    2, 0xBB, 11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x41, 0x40, 0x43, 0x04,
    2, 0xC2, 10, 0x02, 0x42, 0x50, 0x00, 0x02, 0xE4, 0x61, 0x73, 0xF9, 0x08,
    2, 0xEC, 14, 0x07, 0x07, 0x40, 0x00, 0x22, 0x02, 0x00, 0xFF, 0x08, 0x7C, 0x00, 0x00, 0x00, 0x00,
    1, 0xDE, 0x03,
    2, 0xD1, 5, 0x00, 0x00, 0x21, 0xFF, 0x00,
    1, 0xDE, 0x00,
    0, 0x35,
    3, 30,
    0, 0x11,
    3, 120,
    0, 0x29,
    0xFF
};

static bool init_mipi_dsi_and_touch(p4_panel_variant_t *out_panel) {
    /* Probe I2C1 for GT9895 (0x5D -> RM69A10 AMOLED) vs HI8561 Touch (0x68 -> HI8561 TFT) */
    p4_panel_variant_t variant = P4_PANEL_HI8561_TFT;
    if (i2c_master_probe(g_i2c1_bus, P4_GT9895_I2C_ADDR, 30) == ESP_OK) {
        variant = P4_PANEL_RM69A10_AMOLED;
        ESP_LOGI(TAG, "Detected GT9895 Touch (0x5D) -> RM69A10 568x1232 AMOLED Panel");
    } else if (i2c_master_probe(g_i2c1_bus, P4_HI8561_TOUCH_I2C_ADDR, 30) == ESP_OK) {
        variant = P4_PANEL_HI8561_TFT;
        ESP_LOGI(TAG, "Detected HI8561 Touch (0x68) -> HI8561 540x1168 TFT Panel");
    } else {
        ESP_LOGW(TAG, "No touch ACK on 0x5D/0x68; defaulting to HI8561 540x1168");
    }
    if (out_panel) *out_panel = variant;

    /* Attach touch I2C device handle */
    i2c_device_config_t tcfg = {};
    tcfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    tcfg.device_address  = (variant == P4_PANEL_RM69A10_AMOLED) ? P4_GT9895_I2C_ADDR : P4_HI8561_TOUCH_I2C_ADDR;
    tcfg.scl_speed_hz    = 400000;
    if (i2c_master_bus_add_device(g_i2c1_bus, &tcfg, &g_dev_touch) == ESP_OK) {
        if (variant == P4_PANEL_HI8561_TFT) {
            /* Query HI8561 touch info start address from 0x200110CC */
            const uint8_t cmd[6] = {0xF3U, 0x20U, 0x01U, 0x10U, 0xCCU, 0x03U};
            uint8_t buf[48] = {0};
            if (i2c_write_read(g_dev_touch, cmd, 6U, buf, 48U)) {
                uint32_t addr = ((uint32_t)buf[7] << 24U) | ((uint32_t)buf[6] << 16U) |
                                ((uint32_t)buf[5] << 8U)  | ((uint32_t)buf[4]);
                if (addr >= 0x20011000UL && addr < 0x20012000UL) {
                    g_hi8561_touch_info_addr = addr;
                    g_touch_ready = true;
                    ESP_LOGI(TAG, "HI8561 Touch info address = 0x%08lX", (unsigned long)addr);
                }
            }
        } else {
            g_touch_ready = true;
        }
    }

    /* Configure GPIO51 PT4103 Backlight PWM for HI8561 TFT */
    if (variant == P4_PANEL_HI8561_TFT) {
        ledc_timer_config_t bl_tmr = {};
        bl_tmr.speed_mode      = LEDC_LOW_SPEED_MODE;
        bl_tmr.duty_resolution = LEDC_TIMER_10_BIT;
        bl_tmr.timer_num       = LEDC_TIMER_0;
        bl_tmr.freq_hz         = 1000;
        bl_tmr.clk_cfg         = LEDC_AUTO_CLK;
        (void)ledc_timer_config(&bl_tmr);

        ledc_channel_config_t bl_ch = {};
        bl_ch.gpio_num   = P4_BL_PWM_PIN;
        bl_ch.speed_mode = LEDC_LOW_SPEED_MODE;
        bl_ch.channel    = LEDC_CHANNEL_0;
        bl_ch.timer_sel  = LEDC_TIMER_0;
        bl_ch.duty       = 870; /* ~85% brightness */
        (void)ledc_channel_config(&bl_ch);
    }

    /* Create 2-lane 1000 Mbps MIPI-DSI bus */
    esp_lcd_dsi_bus_handle_t dsi_bus = NULL;
    esp_lcd_dsi_bus_config_t bus_cfg = {};
    bus_cfg.bus_id             = 0;
    bus_cfg.num_data_lanes     = 2;
    bus_cfg.phy_clk_src        = MIPI_DSI_PHY_CLK_SRC_DEFAULT;
    bus_cfg.lane_bit_rate_mbps = 1000;
    if (esp_lcd_new_dsi_bus(&bus_cfg, &dsi_bus) != ESP_OK) {
        ESP_LOGE(TAG, "esp_lcd_new_dsi_bus failed");
        return false;
    }

    esp_lcd_dbi_io_config_t dbi_cfg = {};
    dbi_cfg.virtual_channel = 0;
    dbi_cfg.lcd_cmd_bits    = 8;
    dbi_cfg.lcd_param_bits  = 8;
    if (esp_lcd_new_panel_io_dbi(dsi_bus, &dbi_cfg, &g_dbi_io) != ESP_OK) {
        ESP_LOGE(TAG, "esp_lcd_new_panel_io_dbi failed");
        return false;
    }

    esp_lcd_dpi_panel_config_t dpi_cfg = {};
    dpi_cfg.virtual_channel    = 0;
    dpi_cfg.dpi_clk_src        = MIPI_DSI_DPI_CLK_SRC_DEFAULT;
    dpi_cfg.dpi_clock_freq_mhz = 60;
    dpi_cfg.pixel_format       = LCD_COLOR_PIXEL_FORMAT_RGB565;
    dpi_cfg.num_fbs            = 1;
    dpi_cfg.flags.use_dma2d    = true;

    if (variant == P4_PANEL_RM69A10_AMOLED) {
        dpi_cfg.video_timing.h_size            = 568;
        dpi_cfg.video_timing.v_size            = 1232;
        dpi_cfg.video_timing.hsync_pulse_width = 50;
        dpi_cfg.video_timing.hsync_back_porch  = 150;
        dpi_cfg.video_timing.hsync_front_porch = 50;
        dpi_cfg.video_timing.vsync_pulse_width = 40;
        dpi_cfg.video_timing.vsync_back_porch  = 120;
        dpi_cfg.video_timing.vsync_front_porch = 80;
    } else {
        dpi_cfg.video_timing.h_size            = 540;
        dpi_cfg.video_timing.v_size            = 1168;
        dpi_cfg.video_timing.hsync_pulse_width = 28;
        dpi_cfg.video_timing.hsync_back_porch  = 26;
        dpi_cfg.video_timing.hsync_front_porch = 20;
        dpi_cfg.video_timing.vsync_pulse_width = 2;
        dpi_cfg.video_timing.vsync_back_porch  = 22;
        dpi_cfg.video_timing.vsync_front_porch = 200;
    }

    if (esp_lcd_new_panel_dpi(dsi_bus, &dpi_cfg, &g_dpi_panel) != ESP_OK) {
        ESP_LOGE(TAG, "esp_lcd_new_panel_dpi failed");
        return false;
    }

    /* Send panel-specific MIPI-DCS init commands */
    if (variant == P4_PANEL_RM69A10_AMOLED) {
        const uint8_t c_fe_fd[1] = {0xFDU};
        const uint8_t c_80_fc[1] = {0xFCU};
        const uint8_t c_fe_00[1] = {0x00U};
        const uint8_t c_2a[4]    = {0x00U, 0x00U, 0x02U, 0x37U};
        const uint8_t c_2b[4]    = {0x00U, 0x00U, 0x04U, 0xCFU};
        const uint8_t c_31[4]    = {0x00U, 0x03U, 0x02U, 0x34U};
        const uint8_t c_30[4]    = {0x00U, 0x00U, 0x04U, 0xCFU};
        const uint8_t c_zero[1]  = {0x00U};
        const uint8_t c_3a_565[1]= {0x75U};
        const uint8_t c_51_brt[1]= {0xE0U};
        (void)esp_lcd_panel_io_tx_param(g_dbi_io, 0xFE, c_fe_fd, 1);
        (void)esp_lcd_panel_io_tx_param(g_dbi_io, 0x80, c_80_fc, 1);
        (void)esp_lcd_panel_io_tx_param(g_dbi_io, 0xFE, c_fe_00, 1);
        (void)esp_lcd_panel_io_tx_param(g_dbi_io, 0x2A, c_2a, 4);
        (void)esp_lcd_panel_io_tx_param(g_dbi_io, 0x2B, c_2b, 4);
        (void)esp_lcd_panel_io_tx_param(g_dbi_io, 0x31, c_31, 4);
        (void)esp_lcd_panel_io_tx_param(g_dbi_io, 0x30, c_30, 4);
        (void)esp_lcd_panel_io_tx_param(g_dbi_io, 0x12, c_zero, 1);
        (void)esp_lcd_panel_io_tx_param(g_dbi_io, 0x35, c_zero, 1);
        (void)esp_lcd_panel_io_tx_param(g_dbi_io, 0x3A, c_3a_565, 1);
        (void)esp_lcd_panel_io_tx_param(g_dbi_io, 0x11, NULL, 0);
        vTaskDelay(pdMS_TO_TICKS(120));
        (void)esp_lcd_panel_io_tx_param(g_dbi_io, 0x29, NULL, 0);
        (void)esp_lcd_panel_io_tx_param(g_dbi_io, 0x51, c_51_brt, 1);
    } else {
        size_t idx = 0U;
        while (k_hi8561_init_seq[idx] != 0xFFU) {
            uint8_t fmt = k_hi8561_init_seq[idx++];
            if (fmt == 0U) {
                uint8_t cmd = k_hi8561_init_seq[idx++];
                (void)esp_lcd_panel_io_tx_param(g_dbi_io, cmd, NULL, 0);
            } else if (fmt == 1U) {
                uint8_t cmd = k_hi8561_init_seq[idx++];
                uint8_t dat = k_hi8561_init_seq[idx++];
                (void)esp_lcd_panel_io_tx_param(g_dbi_io, cmd, &dat, 1);
            } else if (fmt == 2U) {
                uint8_t cmd = k_hi8561_init_seq[idx++];
                uint8_t len = k_hi8561_init_seq[idx++];
                (void)esp_lcd_panel_io_tx_param(g_dbi_io, cmd, &k_hi8561_init_seq[idx], len);
                idx += len;
            } else if (fmt == 3U) {
                uint8_t ms = k_hi8561_init_seq[idx++];
                vTaskDelay(pdMS_TO_TICKS(ms));
            }
        }
    }

    (void)esp_lcd_panel_init(g_dpi_panel);
    return true;
}

static void poll_multitouch(p4_touch_frame_t *out_tf) {
    memset(out_tf, 0, sizeof(*out_tf));
    if (!g_touch_ready || !g_dev_touch) return;

    if (g_p4.panel_variant == P4_PANEL_HI8561_TFT) {
        const uint8_t cmd[6] = {
            0xF3U,
            (uint8_t)(g_hi8561_touch_info_addr >> 24U),
            (uint8_t)(g_hi8561_touch_info_addr >> 16U),
            (uint8_t)(g_hi8561_touch_info_addr >> 8U),
            (uint8_t)(g_hi8561_touch_info_addr & 0xFFU),
            0x03U
        };
        uint8_t buf[53] = {0};
        if (!i2c_write_read(g_dev_touch, cmd, 6U, buf, sizeof(buf))) return;
        uint8_t fingers = buf[0];
        if (fingers == 0U || fingers > P4_MAX_TOUCH_POINTS) return;
        out_tf->count = fingers;
        for (uint8_t i = 0U; i < fingers; i++) {
            size_t off = 3U + (size_t)i * 5U;
            uint16_t rx = (uint16_t)(((uint16_t)buf[off + 0] << 8U) | buf[off + 1]);
            uint16_t ry = (uint16_t)(((uint16_t)buf[off + 2] << 8U) | buf[off + 3]);
            uint16_t lx = 0U, ly = 0U;
            p4_trx_map_raw_touch_to_landscape(g_p4.panel_variant, rx, ry, &lx, &ly);
            out_tf->points[i].valid    = true;
            out_tf->points[i].id       = i;
            out_tf->points[i].x        = lx;
            out_tf->points[i].y        = ly;
            out_tf->points[i].pressure = buf[off + 4];
        }
    } else {
        /* GT9895 multi-touch read at 0x00010308 */
        const uint8_t cmd[4] = {0x00U, 0x01U, 0x03U, 0x08U};
        uint8_t buf[90] = {0};
        if (!i2c_write_read(g_dev_touch, cmd, 4U, buf, sizeof(buf))) return;
        if ((buf[0] & 0x80U) == 0U) return;
        uint8_t fingers = (uint8_t)(buf[0] & 0x0FU);
        if (fingers == 0U || fingers > P4_MAX_TOUCH_POINTS) return;
        uint8_t valid_cnt = 0U;
        for (uint8_t i = 0U; i < fingers; i++) {
            size_t off = 8U + (size_t)i * 8U;
            uint8_t id  = (uint8_t)((buf[off] >> 4U) & 0x0FU);
            uint16_t rx = (uint16_t)(((uint16_t)buf[off + 3] << 8U) | buf[off + 2]);
            uint16_t ry = (uint16_t)(((uint16_t)buf[off + 5] << 8U) | buf[off + 4]);
            if ((rx == 0U && ry == 0U) || rx >= g_p4.screen_h || ry >= g_p4.screen_w) continue;
            uint16_t lx = 0U, ly = 0U;
            p4_trx_map_raw_touch_to_landscape(g_p4.panel_variant, rx, ry, &lx, &ly);
            out_tf->points[valid_cnt].valid    = true;
            out_tf->points[valid_cnt].id       = id;
            out_tf->points[valid_cnt].x        = lx;
            out_tf->points[valid_cnt].y        = ly;
            out_tf->points[valid_cnt].pressure = buf[off + 6];
            valid_cnt++;
        }
        out_tf->count = valid_cnt;
    }
}

/* ---------------------------------------------------------------------------
 * 4. Onboard Sensors: BQ27220 Battery, ICM20948/QMI8658 IMU, AW86224 Haptic
 * --------------------------------------------------------------------------- */
static void poll_bq27220_battery(void) {
    if (!g_dev_bq27220) return;
    uint8_t reg = 0x08U, b[2] = {0};
    if (i2c_write_read(g_dev_bq27220, &reg, 1U, b, 2U)) {
        uint16_t mv = (uint16_t)(((uint16_t)b[1] << 8U) | b[0]);
        if (mv >= 2500U && mv <= 4500U) {
            g_p4.battery.gauge_present = true;
            g_p4.battery.voltage_mv    = mv;
        }
    }
    reg = 0x0CU;
    if (i2c_write_read(g_dev_bq27220, &reg, 1U, b, 2U)) {
        g_p4.battery.current_ma = (int16_t)(((uint16_t)b[1] << 8U) | b[0]);
    }
    reg = 0x2CU;
    if (i2c_write_read(g_dev_bq27220, &reg, 1U, b, 2U)) {
        uint8_t soc = b[0];
        if (soc <= 100U) g_p4.battery.soc_pct = soc;
    }
}

static void init_and_poll_imu(bool first_init) {
    if (first_init) {
        if (i2c_master_probe(g_i2c2_bus, P4_ICM20948_I2C_ADDR, 30) == ESP_OK) {
            i2c_device_config_t dcfg = {.dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = P4_ICM20948_I2C_ADDR, .scl_speed_hz = 400000};
            if (i2c_master_bus_add_device(g_i2c2_bus, &dcfg, &g_dev_imu) == ESP_OK) {
                const uint8_t bank0[2] = {0x7FU, 0x00U};
                const uint8_t wake[2]  = {0x06U, 0x01U}; /* PWR_MGMT_1 = Auto clock */
                (void)i2c_write_bytes(g_dev_imu, bank0, 2U);
                (void)i2c_write_bytes(g_dev_imu, wake, 2U);
                g_p4.imu.imu_present = true;
                g_p4.imu.is_icm20948 = true;
                ESP_LOGI(TAG, "Detected ICM20948 9-Axis IMU on I2C2 (0x68)");
            }
        } else if (i2c_master_probe(g_i2c2_bus, P4_QMI8658_I2C_ADDR, 30) == ESP_OK) {
            i2c_device_config_t dcfg = {.dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = P4_QMI8658_I2C_ADDR, .scl_speed_hz = 400000};
            if (i2c_master_bus_add_device(g_i2c2_bus, &dcfg, &g_dev_imu) == ESP_OK) {
                const uint8_t ctrl1[2] = {0x02U, 0x60U};
                const uint8_t ctrl7[2] = {0x08U, 0x03U};
                (void)i2c_write_bytes(g_dev_imu, ctrl1, 2U);
                (void)i2c_write_bytes(g_dev_imu, ctrl7, 2U);
                g_p4.imu.imu_present = true;
                g_p4.imu.is_icm20948 = false;
                ESP_LOGI(TAG, "Detected QMI8658 6-Axis IMU on I2C2 (0x6A)");
            }
        }
        return;
    }

    if (!g_p4.imu.imu_present || !g_dev_imu) return;
    if (g_p4.imu.is_icm20948) {
        uint8_t reg = 0x2DU; /* ACCEL_XOUT_H */
        uint8_t raw[6] = {0};
        if (i2c_write_read(g_dev_imu, &reg, 1U, raw, 6U)) {
            int16_t ax = (int16_t)(((uint16_t)raw[0] << 8U) | raw[1]);
            int16_t ay = (int16_t)(((uint16_t)raw[2] << 8U) | raw[3]);
            int16_t az = (int16_t)(((uint16_t)raw[4] << 8U) | raw[5]);
            float gx = (float)ax / 16384.0f;
            float gy = (float)ay / 16384.0f;
            float gz = (float)az / 16384.0f;
            g_p4.imu.accel_g[0] = gx;
            g_p4.imu.accel_g[1] = gy;
            g_p4.imu.accel_g[2] = gz;
            g_p4.imu.roll_deg   = atan2f(gy, gz) * (180.0f / 3.14159265f);
            g_p4.imu.pitch_deg  = atan2f(-gx, sqrtf(gy * gy + gz * gz)) * (180.0f / 3.14159265f);
        }
    } else {
        uint8_t reg = 0x35U; /* QMI8658 AX_L */
        uint8_t raw[6] = {0};
        if (i2c_write_read(g_dev_imu, &reg, 1U, raw, 6U)) {
            int16_t ax = (int16_t)(((uint16_t)raw[1] << 8U) | raw[0]);
            int16_t ay = (int16_t)(((uint16_t)raw[3] << 8U) | raw[2]);
            int16_t az = (int16_t)(((uint16_t)raw[5] << 8U) | raw[4]);
            float gx = (float)ax / 8192.0f;
            float gy = (float)ay / 8192.0f;
            float gz = (float)az / 8192.0f;
            g_p4.imu.roll_deg  = atan2f(gy, gz) * (180.0f / 3.14159265f);
            g_p4.imu.pitch_deg = atan2f(-gx, sqrtf(gy * gy + gz * gz)) * (180.0f / 3.14159265f);
        }
    }
}

static void trigger_aw86224_haptic(void) {
    if (!g_dev_aw86224 || !g_p4.haptics_enabled) return;
    /* Trigger short RTP haptic pulse on AW86224 (0x58) */
    const uint8_t go_cmd[2] = {0x0CU, 0x01U};
    (void)i2c_write_bytes(g_dev_aw86224, go_cmd, 2U);
}

/* ---------------------------------------------------------------------------
 * 5. Serial CLI Parser & Status Output (UART0 /dev/ttyACM7)
 * --------------------------------------------------------------------------- */
static void print_p4_status(void) {
    const elrs_air_rate_config_t *rf = elrs_get_air_rate_by_index(g_p4.elrs_rate_idx);
    const elrs_fhss_domain_info_t *dinfo = elrs_fhss_get_domain_info(g_p4.elrs_domain);
    printf("\n=== LilyGo T-Display P4 ExpressLRS + Reticulum TRX Unit ===\n");
    printf("Panel          : %s (%ux%u Landscape)\n",
           (g_p4.panel_variant == P4_PANEL_RM69A10_AMOLED) ? "RM69A10 AMOLED" : "HI8561 TFT",
           (unsigned)g_p4.screen_w, (unsigned)g_p4.screen_h);
    printf("LR2021 Radio   : %s (FW=0x%04X, TCXO=3.0V, FEM=AT2401C, Ant=%s)\n",
           g_lr2021_ready ? "ONLINE" : "OFFLINE",
           (unsigned)g_lr2021_fw_ver,
           (g_p4.antenna_path == P4_ANTENNA_INTERNAL) ? "INTERNAL" : "MMCX1");
    printf("ELRS Mode/Rate : mode=%u rate=%u (%s) domain=%s pwr=%ddBm\n",
           (unsigned)g_p4.app_mode,
           (unsigned)g_p4.elrs_rate_idx,
           rf ? rf->name : "?",
           dinfo ? dinfo->domain_name : "?",
           (int)g_p4.tx_power_dbm);
    printf("Bind Phrase/UID: \"%s\" -> %02X:%02X:%02X:%02X:%02X:%02X\n",
           g_p4.bind_phrase,
           g_p4.uid[0], g_p4.uid[1], g_p4.uid[2], g_p4.uid[3], g_p4.uid[4], g_p4.uid[5]);
    printf("Packet Counters: tx=%lu rx=%lu sync=%lu lq=%u%% rssi=%ddBm\n",
           (unsigned long)g_p4.tx_pkt_count,
           (unsigned long)g_p4.rx_pkt_count,
           (unsigned long)g_p4.sync_pkt_count,
           (unsigned)g_p4.uplink_lq,
           (int)g_p4.uplink_rssi_dbm);
    printf("CRSF CH1..CH8  : [%u, %u, %u, %u, %u, %u, %u, %u] (Armed=%u, Gimbal=M%u)\n",
           (unsigned)g_p4.crsf_channels[0], (unsigned)g_p4.crsf_channels[1],
           (unsigned)g_p4.crsf_channels[2], (unsigned)g_p4.crsf_channels[3],
           (unsigned)g_p4.crsf_channels[4], (unsigned)g_p4.crsf_channels[5],
           (unsigned)g_p4.crsf_channels[6], (unsigned)g_p4.crsf_channels[7],
           g_p4.sw_armed ? 1U : 0U, (unsigned)g_p4.gimbal_mode);
    printf("RNS Identity   : dest=%02x%02x%02x%02x%02x%02x%02x%02x..%02x%02x peers=%u last=\"%s\"\n",
           g_p4.rns_id.dest_hash[0], g_p4.rns_id.dest_hash[1],
           g_p4.rns_id.dest_hash[2], g_p4.rns_id.dest_hash[3],
           g_p4.rns_id.dest_hash[4], g_p4.rns_id.dest_hash[5],
           g_p4.rns_id.dest_hash[6], g_p4.rns_id.dest_hash[7],
           g_p4.rns_id.dest_hash[14], g_p4.rns_id.dest_hash[15],
           (unsigned)g_p4.rns_peers.count,
           g_p4.last_swarm_action);
    printf("Sensors        : BAT=%umV(%u%%,%dmA) IMU(%s)=%.1f/%.1fdeg GPS(fix=%u,sats=%u,lat=%ld,lon=%ld)\n",
           (unsigned)g_p4.battery.voltage_mv, (unsigned)g_p4.battery.soc_pct, (int)g_p4.battery.current_ma,
           g_p4.imu.imu_present ? (g_p4.imu.is_icm20948 ? "ICM20948" : "QMI8658") : "NONE",
           (double)g_p4.imu.roll_deg, (double)g_p4.imu.pitch_deg,
           g_p4.gnss.fix_valid ? 1U : 0U, (unsigned)g_p4.gnss.satellites,
           (long)g_p4.gnss.lat_deg_e7, (long)g_p4.gnss.lon_deg_e7);
    printf("===========================================================\n");
}

static void handle_cli_line(char *line) {
    while (*line == ' ' || *line == '\t') line++;
    if (*line == '\0') return;

    xSemaphoreTake(g_p4_mutex, portMAX_DELAY);
    if (strcmp(line, "bootloader") == 0) {
        USB_SERIAL_JTAG.chip_rst.usb_uart_chip_rst_dis = 0;
        printf("[OK] USB_SERIAL_JTAG auto-reset re-enabled for bootloader\n");
    } else if (strcmp(line, "help") == 0) {
        printf("Commands: status | bootloader | configure rate=<0..21> domain=<eu868|fcc915|ism2g4> power=<dbm> ant=<int|mmcx1> mode=<1..4> | "
               "stick lx=<f> ly=<f> rx=<f> ry=<f> | arm <0|1> | page <0..3> | rns_pair | rns_form <0..3> | rns_multi | spectrum | dfu\n");
    } else if (strcmp(line, "status") == 0) {
        print_p4_status();
    } else if (strncmp(line, "configure", 9) == 0) {
        char *p = line + 9;
        char *tok = strtok(p, " ");
        while (tok) {
            if (strncmp(tok, "rate=", 5) == 0) {
                uint8_t r = (uint8_t)atoi(tok + 5);
                if (r < elrs_get_air_rate_count()) g_p4.elrs_rate_idx = r;
            } else if (strncmp(tok, "domain=", 7) == 0) {
                const char *d = tok + 7;
                if (strcasecmp(d, "eu868") == 0) g_p4.elrs_domain = ELRS_DOMAIN_EU868;
                else if (strcasecmp(d, "fcc915") == 0) g_p4.elrs_domain = ELRS_DOMAIN_FCC915;
                else if (strcasecmp(d, "au915") == 0) g_p4.elrs_domain = ELRS_DOMAIN_AU915;
                else if (strcasecmp(d, "ce_lbt") == 0 || strcasecmp(d, "ce2g4") == 0) g_p4.elrs_domain = ELRS_DOMAIN_CE2G4;
                else if (strcasecmp(d, "ism2g4") == 0) g_p4.elrs_domain = ELRS_DOMAIN_ISM2G4;
            } else if (strncmp(tok, "power=", 6) == 0) {
                g_p4.tx_power_dbm = (int8_t)atoi(tok + 6);
            } else if (strncmp(tok, "ant=", 4) == 0) {
                g_p4.antenna_path = (strcasecmp(tok + 4, "mmcx1") == 0) ? P4_ANTENNA_MMCX1 : P4_ANTENNA_INTERNAL;
            } else if (strncmp(tok, "mode=", 5) == 0) {
                g_p4.app_mode = (uint8_t)atoi(tok + 5);
            }
            tok = strtok(NULL, " ");
        }
        const elrs_air_rate_config_t *rf = elrs_get_air_rate_by_index(g_p4.elrs_rate_idx);
        if (rf) elrs_ota_init(&g_p4.ota, g_p4.uid, rf->payload_len, ELRS_SW_HYBRID_OR_16CH);
        elrs_fhss_init(&g_p4.fhss, g_p4.elrs_domain, g_p4.ota.uid_seed);
        g_rf_reconfig_needed = true;
        printf("[OK] Configured rate=%u domain=%u pwr=%ddBm ant=%s mode=%u\n",
               (unsigned)g_p4.elrs_rate_idx, (unsigned)g_p4.elrs_domain,
               (int)g_p4.tx_power_dbm, (g_p4.antenna_path == P4_ANTENNA_INTERNAL) ? "INT" : "MMCX1",
               (unsigned)g_p4.app_mode);
    } else if (strncmp(line, "stick", 5) == 0) {
        char *tok = strtok(line + 5, " ");
        while (tok) {
            if (strncmp(tok, "lx=", 3) == 0) g_p4.left_x = atof(tok + 3);
            else if (strncmp(tok, "ly=", 3) == 0) g_p4.left_y = atof(tok + 3);
            else if (strncmp(tok, "rx=", 3) == 0) g_p4.right_x = atof(tok + 3);
            else if (strncmp(tok, "ry=", 3) == 0) g_p4.right_y = atof(tok + 3);
            tok = strtok(NULL, " ");
        }
        p4_trx_update_crsf_channels(&g_p4);
        printf("[OK] Sticks updated -> CH1=%u CH2=%u CH3=%u CH4=%u\n",
               (unsigned)g_p4.crsf_channels[0], (unsigned)g_p4.crsf_channels[1],
               (unsigned)g_p4.crsf_channels[2], (unsigned)g_p4.crsf_channels[3]);
    } else if (strncmp(line, "arm", 3) == 0) {
        g_p4.sw_armed = (atoi(line + 3) != 0);
        p4_trx_update_crsf_channels(&g_p4);
        printf("[OK] Armed=%u (CH5=%u)\n", g_p4.sw_armed ? 1U : 0U, (unsigned)g_p4.crsf_channels[4]);
    } else if (strncmp(line, "page", 4) == 0) {
        uint8_t pg = (uint8_t)atoi(line + 4);
        if (pg < P4_PAGE_COUNT) g_p4.active_page = (p4_ui_page_t)pg;
        printf("[OK] Active UI Page = %u\n", (unsigned)g_p4.active_page);
    } else if (strncmp(line, "rns_form", 8) == 0) {
        if (line[8] != '\0') g_p4.sw_formation = (uint8_t)(atoi(line + 8) & 3);
        g_p4.pending_rns_len = p4_trx_build_follow_me_formation_pkt(&g_p4, g_p4.pending_rns_pkt);
        (void)elrs_tunnel_enqueue_tx(&g_p4.tunnel, g_p4.pending_rns_pkt, (uint16_t)g_p4.pending_rns_len, g_p4.ota.is_full_res);
        if (g_p4.app_mode == 3U && g_lr2021_ready) {
            p4_lr2021_transmit_frame(g_p4.pending_rns_pkt, (uint8_t)g_p4.pending_rns_len, false);
        }
        snprintf(g_p4.last_swarm_action, sizeof(g_p4.last_swarm_action), "CLI RNSF FORM=%u #%u",
                 (unsigned)g_p4.sw_formation, (unsigned)g_p4.formation_seq);
        printf("[OK] Queued 64B RNSF Formation #%u (shape=%u)\n",
               (unsigned)g_p4.formation_seq, (unsigned)g_p4.sw_formation);
    } else if (strncmp(line, "rns_multi", 9) == 0) {
        g_p4.pending_rns_len = p4_trx_build_default_swarm_bundle_pkt(&g_p4, g_p4.pending_rns_pkt);
        (void)elrs_tunnel_enqueue_tx(&g_p4.tunnel, g_p4.pending_rns_pkt, (uint16_t)g_p4.pending_rns_len, g_p4.ota.is_full_res);
        if (g_p4.app_mode == 3U && g_lr2021_ready) {
            p4_lr2021_transmit_frame(g_p4.pending_rns_pkt, (uint8_t)g_p4.pending_rns_len, false);
        }
        snprintf(g_p4.last_swarm_action, sizeof(g_p4.last_swarm_action), "CLI RNSM 4-DRONE #%u", (unsigned)g_p4.multi_seq);
        printf("[OK] Queued 64B RNSM 4-Drone Bundle #%u\n", (unsigned)g_p4.multi_seq);
    } else if (strcmp(line, "dfu") == 0) {
        g_p4.dfu_requested = true;
    } else {
        printf("[?] Unknown command: %s (type 'help')\n", line);
    }
    xSemaphoreGive(g_p4_mutex);
}

/* ---------------------------------------------------------------------------
 * 6. Hardware Timer (esp_timer on Core 0) + UI/Touch/Sensors/CLI Task (Core 1)
 * --------------------------------------------------------------------------- */
static esp_timer_handle_t g_rf_timer = NULL;
static uint32_t           g_rf_timer_period_us = 1000UL;
static uint32_t           g_last_rf_freq_hz    = 0UL;
static volatile bool      g_ui_dirty           = true;
static bool               g_disp_ok            = false;

static void rf_timer_cb(void *arg) {
    (void)arg;
    if (!g_lr2021_ready) return;

    if (g_rf_reconfig_needed) {
        p4_lr2021_configure_active_mode();
        g_rf_reconfig_needed = false;
        g_last_rf_freq_hz    = 0UL;
        const elrs_air_rate_config_t *rf_new = elrs_get_air_rate_by_index(g_p4.elrs_rate_idx);
        uint32_t new_per = rf_new ? rf_new->interval_us : 4000UL;
        if (g_rf_timer && new_per != g_rf_timer_period_us) {
            g_rf_timer_period_us = new_per;
            (void)esp_timer_restart(g_rf_timer, g_rf_timer_period_us);
        }
    }

    uint8_t mode = g_p4.app_mode;
    const elrs_air_rate_config_t *rf = elrs_get_air_rate_by_index(g_p4.elrs_rate_idx);
    if (mode == 2U && rf) {
        uint32_t freq_hz = 0UL;
        uint8_t ota_pkt[16] = {0};
        size_t plen = p4_trx_step_elrs_tx_frame(&g_p4, &freq_hz, ota_pkt);
        if (freq_hz != g_last_rf_freq_hz) {
            p4_lr2021_set_rf_freq(freq_hz);
            g_last_rf_freq_hz = freq_hz;
        }
        bool is_fsk_fec = (rf->phy_type == ELRS_PHY_FSK_2G4 && plen == 8U);
        p4_lr2021_transmit_frame(ota_pkt, (uint8_t)plen, is_fsk_fec);
    }
}

static void p4_ui_task(void *arg) {
    (void)arg;
    char cli_buf[160];
    size_t cli_pos = 0U;
    uint32_t loop_tick = 0U;
    uint32_t last_fb_cnt = 0U;

    while (1) {
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

        /* Poll Multi-Touch & Update Virtual Gimbals / Switches */
        p4_touch_frame_t tf;
        poll_multitouch(&tf);
        if (tf.count > 0U) {
            g_ui_dirty = true;
        }

        xSemaphoreTake(g_p4_mutex, portMAX_DELAY);
        if ((loop_tick % 2U) == 0U) {
            init_and_poll_imu(false);
            if (g_p4.sw_imu_tilt_steer) {
                g_ui_dirty = true;
            }
        }
        if ((loop_tick % 50U) == 0U) {
            poll_bq27220_battery();
            g_ui_dirty = true;
        }
        if (tf.count > 0U || g_p4.left_touch_id >= 0 || g_p4.right_touch_id >= 0) {
            p4_trx_process_touch(&g_p4, &tf, now_ms);
        }

        if (g_p4.feedback_count != last_fb_cnt) {
            last_fb_cnt = g_p4.feedback_count;
            trigger_aw86224_haptic();
            g_ui_dirty = true;
        }

        if (g_p4.dfu_requested) {
            printf("[DFU] Rebooting ESP32-P4 into Bootloader mode...\n");
            vTaskDelay(pdMS_TO_TICKS(50));
            esp_restart();
        }

        /* Render Landscape HUD & Blit to MIPI-DSI Panel on Core 1 when dirty or every ~1s */
        bool do_render = g_ui_dirty || ((loop_tick % 33U) == 0U);
        if (do_render && g_disp_ok && g_fb_landscape && g_fb_portrait && g_dpi_panel) {
            g_ui_dirty = false;
            p4_trx_render_landscape_rgb565(&g_p4, g_fb_landscape);
            xSemaphoreGive(g_p4_mutex);
            p4_trx_blit_landscape_to_portrait_rgb565(g_fb_landscape, g_fb_portrait, g_p4.screen_w, g_p4.screen_h);
            (void)esp_lcd_panel_draw_bitmap(g_dpi_panel, 0, 0, g_p4.screen_h, g_p4.screen_w, g_fb_portrait);
        } else {
            xSemaphoreGive(g_p4_mutex);
        }

        /* Drain L76K GNSS UART1 NMEA stream */
        uint8_t gps_rx[64];
        int glen = uart_read_bytes(UART_NUM_1, gps_rx, sizeof(gps_rx), 0);
        if (glen > 0) {
            xSemaphoreTake(g_p4_mutex, portMAX_DELAY);
            for (int i = 0; i < glen; i++) {
                if (p4_trx_gnss_feed_byte(&g_p4.gnss, (char)gps_rx[i])) {
                    g_ui_dirty = true;
                }
            }
            xSemaphoreGive(g_p4_mutex);
        }

        /* Poll non-blocking stdin for CLI commands over UART0 (/dev/ttyACM7) */
        int ch;
        while ((ch = fgetc(stdin)) != EOF) {
            if (ch == '\r' || ch == '\n') {
                if (cli_pos > 0U) {
                    cli_buf[cli_pos] = '\0';
                    cli_pos = 0U;
                    handle_cli_line(cli_buf);
                    g_ui_dirty = true;
                }
            } else if (cli_pos + 1U < sizeof(cli_buf)) {
                cli_buf[cli_pos++] = (char)ch;
            }
        }

        loop_tick++;
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

extern "C" void app_main(void) {
    /* Prevent host serial terminal DTR/RTS close transitions from resetting ESP32-P4 into USB_BOOT */
    USB_SERIAL_JTAG.chip_rst.usb_uart_chip_rst_dis = 1;
    printf("\n[BOOT] LilyGo T-Display P4 ExpressLRS + Reticulum Drone TRX Unit starting...\n");

    g_p4_mutex = xSemaphoreCreateMutex();

    /* 1. Enable ESP32-P4 On-Chip LDOs: LDO3 = 2500mV (MIPI DPHY), LDO4 = 3300mV */
    esp_ldo_channel_handle_t ldo3 = NULL, ldo4 = NULL;
    esp_ldo_channel_config_t ldo3_cfg = {.chan_id = 3, .voltage_mv = 2500};
    esp_ldo_channel_config_t ldo4_cfg = {.chan_id = 4, .voltage_mv = 3300};
    (void)esp_ldo_acquire_channel(&ldo3_cfg, &ldo3);
    (void)esp_ldo_acquire_channel(&ldo4_cfg, &ldo4);

    /* 2. Initialize I2C1 (GPIO7/8) and I2C2 (GPIO20/21) */
    i2c_master_bus_config_t bus1_cfg = {};
    bus1_cfg.i2c_port          = I2C_NUM_0;
    bus1_cfg.sda_io_num        = (gpio_num_t)P4_I2C1_SDA_PIN;
    bus1_cfg.scl_io_num        = (gpio_num_t)P4_I2C1_SCL_PIN;
    bus1_cfg.clk_source        = I2C_CLK_SRC_DEFAULT;
    bus1_cfg.glitch_ignore_cnt = 7;
    bus1_cfg.flags.enable_internal_pullup = true;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus1_cfg, &g_i2c1_bus));

    i2c_master_bus_config_t bus2_cfg = {};
    bus2_cfg.i2c_port          = I2C_NUM_1;
    bus2_cfg.sda_io_num        = (gpio_num_t)P4_I2C2_SDA_PIN;
    bus2_cfg.scl_io_num        = (gpio_num_t)P4_I2C2_SCL_PIN;
    bus2_cfg.clk_source        = I2C_CLK_SRC_DEFAULT;
    bus2_cfg.glitch_ignore_cnt = 7;
    bus2_cfg.flags.enable_internal_pullup = true;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus2_cfg, &g_i2c2_bus));

    i2c_device_config_t xl_cfg = {.dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = P4_XL9535_I2C_ADDR, .scl_speed_hz = 400000};
    ESP_ERROR_CHECK(i2c_master_bus_add_device(g_i2c1_bus, &xl_cfg, &g_dev_xl9535));

    if (i2c_master_probe(g_i2c1_bus, P4_BQ27220_I2C_ADDR, 30) == ESP_OK) {
        i2c_device_config_t bq_cfg = {.dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = P4_BQ27220_I2C_ADDR, .scl_speed_hz = 400000};
        (void)i2c_master_bus_add_device(g_i2c1_bus, &bq_cfg, &g_dev_bq27220);
    }
    if (i2c_master_probe(g_i2c2_bus, P4_SGM38121_I2C_ADDR, 30) == ESP_OK) {
        i2c_device_config_t sg_cfg = {.dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = P4_SGM38121_I2C_ADDR, .scl_speed_hz = 400000};
        (void)i2c_master_bus_add_device(g_i2c2_bus, &sg_cfg, &g_dev_sgm38121);
    }
    if (i2c_master_probe(g_i2c2_bus, P4_AW86224_I2C_ADDR, 30) == ESP_OK) {
        i2c_device_config_t aw_cfg = {.dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = P4_AW86224_I2C_ADDR, .scl_speed_hz = 400000};
        (void)i2c_master_bus_add_device(g_i2c2_bus, &aw_cfg, &g_dev_aw86224);
    }

    /* 3. Read ESP32-P4 Factory MAC to seed deterministic Reticulum 16B SINGLE Identity */
    uint8_t mac_id[8] = {0x30, 0xED, 0xA0, 0xE1, 0xC3, 0x58, 0x50, 0x34};
    (void)esp_efuse_mac_get_default(mac_id);
    p4_trx_init(&g_p4, P4_PANEL_HI8561_TFT, "WinniePico", mac_id);

    /* 4. Bring up XL9535 power rails and resets */
    (void)init_xl9535_and_power_rails();

    /* 5. Auto-detect and initialize MIPI-DSI Display & Multi-Touch */
    p4_panel_variant_t detected_panel = P4_PANEL_HI8561_TFT;
    g_disp_ok = init_mipi_dsi_and_touch(&detected_panel);
    if (detected_panel != g_p4.panel_variant) {
        p4_trx_init(&g_p4, detected_panel, "WinniePico", mac_id);
    }

    size_t fb_bytes = (size_t)g_p4.screen_w * g_p4.screen_h * sizeof(uint16_t);
    g_fb_landscape = (uint16_t *)heap_caps_malloc(fb_bytes, MALLOC_CAP_SPIRAM);
    g_fb_portrait  = (uint16_t *)heap_caps_malloc(fb_bytes, MALLOC_CAP_SPIRAM);

    if (g_disp_ok && g_fb_landscape && g_fb_portrait && g_dpi_panel) {
        p4_trx_render_landscape_rgb565(&g_p4, g_fb_landscape);
        p4_trx_blit_landscape_to_portrait_rgb565(g_fb_landscape, g_fb_portrait, g_p4.screen_w, g_p4.screen_h);
        (void)esp_lcd_panel_draw_bitmap(g_dpi_panel, 0, 0, g_p4.screen_h, g_p4.screen_w, g_fb_portrait);
        ESP_LOGI(TAG, "Landscape RGB565 HUD rendered to MIPI-DSI panel (%ux%u)",
                 (unsigned)g_p4.screen_w, (unsigned)g_p4.screen_h);
    }

    /* 6. Initialize SPI2 & Onboard Semtech LR2021 Radio */
    gpio_config_t cs_cfg = {.pin_bit_mask = (1ULL << P4_LR2021_CS_PIN), .mode = GPIO_MODE_OUTPUT};
    gpio_config(&cs_cfg);
    gpio_set_level((gpio_num_t)P4_LR2021_CS_PIN, 1);

    gpio_config_t busy_cfg = {.pin_bit_mask = (1ULL << P4_LR2021_BUSY_PIN), .mode = GPIO_MODE_INPUT};
    gpio_config(&busy_cfg);

    spi_bus_config_t spi_cfg = {};
    spi_cfg.mosi_io_num     = P4_LR2021_MOSI_PIN;
    spi_cfg.miso_io_num     = P4_LR2021_MISO_PIN;
    spi_cfg.sclk_io_num     = P4_LR2021_SCLK_PIN;
    spi_cfg.quadwp_io_num   = -1;
    spi_cfg.quadhd_io_num   = -1;
    spi_cfg.max_transfer_sz = 64;
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &spi_cfg, SPI_DMA_DISABLED));

    spi_device_interface_config_t dev_cfg = {};
    dev_cfg.clock_speed_hz = 10 * 1000 * 1000; /* 10 MHz SPI */
    dev_cfg.mode           = 0;
    dev_cfg.spics_io_num   = -1; /* Manual CS on GPIO24 */
    dev_cfg.queue_size     = 4;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_cfg, &g_lr2021_spi));

    (void)p4_lr2021_init();

    /* 7. Initialize Onboard IMU (ICM20948 / QMI8658) & L76K GNSS UART1 */
    init_and_poll_imu(true);
    poll_bq27220_battery();

    uart_config_t gps_uart_cfg = {};
    gps_uart_cfg.baud_rate  = 9600;
    gps_uart_cfg.data_bits  = UART_DATA_8_BITS;
    gps_uart_cfg.parity     = UART_PARITY_DISABLE;
    gps_uart_cfg.stop_bits  = UART_STOP_BITS_1;
    gps_uart_cfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
    gps_uart_cfg.source_clk = UART_SCLK_DEFAULT;
    if (uart_driver_install(UART_NUM_1, 1024, 0, 0, NULL, 0) == ESP_OK) {
        (void)uart_param_config(UART_NUM_1, &gps_uart_cfg);
        (void)uart_set_pin(UART_NUM_1, P4_GPS_UART_TX_PIN, P4_GPS_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }

    print_p4_status();

    /* 8. Launch Microsecond-Accurate Hardware Timer (esp_timer) for ExpressLRS RF TX/RX */
    const elrs_air_rate_config_t *init_rf = elrs_get_air_rate_by_index(g_p4.elrs_rate_idx);
    g_rf_timer_period_us = init_rf ? init_rf->interval_us : 1000UL;
    esp_timer_create_args_t rf_tmr_args = {};
    rf_tmr_args.callback        = &rf_timer_cb;
    rf_tmr_args.name            = "elrs_rf";
    rf_tmr_args.dispatch_method = ESP_TIMER_TASK;
    ESP_ERROR_CHECK(esp_timer_create(&rf_tmr_args, &g_rf_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(g_rf_timer, g_rf_timer_period_us));
    ESP_LOGI(TAG, "Started esp_timer ELRS RF loop at %lu us interval", (unsigned long)g_rf_timer_period_us);

    /* 9. Launch UI, Touch, Sensor & CLI Task Pinned to Core 1 (leaving Core 0 for esp_timer RF) */
    xTaskCreatePinnedToCore(p4_ui_task, "p4_ui", 16384, NULL, 5, NULL, 1);
}
