/* SPDX-License-Identifier: AGPL-3.0-or-later */
/*
 * RadioMaster XR4 (ESP32-PICO-D4 + 2x Semtech LR1121) Target Firmware
 *
 * Simultaneous Dual-Radio Architecture:
 *  - Radio 1 (LR1121 #1, NSS=27, DIO1=37): Full-time 2.4 GHz / Sub-GHz ExpressLRS
 *  - Radio 2 (LR1121 #2, NSS=13, DIO1=34): Full-time 868 MHz Reticulum LoRa Mesh
 *  - UART0 (RX=3, TX=1 @ 420000 baud): CRSF RC + Telemetry + Bootloader Passthrough
 *  - UART1 (RX=5, TX=18 @ 115200 baud): Auxiliary X1/X2 MAVLink v2 + RNode CLI
 */
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)

#include <Arduino.h>
#include <SPI.h>
#include "soc/rtc_cntl_reg.h"
#include "../radio/lr1121_driver.h"
#include "../crsf/crsf.h"
#include "../elrs/elrs_protocol.h"
#include "../rnode/rns_node.h"

static SPIClass         g_spi(VSPI);
static HardwareSerial   g_fc_crsf(0); /* UART0: GPIO3 RX, GPIO1 TX */
static HardwareSerial   g_aux_uart(1); /* UART1: GPIO5 RX, GPIO18 TX */
static hw_timer_t      *g_elrs_timer = NULL;

static xr4_dual_radio_t g_xr4;
static rns_mav_parser_t g_mav_parser;
static volatile bool    g_radio1_irq_flag = false;
static volatile bool    g_radio2_irq_flag = false;
static volatile bool    g_timer_tock_flag = false;

static inline int nss_pin_for(lr1121_radio_idx_t r) {
    return (r == LR1121_RADIO_1) ? XR4_PIN_RADIO1_NSS : XR4_PIN_RADIO2_NSS;
}

static inline int rst_pin_for(lr1121_radio_idx_t r) {
    return (r == LR1121_RADIO_1) ? XR4_PIN_RADIO1_RST : XR4_PIN_RADIO2_RST;
}

static inline int busy_pin_for(lr1121_radio_idx_t r) {
    return (r == LR1121_RADIO_1) ? XR4_PIN_RADIO1_BUSY : XR4_PIN_RADIO2_BUSY;
}

static bool esp32_lr1121_wait_busy(lr1121_radio_idx_t radio, uint32_t timeout_us) {
    int pin = busy_pin_for(radio);
    uint32_t t0 = micros();
    while (digitalRead(pin) == HIGH) {
        if ((uint32_t)(micros() - t0) > timeout_us) {
            return false;
        }
    }
    return true;
}

static void esp32_lr1121_reset(lr1121_radio_idx_t radio) {
    int pin = rst_pin_for(radio);
    digitalWrite(pin, LOW);
    delay(2);
    digitalWrite(pin, HIGH);
    delay(20);
    (void)esp32_lr1121_wait_busy(radio, 50000UL);
}

static void esp32_lr1121_write_cmd(lr1121_radio_idx_t radio, uint16_t opcode, const uint8_t *params, uint16_t len) {
    int nss = nss_pin_for(radio);
    (void)esp32_lr1121_wait_busy(radio, 20000UL);
    digitalWrite(nss, LOW);
    g_spi.transfer((uint8_t)(opcode >> 8U));
    g_spi.transfer((uint8_t)(opcode & 0xFFU));
    for (uint16_t i = 0; i < len; i++) {
        g_spi.transfer(params[i]);
    }
    digitalWrite(nss, HIGH);
}

static void esp32_lr1121_read_cmd(lr1121_radio_idx_t radio,
                                  uint16_t opcode,
                                  const uint8_t *args,
                                  uint16_t arg_len,
                                  uint8_t *out,
                                  uint16_t out_len) {
    int nss = nss_pin_for(radio);
    (void)esp32_lr1121_wait_busy(radio, 20000UL);
    digitalWrite(nss, LOW);
    g_spi.transfer((uint8_t)(opcode >> 8U));
    g_spi.transfer((uint8_t)(opcode & 0xFFU));
    for (uint16_t i = 0; i < arg_len; i++) {
        g_spi.transfer(args[i]);
    }
    digitalWrite(nss, HIGH);

    (void)esp32_lr1121_wait_busy(radio, 20000UL);
    digitalWrite(nss, LOW);
    for (uint16_t i = 0; i < out_len; i++) {
        out[i] = g_spi.transfer(0x00U);
    }
    digitalWrite(nss, HIGH);
}

static const lr1121_hal_ops_t k_esp32_hal_ops = {
    .reset_radio = esp32_lr1121_reset,
    .wait_busy   = esp32_lr1121_wait_busy,
    .write_cmd   = esp32_lr1121_write_cmd,
    .read_cmd    = esp32_lr1121_read_cmd,
};

static void IRAM_ATTR on_radio1_dio1_isr(void) {
    g_radio1_irq_flag = true;
}

static void IRAM_ATTR on_radio2_dio1_isr(void) {
    g_radio2_irq_flag = true;
}

static void IRAM_ATTR on_elrs_timer_isr(void) {
    g_timer_tock_flag = true;
}

/* Reboot into ESP32 bootloader / restart cleanly on CRSF bootloader frame */
static void reboot_into_esp32_uart_bootloader(void) {
    g_fc_crsf.println("RM_XR4_RNS_BL");
    g_fc_crsf.flush();
    delay(50);
    esp_restart();
}

/* Inspect incoming UART0 CRSF stream for ExpressLRS bootloader entry frame:
 * [0xC8/0xEC, 0x04, 0x32, 'b', 'l', 0x0A]
 */
static void poll_fc_crsf_rx(void) {
    static uint8_t rx_ring[8];
    static uint8_t rx_pos = 0U;
    while (g_fc_crsf.available() > 0) {
        uint8_t b = (uint8_t)g_fc_crsf.read();
        rx_ring[rx_pos & 7U] = b;
        rx_pos++;
        if (rx_pos >= 6U) {
            uint8_t s0 = rx_ring[(rx_pos - 6U) & 7U];
            uint8_t s1 = rx_ring[(rx_pos - 5U) & 7U];
            uint8_t s2 = rx_ring[(rx_pos - 4U) & 7U];
            uint8_t s3 = rx_ring[(rx_pos - 3U) & 7U];
            uint8_t s4 = rx_ring[(rx_pos - 2U) & 7U];
            if ((s0 == 0xC8U || s0 == 0xECU) && s1 == 0x04U && s2 == 0x32U && s3 == 'b' && s4 == 'l') {
                reboot_into_esp32_uart_bootloader();
            }
        }
    }
}

static void poll_aux_uart_rx(uint32_t now_ms) {
    static char    line_buf[96];
    static uint8_t line_pos = 0U;
    while (g_aux_uart.available() > 0) {
        uint8_t b = (uint8_t)g_aux_uart.read();
        (void)rns_mav_parse_byte(&g_mav_parser, b, &g_xr4.fc_tlm);
        if (b == '\r' || b == '\n') {
            if (line_pos > 0U) {
                line_buf[line_pos] = '\0';
                if (strcmp(line_buf, "status") == 0 || strcmp(line_buf, "rns") == 0) {
                    g_aux_uart.printf(
                        "XR4_STATUS r1_locked=%u r1_rx=%lu r1_crc_err=%lu r1_tlm_tx=%lu "
                        "r2_freq=%lu r2_rx=%lu r2_tx=%lu bridge=%lu slot=%u vbatt=%umV\r\n",
                        g_xr4.elrs_locked ? 1U : 0U,
                        (unsigned long)g_xr4.elrs_rx_ok_cnt,
                        (unsigned long)g_xr4.elrs_crc_err_cnt,
                        (unsigned long)g_xr4.elrs_tlm_tx_cnt,
                        (unsigned long)g_xr4.mesh_freq_hz,
                        (unsigned long)g_xr4.mesh_rx_cnt,
                        (unsigned long)g_xr4.mesh_tx_cnt,
                        (unsigned long)g_xr4.mesh_bridge_cnt,
                        (unsigned)g_xr4.drone_slot_id,
                        (unsigned)g_xr4.fc_tlm.vbatt_mv);
                } else if (strncmp(line_buf, "rns_slot ", 9) == 0) {
                    int sl = atoi(&line_buf[9]);
                    if (sl >= 1 && sl <= 4) {
                        g_xr4.drone_slot_id = (uint8_t)sl;
                        g_aux_uart.printf("OK RNS_SLOT=%u\r\n", (unsigned)g_xr4.drone_slot_id);
                    }
                } else if (strcmp(line_buf, "rns_ann") == 0) {
                    uint8_t ann_pkt[64];
                    size_t alen = rns_build_node_announce(&g_xr4.identity,
                                                          g_xr4.mesh_freq_hz,
                                                          g_xr4.mesh_bw_hz,
                                                          g_xr4.mesh_sf,
                                                          g_xr4.mesh_cr,
                                                          &g_xr4.fc_tlm,
                                                          ann_pkt,
                                                          sizeof(ann_pkt));
                    if (alen > 0U) {
                        rns_relay_record_sent(&g_xr4.relay, ann_pkt, alen, now_ms);
                        lr1121_tx_nb(LR1121_RADIO_2, ann_pkt, (uint8_t)alen);
                        (void)elrs_tunnel_enqueue_tx(&g_xr4.elrs_tunnel, ann_pkt, (uint16_t)alen, false);
                        g_xr4.mesh_tx_cnt++;
                        g_aux_uart.println("OK XR4_RNS_ANN_TX");
                    }
                } else if (strcmp(line_buf, "dfu") == 0) {
                    reboot_into_esp32_uart_bootloader();
                }
                line_pos = 0U;
            }
        } else if (b >= 0x20U && b < 0x7FU && line_pos < (sizeof(line_buf) - 1U)) {
            line_buf[line_pos++] = (char)b;
        } else if (b < 0x20U || b >= 0x7FU) {
            line_pos = 0U;
        }
    }
}

void setup(void) {
    pinMode(XR4_PIN_RADIO1_NSS, OUTPUT);
    digitalWrite(XR4_PIN_RADIO1_NSS, HIGH);
    pinMode(XR4_PIN_RADIO2_NSS, OUTPUT);
    digitalWrite(XR4_PIN_RADIO2_NSS, HIGH);

    pinMode(XR4_PIN_RADIO1_RST, OUTPUT);
    digitalWrite(XR4_PIN_RADIO1_RST, HIGH);
    pinMode(XR4_PIN_RADIO2_RST, OUTPUT);
    digitalWrite(XR4_PIN_RADIO2_RST, HIGH);

    pinMode(XR4_PIN_RADIO1_BUSY, INPUT);
    pinMode(XR4_PIN_RADIO2_BUSY, INPUT);
    pinMode(XR4_PIN_RADIO1_DIO1, INPUT);
    pinMode(XR4_PIN_RADIO2_DIO1, INPUT);
    pinMode(XR4_PIN_BUTTON, INPUT_PULLUP);

    g_spi.begin(XR4_PIN_SPI_SCK, XR4_PIN_SPI_MISO, XR4_PIN_SPI_MOSI, -1);
    g_spi.setFrequency(16000000UL);
    g_spi.setDataMode(SPI_MODE0);
    g_spi.setBitOrder(MSBFIRST);

    g_fc_crsf.begin(420000UL, SERIAL_8N1, XR4_PIN_UART0_RX, XR4_PIN_UART0_TX);
    g_aux_uart.begin(115200UL, SERIAL_8N1, XR4_PIN_UART1_RX, XR4_PIN_UART1_TX);

    lr1121_set_hal_ops(&k_esp32_hal_ops);

    uint64_t efuse_mac = ESP.getEfuseMac();
    uint8_t hw_id[8];
    memcpy(hw_id, &efuse_mac, 8U);

    rns_mav_parser_init(&g_mav_parser);
    (void)xr4_dual_radio_init(&g_xr4,
                              "WinniePico",
                              ELRS_DOMAIN_CE2G4,
                              13U, /* 2G4_K1000 */
                              868000000UL,
                              500000UL,
                              7U,
                              LR1121_LORA_CR_4_5,
                              14,
                              1U,
                              hw_id);

    attachInterrupt(digitalPinToInterrupt(XR4_PIN_RADIO1_DIO1), on_radio1_dio1_isr, RISING);
    attachInterrupt(digitalPinToInterrupt(XR4_PIN_RADIO2_DIO1), on_radio2_dio1_isr, RISING);

    /* 1000 Hz FHSS timer for 2G4_K1000 on Radio 1 */
    g_elrs_timer = timerBegin(0, 80, true); /* 1 MHz tick */
    timerAttachInterrupt(g_elrs_timer, &on_elrs_timer_isr, true);
    timerAlarmWrite(g_elrs_timer, 1000UL, true); /* 1000 us = 1000 Hz */
    timerAlarmEnable(g_elrs_timer);

    g_aux_uart.println("READY RM_XR4_DUAL_LR1121 R1=ELRS_2G4_K1000 R2=RNS_868MHZ");
}

void loop(void) {
    uint32_t now_ms = millis();
    poll_fc_crsf_rx();
    poll_aux_uart_rx(now_ms);

    if (g_timer_tock_flag) {
        g_timer_tock_flag = false;
        xr4_dual_radio_elrs_tock(&g_xr4);
    }

    if (g_radio1_irq_flag) {
        g_radio1_irq_flag = false;
        uint32_t irq1 = lr1121_get_and_clear_irq(LR1121_RADIO_1);
        if (irq1 & LR1121_IRQ_RX_DONE) {
            uint8_t pkt8[8] = {0};
            lr1121_pkt_stats_t st = {0, 0, 0};
            if (lr1121_rx_read(LR1121_RADIO_1, pkt8, sizeof(pkt8), &st)) {
                uint8_t prev_mav_len = g_xr4.last_mavlink_len;
                g_xr4.last_mavlink_len = 0U;
                if (xr4_dual_radio_on_elrs_rx(&g_xr4, pkt8, st.rssi_dbm, st.snr_q2_db, now_ms)) {
                    if ((pkt8[0] & 0x03U) == ELRS_PKT_TYPE_RCDATA) {
                        uint8_t crsf_buf[26];
                        size_t flen = crsf_build_rc_channels_frame(g_xr4.rc_channels, crsf_buf);
                        g_fc_crsf.write(crsf_buf, flen);
                    }
                    if (g_xr4.last_mavlink_len > 0U) {
                        g_aux_uart.write(g_xr4.last_mavlink_frame, g_xr4.last_mavlink_len);
                    } else {
                        g_xr4.last_mavlink_len = prev_mav_len;
                    }
                }
            }
        }
        if (irq1 & LR1121_IRQ_TX_DONE) {
            lr1121_set_mode(LR1121_RADIO_1, LR1121_MODE_RX_CONT);
        }
    }

    if (g_radio2_irq_flag) {
        g_radio2_irq_flag = false;
        uint32_t irq2 = lr1121_get_and_clear_irq(LR1121_RADIO_2);
        if (irq2 & LR1121_IRQ_RX_DONE) {
            uint8_t lora_pkt[255] = {0};
            lr1121_pkt_stats_t st = {0, 0, 0};
            if (lr1121_rx_read(LR1121_RADIO_2, lora_pkt, sizeof(lora_pkt), &st) && st.raw_len > 0U) {
                g_xr4.last_mavlink_len = 0U;
                if (xr4_dual_radio_on_mesh_rx(&g_xr4, lora_pkt, st.raw_len, st.rssi_dbm, st.snr_q2_db, now_ms)) {
                    if (g_xr4.last_mavlink_len > 0U) {
                        g_aux_uart.write(g_xr4.last_mavlink_frame, g_xr4.last_mavlink_len);
                    }
                }
            }
        }
        if (irq2 & LR1121_IRQ_TX_DONE) {
            lr1121_set_mode(LR1121_RADIO_2, LR1121_MODE_RX_CONT);
        }
    }

    (void)xr4_dual_radio_service_tdma(&g_xr4, now_ms);
}

#endif /* ARDUINO_ARCH_ESP32 || ESP32 */
