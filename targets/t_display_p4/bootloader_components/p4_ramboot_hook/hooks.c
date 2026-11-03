/* SPDX-License-Identifier: AGPL-3.0-or-later */
#include "esp_rom_spiflash.h"
#include "esp_image_format.h"
#include "bootloader_flash_config.h"
#include "rom/cache.h"
#include "soc/usb_serial_jtag_struct.h"
#include "esp_log.h"

void bootloader_hooks_include(void) {
}

extern void bootloader_configure_spi_pins(int drv);

void bootloader_before_init(void) {
    USB_SERIAL_JTAG.chip_rst.usb_uart_chip_rst_dis = 1;
    ROM_Boot_Cache_Init();
    esp_rom_spiflash_attach(0, false);
    bootloader_configure_spi_pins(1);
    esp_rom_spiflash_config_readmode(ESP_ROM_SPIFLASH_DIO_MODE);
    esp_image_header_t hdr = {
        .spi_mode = ESP_IMAGE_SPI_MODE_DIO,
        .spi_speed = ESP_IMAGE_SPI_SPEED_DIV_2,
        .spi_size = ESP_IMAGE_FLASH_SIZE_16MB,
    };
    bootloader_flash_clock_config(&hdr);
    bootloader_flash_cs_timing_config();
}

void bootloader_after_init(void) {
    ESP_EARLY_LOGI("hook", "bootloader_after_init reached!");
}
