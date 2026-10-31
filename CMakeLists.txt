# SPDX-License-Identifier: AGPL-3.0-or-later
cmake_minimum_required(VERSION 3.20)
project(lr2021_elrs LANGUAGES C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(MCU_FLAGS "-mcpu=cortex-m33 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mcmse")
set(COMMON_FLAGS "${MCU_FLAGS} -O2 -g3 -ffunction-sections -fdata-sections -Wall -Wextra -Wno-unused-parameter -DNRF54L15_XXAA -DNRF_APPLICATION -DNRF_SKIP_SAU_CONFIGURATION")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${COMMON_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COMMON_FLAGS} -fno-exceptions -fno-rtti")

set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/firmware/mcu/nrf54l15_xxaa.ld")
set(CMAKE_EXE_LINKER_FLAGS "${MCU_FLAGS} -T${LINKER_SCRIPT} -Wl,--gc-sections -specs=nano.specs -specs=nosys.specs -Wl,-Map=${CMAKE_BINARY_DIR}/lr2021_elrs.map")

add_executable(lr2021_elrs.elf
    firmware/mcu/startup_nrf54l15.c
    firmware/third_party/nordic_mdk/nrf54l/system_nrf54l.c
    firmware/hal/hal_nrf54l15.c
    firmware/radio/lr2021_driver.cpp
    firmware/elrs/elrs_protocol.cpp
    firmware/crsf/crsf.cpp
    firmware/src/main.cpp
)

target_include_directories(lr2021_elrs.elf PRIVATE
    firmware/third_party/nordic_mdk
    firmware/third_party/nordic_mdk/common
    firmware/third_party/nordic_mdk/nrf54l
    firmware/third_party/nordic_mdk/nrf54l/nrf54l15
    firmware/hal
    firmware/radio
    firmware/elrs
    firmware/crsf
)

add_custom_command(TARGET lr2021_elrs.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:lr2021_elrs.elf> ${CMAKE_BINARY_DIR}/lr2021_elrs.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:lr2021_elrs.elf> ${CMAKE_BINARY_DIR}/lr2021_elrs.hex
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:lr2021_elrs.elf>
    COMMENT "Generating binary/hex and printing firmware size"
)
