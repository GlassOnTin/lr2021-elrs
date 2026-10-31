# lr2021-elrs

Bare-metal ExpressLRS-compatible transceiver, passive OTA monitor, and RF bridge firmware for the Seeed Studio Semtech LR2021 LoRa Plus Evaluation Kit (`Wio-LR2021` + `Seeed Studio XIAO nRF54L15`).

## License

Licensed under the GNU Affero General Public License v3.0 or later (`SPDX-License-Identifier: AGPL-3.0-or-later`). See [LICENSE](LICENSE).

Portions of the radio register map, PRAM patch table, and ExpressLRS OTA/FHSS/CRSF protocol implementation are adapted from:
- ExpressLRS (`GPL-3.0-or-later`, `src/lib/LR2021Driver/` and `src/lib/FHSS/`)
- Semtech Corporation `lr20xx_driver` (`BSD-3-Clause`)

## Hardware Target

- **MCU Board:** Seeed Studio XIAO nRF54L15 (`nRF54L15-QFAA`, ARM Cortex-M33 @ 128 MHz, 1.5 MB RRAM, 256 KB SRAM)
- **Radio Shield:** Seeed Studio Wio-LR2021 (`LR2021EVK1XBS1_XIAO`, Semtech LR2021 rev B silicon `0x0118`)
- **Oscillator:** Passive 32 MHz crystal (`has_tcxo = false`). Do not issue `SET_TCXO_MODE` (`0x0120`) on this board; doing so disables the passive crystal oscillator and causes `HF_XOSC_START_ERR` (`0x0001`).
- **RF Input Rating:** Maximum +10 dBm at the SMA ports (`ANT_LF` and `ANT_HF`). Always attach antennas or a 20-30 dB attenuator before transmitting on the bench.

### Pin Mapping (`Wio-LR2021` <-> `XIAO nRF54L15`)

| Function | XIAO Pin | nRF54L15 GPIO | Notes |
|---|---|---|---|
| `SPI_SCK` | `D8` | `P2.01` | `SPIM21` @ 8 MHz |
| `SPI_MISO` | `D9` | `P2.04` | `SPIM21` |
| `SPI_MOSI` | `D10` | `P2.02` | `SPIM21` |
| `RADIO_NSS` | `D3` | `P1.07` | Active-low chip select |
| `RADIO_BUSY` | `D2` | `P1.06` | Active-high busy input |
| `RADIO_RST` | `D1` | `P1.05` | Active-low reset |
| `RADIO_IRQ` (`DIO8`) | `D0` | `P1.04` | `GPIOTE20` rising-edge interrupt |
| `DIO5` (LF RF SW) | Radio `DIO5` | Internal | `0x03` (RX + TX on Sub-GHz) |
| `DIO6` (HF RF SW) | Radio `DIO6` | Internal | `0x01` (RX on 2.4 GHz) |
| `DIO7` (HF RF SW) | Radio `DIO7` | Internal | `0x02` (TX on 2.4 GHz) |
| `USB_CDC_TX` | SAMD11 bridge | `P1.09` | `UARTE20` @ 115200 8N1 |
| `USB_CDC_RX` | SAMD11 bridge | `P1.08` | `UARTE20` @ 115200 8N1 |
| `CRSF_TX` | `D6` | `P2.08` | `UARTE21` @ 420000 8N1 |
| `CRSF_RX` | `D7` | `P2.07` | `UARTE21` @ 420000 8N1 |
| `USER_LED` | Built-in | `P2.00` | Active-high status LED |

## Supported Modes & Air Rates

The firmware supports 5 operating modes (`IDLE`, `TX`, `RX`, `MONITOR`, `BRIDGE`) and 17 air rate configurations across 5 regulatory domains (`FCC915`, `AU915`, `EU868`, `ISM2G4`, `CE2G4`):

- **Sub-GHz (868/915 MHz):** `900_25HZ` (LoRa SF9 BW500), `900_50HZ` (SF8 BW500), `900_100HZ` (SF7 BW500), `900_200HZ` (SF6 BW500), `900_200HZ_8CH` (SF6 BW500), `900_250HZ` (SF5 BW500), `900_1000HZ` (GFSK 400 kbps + Hamming(7,4) FEC)
- **2.4 GHz ISM:** `2G4_50HZ` (LoRa SF9 BW800), `2G4_100HZ_8CH` (SF8 BW800), `2G4_150HZ` (SF7 BW800), `2G4_250HZ` (SF6 BW800), `2G4_333HZ_8CH` (SF5 BW800), `2G4_500HZ` (SF5 BW800), `2G4_D250` (FLRC 650 kbps), `2G4_D500` (FLRC 650 kbps), `2G4_F500` (FLRC 650 kbps), `2G4_F1000` (FLRC 1.3 Mbps)
- **Dual-Band:** `DUAL_150HZ` (Sub-GHz SF7 BW500), `DUAL_500HZ` (2.4 GHz FLRC)

## Build & Flash

Requirements:
- `arm-none-eabi-gcc` / `arm-none-eabi-g++`
- `cmake`
- `openocd` (0.12.0+ with `cmsis-dap` support)

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
./scripts/flash.sh FC6D1A37
```

## Host CLI & Bench Power Control

`tools/lr2021_cli.py` provides command-line control over the USB CDC serial console and the bench `DSD Tech SH-UR04A` 4-channel USB power relay (`/dev/ttyUSB0`):

```bash
# Query board status
./tools/lr2021_cli.py -p /dev/ttyACM7 status

# List all 17 air rates
./tools/lr2021_cli.py -p /dev/ttyACM7 rates

# Run active RF ping test on 915 MHz or 2.44 GHz
./tools/lr2021_cli.py -p /dev/ttyACM7 ping --freq 915000000 --mod lora --count 5
./tools/lr2021_cli.py -p /dev/ttyACM7 ping --freq 2440000000 --mod flrc --count 5

# Run passive OTA monitor
./tools/lr2021_cli.py -p /dev/ttyACM7 configure --domain fcc915 --rate 900_200HZ --phrase "my-phrase" --mode monitor

# Control bench drone power via DSD Tech SH-UR04A relay (channel 1)
./tools/lr2021_cli.py relay --relay-port /dev/ttyUSB0 --ch 1 --state status
./tools/lr2021_cli.py relay --relay-port /dev/ttyUSB0 --ch 1 --state cycle
```
