# lr2021-elrs

Bare-metal ExpressLRS v3/v4 transceiver (`ELRS_TX`, `ELRS_RX`), passive over-the-air packet monitor (`MONITOR`), and raw FLRC packet bridge (`BRIDGE`) firmware for the Seeed Studio Semtech LR2021 LoRa Plus Evaluation Kit (`Seeed Studio XIAO nRF54L15` + `Wio-LR2021`).

## Features

- **Dual-Band RF Support (Sub-GHz & 2.4 GHz):** Operates on Sub-GHz (`EU868` 863.275–869.575 MHz, `FCC915` 903.5–926.9 MHz, `AU915` 915.5–926.9 MHz) and 2.4 GHz (`CE_LBT` and `ISM2G4` 2400.4–2479.4 MHz) using the Semtech LR2021 (`0x0118`) transceiver.
- **23 ExpressLRS Air Rates:**
  - **2.4 GHz GFSK + Hamming(7,4) FEC:** `2G4_K1000` (1000 Hz, 300 kbps, 8B->14B FEC), `2G4_DK500`, `2G4_DK250`
  - **2.4 GHz FLRC:** `2G4_F1000` (1000 Hz, 650 kbps), `2G4_F500`, `2G4_D500`, `2G4_D250`
  - **2.4 GHz LoRa (800 kHz BW):** `2G4_500HZ` (SF5), `2G4_333HZ_8CH` (SF5 13B), `2G4_250HZ` (SF6), `2G4_150HZ` (SF7), `2G4_100HZ_8CH` (SF7 13B), `2G4_50HZ` (SF8)
  - **Sub-GHz GFSK + Hamming(7,4) FEC:** `900_K1000` (1000 Hz 8B), `900_K1000_8CH` (1000 Hz 13B)
  - **Sub-GHz LoRa (500 kHz BW):** `900_250HZ` (SF5), `900_200HZ_8CH` (SF5 13B), `900_200HZ` (SF6), `900_100HZ_8CH` (SF6 13B), `900_100HZ` (SF7), `900_50HZ` (SF8), `900_25HZ` (SF9), `900_D50HZ` (SF6 DVDA)
  - **Gemini X Dual-Band Legs:** `X150HZ_900`, `X150HZ_2G4`, `X100HZ_8CH_900`, `X100HZ_8CH_2G4`
- **Five Operating Modes:**
  1. `MONITOR`: Passive OTA sniffer that locks onto an active ExpressLRS link (via `SYNC` packets or fast sync-channel consecutive-nonce pair detection in `< 160 ms`) and follows the FHSS sequence without transmitting, logging both uplink (`SYNC`, `RCDATA`) and downlink (`LINKSTATS`, CRSF `DATA`) packets over USB CDC.
  2. `ELRS_RX`: Active ExpressLRS receiver that tracks the FHSS hop sequence, transmits downlink `LINKSTATS` telemetry on telemetry slots, and outputs 420,000-baud CRSF `RC_CHANNELS_PACKED` (`0x16`) and `LINK_STATISTICS` (`0x14`) frames on the Grove UART (`D6`/`D7`) for flight controllers (ArduPilot `SERIALx_PROTOCOL = 23`, Betaflight, INAV).
  3. `ELRS_TX`: Active ExpressLRS transmitter that generates the FHSS hop sequence, transmits `SYNC` and `RCDATA` packets from USB CLI `rc` commands or 420,000-baud CRSF input on `D7`, and decodes downlink `LINKSTATS` telemetry.
  4. `BRIDGE` / `PONG`: Raw FLRC / LoRa packet transceiver and ping-pong link tester for RF range and antenna characterization.
  5. `IDLE`: Standby (`STDBY_XOSC`) for configuration and spectral RSSI scanning (`scan 900`, `scan 2400`).
- **Host CLI & Bench Automation (`tools/lr2021_cli.py`):** Configures nodes over USB serial, streams live `MONITOR` packets, controls a `DSD Tech SH-UR04A` USB relay for drone power cycling, and controls `MEGA4` PPPS USB hub ports via `uhubctl`.

See [docs/HARDWARE_AND_PROTOCOL.md](docs/HARDWARE_AND_PROTOCOL.md) for hardware schematics, register-level LR2021 notes, OTA packet layouts, and the full CLI command reference.

---

## Hardware Target & Pin Mapping

- **MCU Board:** Seeed Studio XIAO nRF54L15 (`nRF54L15-QFAA`, ARM Cortex-M33 @ 128 MHz, 1.5 MB RRAM, 256 KB SRAM)
- **Radio Expansion Board:** Seeed Studio Wio-LR2021 (`LR2021EVK1XBS1_XIAO`, Semtech LR2021 rev B silicon `0x0118`)
- **Oscillator:** Passive 32 MHz crystal (`has_tcxo = false`). Do not send `SET_TCXO_MODE` (`0x0120`) on this board; doing so disables the passive crystal oscillator and raises `HF_XOSC_START_ERR` (`0x0001`).
- **RF Input Limit:** Maximum +10 dBm at the SMA ports (`ANT_LF` and `ANT_HF`). Attach antennas before transmitting.

| Function | XIAO Pin | nRF54L15 GPIO | Peripheral / Notes |
|---|---|---|---|
| `SPI_SCK` | `D8` | `P2.01` | `SPIM00` @ 8 MHz |
| `SPI_MOSI` | `D10` | `P2.02` | `SPIM00` |
| `SPI_MISO` | `D9` | `P2.04` | `SPIM00` |
| `RADIO_NSS` | `D3` | `P1.07` | Active-low chip select |
| `RADIO_RST` | `D2` | `P1.06` | Active-low reset |
| `RADIO_BUSY` | `D1` | `P1.05` | Active-high busy input |
| `RADIO_IRQ` (`DIO8`) | `D0` | `P1.04` | `GPIOTE20` rising-edge interrupt |
| `USB_CDC_TX` | SAMD11 bridge | `P1.09` | `UARTE22` @ 115200 8N1 |
| `USB_CDC_RX` | SAMD11 bridge | `P1.08` | `UARTE22` @ 115200 8N1 |
| `CRSF_TX` | `D6` (Grove TX) | `P2.08` | `UARTE21` @ 420000 8N1 (to FC RX) |
| `CRSF_RX` | `D7` (Grove RX) | `P2.07` | `UARTE21` @ 420000 8N1 (from FC TX or Handset) |
| `USER_LED` | Built-in | `P2.00` | Active-low status LED |
| `USR_BTN` | Built-in | `P0.00` | Active-low user button |

---

## Build, Test & Flash

### Prerequisites
- `arm-none-eabi-gcc` / `arm-none-eabi-g++` (12+)
- `cmake` (3.20+)
- `openocd` (0.12.0+ with CMSIS-DAP support)
- `python3` with `pyserial` (for `tools/lr2021_cli.py`)

### Build Firmware
```bash
cmake -B build -DCMAKE_TOOLCHAIN_FILE=firmware/toolchain-arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

### Run Host Unit Tests
```bash
mkdir -p build_host
g++ -std=c++17 -O2 -Wall -Wextra tests/test_elrs_host.cpp firmware/elrs/elrs_protocol.cpp firmware/crsf/crsf.cpp -o build_host/test_elrs_host
./build_host/test_elrs_host
```

### Flash to XIAO nRF54L15 via CMSIS-DAP
Pass the CMSIS-DAP serial number (visible in `ls /dev/serial/by-id/`) to `scripts/flash.sh`:
```bash
./scripts/flash.sh FC6D1A37 build/lr2021_elrs.elf
./scripts/flash.sh D56B6941 build/lr2021_elrs.elf
```

---

## Quick-Start Usage

### 1. Passive OTA Monitor on a Live 2.4 GHz ExpressLRS Link (`CE_LBT` / `ISM2G4`)
Sniff an active ExpressLRS link (both handset uplink and receiver downlink telemetry) without transmitting:
```bash
./tools/lr2021_cli.py -p /dev/ttyACM7 configure \
    --phrase "lr2021-elrs-demo" \
    --domain ce2g4 \
    --rate 2G4_K1000 \
    --mode monitor
./tools/lr2021_cli.py -p /dev/ttyACM7 monitor --duration 5
```

### 2. Active ExpressLRS Receiver (`ELRS_RX`) for ArduPilot / Betaflight
Connect XIAO `D6` (`P2.08` CRSF TX) to the flight controller UART RX pin, `D7` (`P2.07` CRSF RX) to the flight controller UART TX pin, and `GND` to `GND` (on ArduPilot, set `SERIALx_PROTOCOL = 23`):
```bash
./tools/lr2021_cli.py -p /dev/ttyACM7 configure \
    --phrase "lr2021-elrs-demo" \
    --domain ce2g4 \
    --rate 2G4_K1000 \
    --mode rx
```

### 3. Sub-GHz UK `EU868` (863.275–869.575 MHz) Transmitter & Receiver Pair
Run Board 1 (`D56B6941` on `/dev/ttyACM5`) as an `EU868` transmitter and Board 2 (`FC6D1A37` on `/dev/ttyACM7`) as an `EU868` receiver:
```bash
# Receiver (Board 2)
./tools/lr2021_cli.py -p /dev/ttyACM7 configure --phrase "lr2021-elrs-demo" --domain eu868 --rate 900_200HZ --mode rx

# Transmitter (Board 1)
./tools/lr2021_cli.py -p /dev/ttyACM5 configure --phrase "lr2021-elrs-demo" --domain eu868 --rate 900_200HZ --mode tx
./tools/lr2021_cli.py -p /dev/ttyACM5 rc 1500 1500 1000 1500 2000 1000 1500 1500
./tools/lr2021_cli.py -p /dev/ttyACM7 status
```

### 4. Direct Serial Console Commands (`115200 8N1`)
Connect to `/dev/ttyACM7` with any serial terminal (`minicom`, `picocom`, `screen`) and run:
```text
help                          - List CLI commands
status                        - Show radio, FHSS lock, packet counters, RSSI/SNR, and RC channels
rates                         - List all 23 supported air rates
phrase <binding_phrase>       - Compute 6-byte UID from ExpressLRS binding phrase (MD5)
uid <XX:XX:XX:XX:XX:XX>       - Set 6-byte UID directly
domain <fcc915|au915|eu868|ism2g4|ce_lbt> - Set regulatory frequency domain
rate <idx_or_name>            - Select ExpressLRS air rate (0..22 or name)
power <dbm>                   - Set TX output power in dBm (-20..22 Sub-GHz, -20..12 2.4 GHz)
mode <idle|tx|rx|monitor|bridge> - Switch operating mode
rc <ch1_us> [ch2_us..ch16_us] - Set uplink RC channel values in microseconds (988..2012)
ping <freq_hz> <lora|flrc> [n]- Transmit n ping packets and measure PONG RTT/RSSI
pong <freq_hz> <lora|flrc>    - Listen on freq_hz and auto-reply PONG
txpkt <hex_bytes>             - Transmit raw hex payload in BRIDGE mode
scan <900|2400>               - Sweep RSSI across Sub-GHz or 2.4 GHz band
```

---

## Measured Hardware Verification

Tested on two `LR2021EVK1XBS1_XIAO` boards (`FC6D1A37` and `D56B6941`) and a live **RadioMaster TX16SMK3 ↔ RadioMaster XR4 (`2× LR1121`)** ExpressLRS link (`lr2021-elrs-demo`, `UID = 0A:B7:0C:7C:38:03`):

1. **Live 2.4 GHz `2G4_K1000` (`CE_LBT`, 80 channels, 1000 Hz GFSK + Hamming FEC, 1:2 telemetry) Passive `MONITOR`:**
   - Captures 500 Hz TX16SMK3 uplink (`SYNC` + `RCDATA`) and 500 Hz XR4 downlink (`LINKSTATS` + CRSF telemetry `DATA` frames) simultaneously (`1,789 / 1,842 = 97.1%` combined packet CRC rate over `2.0 s`, `phase_err = 4 us`):
   ```text
   MON t=9691380 freq=2440400000 nonce=193 hop=160 type=SYNC crc=1 rssi= -40 snr= 0 ch=[1500,1500, 989,1498] raw=AAA0C1240F466AD3
   STATUS mode=MONITOR lock=CONNECTED chip=0x0118 err=0x0000 domain=CE_LBT rate=2G4_K1000(13) uid=0A:B7:0C:7C:38:03 pwr=10dBm irq=1842 tx=0 rx=1842 crc_ok=1789 crc_err=53 sync_rx=0 rc_rx=908 tlm_rx=880 hops=927 rssi=-36dBm snr=0.00dB phase_err=4us
   RC_US ch1=1500 ch2=1500 ch3=989 ch4=1498 ch5=1000 ch6=1000 ch7=1000 ch8=1000 armed=0
   TLM_LINK uplink_rssi=-74dBm lq=99% snr=0dB
   ```
2. **Live 2.4 GHz `2G4_K1000` Active `ELRS_RX` (5 consecutive runs):**
   - Locks in `< 160 ms` across all 5 runs (`98.6%–99.5%` RC packet CRC rate, `phase_err` within `-3..+5 us`, transmitting 500 Hz `LINKSTATS` responses):
   ```text
   STATUS mode=ELRS_RX lock=CONNECTED chip=0x0118 err=0x0000 domain=CE_LBT rate=2G4_K1000(13) uid=0A:B7:0C:7C:38:03 pwr=10dBm irq=1456 tx=728 rx=728 crc_ok=723 crc_err=5 sync_rx=0 rc_rx=722 tlm_rx=0 hops=731 rssi=-40dBm snr=0.00dB phase_err=4us
   ```
3. **EVK-to-EVK UK `EU868` (863.275–869.575 MHz, 13 channels) `900_200HZ` LoRa & `900_K1000` FSK Active `ELRS_TX` ↔ `ELRS_RX`:**
   - `900_200HZ`: `371 / 384` (`96.6%`) CRC-valid packets, `snr = +9.25 dB`, bidirectional `LINKSTATS` (`lq = 100%`).
   - `900_K1000`: `654 / 667` (`98.1%`) CRC-valid packets, `phase_err = -5 us`, bidirectional `LINKSTATS` (`lq = 100%`).

### Limitations
- Single-transceiver hardware (`1× LR2021` per EVK): Gemini X dual-band modes (`X150HZ_*`, `X100HZ_8CH_*`) can track or transmit on either the Sub-GHz leg or the 2.4 GHz leg one at a time, not both bands simultaneously on a single EVK.
- Downlink telemetry in `ELRS_RX` mode currently packs `LINKSTATS` (`ELRS_PKT_TYPE_LINKSTATS`); multi-packet stubborn-sender MSP/MAVLink downlink `DATA` framing from the flight controller UART is not yet implemented.

---

## License

Licensed under the GNU Affero General Public License v3.0 or later (`SPDX-License-Identifier: AGPL-3.0-or-later`). See [LICENSE](LICENSE).

Portions of the radio register definitions, PRAM patch image, and ExpressLRS OTA/FHSS/CRSF protocol logic are adapted from:
- ExpressLRS (`GPL-3.0-or-later`, `src/lib/LR2021Driver/`, `src/lib/OTA/`, `src/lib/FHSS/`)
- Semtech Corporation `lr20xx_driver` (`BSD-3-Clause`)
- Nordic Semiconductor `nrfx` / MDK headers (`BSD-3-Clause`)
