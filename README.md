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
| `RADIO_RST` | `D2` | `P1.06` | Active-low reset |
| `RADIO_BUSY` | `D1` | `P1.05` | Active-high busy input |
| `RADIO_IRQ` (`DIO8`) | `D0` | `P1.04` | `GPIOTE20` rising-edge interrupt |
| `RF Front-End` | Direct-tie | Internal | Switchless LF (`RFI_LF0`) and HF (`RFI_HF`, `HF_BOOST=7`) paths |
| `USB_CDC_TX` | SAMD11 bridge | `P1.09` | `UARTE22` @ 115200 8N1 |
| `USB_CDC_RX` | SAMD11 bridge | `P1.08` | `UARTE22` @ 115200 8N1 |
| `CRSF_TX` | `D6` | `P2.08` | `UARTE21` @ 420000 8N1 |
| `CRSF_RX` | `D7` | `P2.07` | `UARTE21` @ 420000 8N1 |
| `USER_LED` | Built-in | `P2.00` | Active-low status LED |

## Supported Modes & Air Rates

The firmware supports 5 operating modes (`IDLE`, `ELRS_TX`, `ELRS_RX`, `MONITOR`, `BRIDGE`) and 23 air rate configurations across 5 regulatory domains (`FCC915`, `AU915`, `EU868`, `ISM2G4`, `CE_LBT`):

- **Sub-GHz (`EU868` 863.275–869.575 MHz / `FCC915` / `AU915`):** `900_K1000_8CH`, `900_K1000` (300 kbps GFSK + Hamming(7,4) FEC), `900_250HZ` (LoRa SF5 BW500), `900_200HZ_8CH` (SF5 BW500), `900_200HZ` (SF6 BW500), `900_100HZ_8CH` (SF6 BW500), `900_100HZ` (SF7 BW500), `900_50HZ` (SF8 BW500), `900_25HZ` (SF9 BW500), `900_D50HZ` (SF6 DVDA)
- **2.4 GHz (`CE_LBT` / `ISM2G4` 2400.4–2479.4 MHz, 80 channels):** `2G4_F1000`, `2G4_F500`, `2G4_D500`, `2G4_D250` (FLRC 650 kbps), `2G4_K1000`, `2G4_DK500`, `2G4_DK250` (300 kbps GFSK + Hamming(7,4) 8B->14B FEC), `2G4_500HZ` (LoRa SF5 BW800), `2G4_333HZ_8CH` (SF5 BW800), `2G4_250HZ` (SF6 BW800), `2G4_150HZ` (SF7 BW800), `2G4_100HZ_8CH` (SF7 BW800), `2G4_50HZ` (SF8 BW800)
- **Gemini X Dual-Band:** `X150HZ_900`, `X150HZ_2G4`, `X100HZ_8CH_900`, `X100HZ_8CH_2G4`

## Hardware Verification (`TX16SMK3` <-> `XR4` Live Link)

Verified on `LR2021EVK1XBS1_XIAO` (`FC6D1A37`) against a live RadioMaster TX16SMK3 transmitter and RadioMaster XR4 (`2x LR1121`) receiver (`lr2021-elrs-demo`, `UID = 0A:B7:0C:7C:38:03`, `CE_LBT`, `2G4_K1000` 1000 Hz 300 kbps GFSK with Hamming(7,4) FEC, `1:2` telemetry ratio across 80 FHSS channels `2400.4–2479.4 MHz`):

- **`MONITOR` Mode (Passive OTA Sniffer across all 80 FHSS channels):**
  - Captures both 500 Hz TX16SMK3 uplink (`SYNC` and `RCDATA`) and 500 Hz XR4 downlink (`LINKSTATS` and CRSF telemetry `DATA`) in real time:
  ```text
  MON t=9691380 freq=2440400000 nonce=193 hop=160 type=SYNC crc=1 rssi= -40 snr= 0 ch=[1500,1500, 989,1498] raw=AAA0C1240F466AD3
  STATUS mode=MONITOR lock=CONNECTED chip=0x0118 err=0x0000 domain=CE_LBT rate=2G4_K1000(13) uid=0A:B7:0C:7C:38:03 pwr=10dBm irq=1842 tx=0 rx=1842 crc_ok=1789 crc_err=53 sync_rx=0 rc_rx=908 tlm_rx=880 hops=927 rssi=-36dBm snr=0.00dB phase_err=4us
  RC_US ch1=1500 ch2=1500 ch3=989 ch4=1498 ch5=1000 ch6=1000 ch7=1000 ch8=1000 armed=0
  TLM_LINK uplink_rssi=-74dBm lq=99% snr=0dB
  ```
- **`ELRS_RX` Mode (Active Receiver + 500 Hz Telemetry TX + 420k-baud CRSF Output):**
  - Locks onto the 80-channel FHSS sequence in `< 160 ms` (`98.6%–99.5%` packet CRC rate across 5 consecutive runs, `phase_err` within `-3..+5 us`):
  ```text
  STATUS mode=ELRS_RX lock=CONNECTED chip=0x0118 err=0x0000 domain=CE_LBT rate=2G4_K1000(13) uid=0A:B7:0C:7C:38:03 pwr=10dBm irq=1456 tx=728 rx=728 crc_ok=723 crc_err=5 sync_rx=0 rc_rx=722 tlm_rx=0 hops=731 rssi=-40dBm snr=0.00dB phase_err=4us
  RC_US ch1=1500 ch2=1500 ch3=989 ch4=1498 ch5=1000 ch6=1000 ch7=1000 ch8=1000 armed=0
  TLM_LINK uplink_rssi=-73dBm lq=99% snr=0dB
  ```

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
