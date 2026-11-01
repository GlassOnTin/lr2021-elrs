# Hardware & Protocol Architecture

This document covers the hardware interface on the Seeed Studio Semtech LR2021 LoRa Plus Evaluation Kit (`LR2021EVK1XBS1_XIAO`), the Semtech LR2021 Rev B (`0x0118`) initialization sequence, the ExpressLRS 3.x OTA4/OTA8 packet and FHSS engines, and the USB CDC CLI protocol.

---

## 1. Hardware Architecture (`XIAO nRF54L15` + `Wio-LR2021`)

### 1.1 MCU Clock and Timer Domains

The Nordic `nRF54L15` runs on its 128 MHz CPU clock with three separate peripheral domains (`PERI00`, `PERI10`, `PERI20`), each clocked differently:

| Peripheral | Base Address | Bus Clock | Prescaler | Resolution | Role in Firmware |
|---|---|---|---|---|---|
| `SPIM00` | `0x5004A000` | 32 MHz (`PCLK32M`) | `/2` → 16 MHz SCK | 62.5 ns/bit | Full-duplex SPI to `Wio-LR2021` on `P2.01..P2.04` |
| `TIMER10` | `0x50085000` | 32 MHz (`PCLK32M`) | `PRESCALER = 5` (`2^5 = 32`) | 1.0 µs/tick | Free-running 32-bit monotonic microsecond clock (`hal_micros()`) |
| `TIMER20` | `0x500CE000` | 16 MHz (`PCLK16M`) | `PRESCALER = 4` (`2^4 = 16`) | 1.0 µs/tick | Phase-lockable ExpressLRS tick/tock hardware timer (`TIMER20_IRQn`) |
| `UARTE20` | `0x500C6000` | 16 MHz (`PCLK16M`) | `BAUDRATE = 0x01D60000` | 115200 8N1 | USB CDC UART via onboard SAMD11 CMSIS-DAP bridge (`P1.08` RX / `P1.09` TX) |
| `UARTE21` | `0x500C7000` | 16 MHz (`PCLK16M`) | `BAUDRATE = 0x10000000` (420k) | 420000 / 400000 8N1 | External CRSF output (`P1.11` TX / `P1.10` RX) |
| `GPIOTE20` | `0x500DA000` | 16 MHz (`PCLK16M`) | — | Rising edge | `DIO9` (`P1.06`) hardware interrupt (`GPIOTE20_0_IRQn`) |

> **Note on `TIMER10` vs `TIMER20`:** `TIMER10` sits on the `PERI10` 32 MHz bus (`PCLK32M`), so a 1 MHz tick requires `PRESCALER = 5` (`32 MHz / 32 = 1 MHz`). `TIMER20` sits on the `PERI20` 16 MHz bus (`PCLK16M`), so a 1 MHz tick requires `PRESCALER = 4` (`16 MHz / 16 = 1 MHz`). Setting `TIMER10` to `PRESCALER = 4` causes `hal_micros()` to run at 2 MHz (2× fast), which doubles the measured sync-channel nonce delta and breaks passive `MONITOR` hop period validation.

### 1.2 Semtech LR2021 Rev B (`0x0118`) Initialization Sequence

The `Wio-LR2021` module on the Seeed EVK uses two hardware configurations that differ from generic Semtech reference boards:

1. **Passive 32 MHz Crystal (`XOSC`), No TCXO:**
   Calling `SET_TCXO_MODE` (`0x0120`) shuts down the crystal oscillator and leaves the chip unable to lock its PLL (`PLL_LOCK_ERR`). Do **not** issue `0x0120` on the Wio-LR2021 EVK.
2. **Switchless Front-End (Internal RF Switch):**
   `DIO5..DIO8` must not be driven as external Skyworks SKY13373 RF switch controls during boot.

The bring-up sequence in `lr2021_init()` (`firmware/radio/lr2021_driver.cpp`) executes:

1. Toggle `NRESET` (`P1.07`) low for 2 ms, release high, and wait for `BUSY` (`P1.05`) to deassert.
2. Issue `SET_STANDBY(XOSC = 0x01)` (`0x011C`) and verify `GET_VERSION` (`0x0101`) returns `0x0118` (LR2021 Rev B).
3. Load the 2652-byte Rev B PRAM firmware patch (`pram_revB_0118.h`) into address `0x008000` in 128-byte chunks using `WRITE_MEM32` (`0x0108`), then enable the 6 patch slots at `0x008A5C` via `WRITE_REG_MEM32` (`0x010A`).
4. Run `CALIBRATE(0x7F)` (`0x0122`) to calibrate LF RC, HF RC, PLL, ADC, image rejection, and PA offset.
5. Configure `DIO9` (`0x0112`, `dio = 9`, `func = 0x01`, `pull_drive = 0x01`) as an active-high IRQ output wired to `RX_DONE | TX_DONE | CRC_ERROR | TIMEOUT` (`0x001C0040`).

### 1.3 ISR Concurrency and SPI Bus Locking

Both `GPIOTE20_0_IRQn` (`DIO9` radio interrupt) and `TIMER20_IRQn` (ExpressLRS tick/tock timer) issue multi-byte SPI transactions over `SPIM00`. To prevent SPI transaction corruption:

- `GPIOTE20_0_IRQn` and `TIMER20_IRQn` share the same NVIC priority level (`2`), so neither preempts the other mid-transfer.
- Foreground code in `main()` never calls `lr2021_*` while a radio mode is running (`g_chip_version` is cached at boot for `status`), and `start_mode()` disables `hal_radio_irq_enable(false)` and stops `hal_hwtimer_stop()` before reconfiguring the radio.
- All UART formatting (`MONITOR` packet logs, `ELRS_RX` status lines, and 420 kbaud CRSF frames) is deferred out of the ISRs into volatile state flags (`g_rx_log_pending`, `g_crsf_tx_pending`) serviced by the main loop.

---

## 2. ExpressLRS 3.x Protocol Implementation

### 2.1 UID Derivation and CRC Seeds

Binding phrases are hashed with MD5 following the ExpressLRS algorithm (`elrs_uid_from_phrase()` in `firmware/elrs/elrs_protocol.cpp`):

```text
MD5("-DMY_BINDING_PHRASE=\"" + phrase + "\"") -> first 6 bytes = UID[0..5]
```

For example, `"lr2021-elrs-demo"` yields `UID = 0A:B7:0C:7C:38:03`:
- **`OtaCrcInitializer` (16-bit):** `(UID[4] << 8) | UID[5] = 0x466A`
  - **OTA4 (8-byte packet, 14-bit CRC, polynomial `0x2E57`):** `crc_init = OtaCrcInitializer ^ (ota_version_id << 12)` (`ota_version_id = 3` for ELRS 3.x).
  - **OTA8 (13-byte packet, 16-bit CRC, polynomial `0x3D65`):** `crc_init = OtaCrcInitializer ^ (ota_version_id << 8)`.
- **`FHSSrandomisr` Seed (32-bit):** `((UID[2] << 24) | (UID[3] << 16) | (UID[4] << 8) | (UID[5] ^ OTA_VERSION_ID)) & 0x7FFFFFFF`.

### 2.2 Supported Rate Profiles

`firmware/elrs/elrs_protocol.cpp` defines 11 rate indices (`0..10`):

| Index | Name | Band | Modulation | Rate | Packet Format | `fhss_hop_interval` | `tlm_interval` |
|---|---|---|---|---|---|---|---|
| `0` | `2G4_500HZ` | 2.4 GHz | LoRa SF6 BW812 CR4/5 | 500 Hz (2000 µs) | OTA4 (8 B) | 4 | 8 |
| `1` | `2G4_250HZ` | 2.4 GHz | LoRa SF7 BW812 CR4/5 | 250 Hz (4000 µs) | OTA4 (8 B) | 4 | 4 |
| `2` | `2G4_150HZ` | 2.4 GHz | LoRa SF8 BW812 CR4/5 | 150 Hz (6666 µs) | OTA4 (8 B) | 2 | 4 |
| `3` | `2G4_50HZ` | 2.4 GHz | LoRa SF9 BW812 CR4/5 | 50 Hz (20000 µs) | OTA4 (8 B) | 2 | 2 |
| `4` | `2G4_K1000` | 2.4 GHz | FLRC 1.04 Mbps CR1/2 | 1000 Hz (1000 µs) | OTA8 (13 B) | 2 | 2 |
| `5` | `2G4_F1000` | 2.4 GHz | FLRC 1.04 Mbps CR1/2 | 1000 Hz (1000 µs) | OTA4 (8 B) | 2 | 8 |
| `6` | `900_200HZ` | Sub-GHz | LoRa SF6 BW500 CR4/5 | 200 Hz (5000 µs) | OTA4 (8 B) | 4 | 4 |
| `7` | `900_100HZ` | Sub-GHz | LoRa SF7 BW500 CR4/5 | 100 Hz (10000 µs) | OTA4 (8 B) | 2 | 4 |
| `8` | `900_50HZ` | Sub-GHz | LoRa SF8 BW500 CR4/5 | 50 Hz (20000 µs) | OTA4 (8 B) | 2 | 2 |
| `9` | `900_25HZ` | Sub-GHz | LoRa SF9 BW500 CR4/5 | 25 Hz (40000 µs) | OTA4 (8 B) | 2 | 2 |
| `10` | `900_K1000` | Sub-GHz | GFSK 300 kbps + FEC | 1000 Hz (1000 µs) | OTA4 (8 B → 14 B air) | 2 | 2 |

### 2.3 Regulatory Domains

| Index | Name | CLI Alias | Start Frequency | Step | Channels | Sync Channel |
|---|---|---|---|---|---|---|
| `0` | `ISM_2400` | `ism2400` | 2400.4 MHz | 1.0 MHz | 80 | 40 (`2440.4 MHz`) |
| `1` | `CE_LBT` | `ce_lbt` | 2400.4 MHz | 1.0 MHz | 80 | 40 (`2440.4 MHz`) |
| `2` | `AU915` | `au915` | 915.5 MHz | 0.6 MHz | 20 | 10 (`921.5 MHz`) |
| `3` | `FCC915` | `fcc915` | 903.5 MHz | 0.6 MHz | 40 | 20 (`915.5 MHz`) |
| `4` | `EU868` | `eu868` | 863.275 MHz | 0.525 MHz | 13 | 6 (`866.425 MHz`) |
| `5` | `IN866` | `in866` | 865.375 MHz | 0.5 MHz | 4 | 2 (`866.375 MHz`) |

### 2.4 Sub-GHz K1000 (`900_K1000`) Hamming(7,4) FEC & Bit Interleaver

In `900_K1000` (`LR2021_MOD_GFSK`, 300 kbps, BT=0.5, Δf=180 kHz, BW=666 kHz, 32-bit syncword = `UID[2..5]`), the 8-byte OTA4 frame is encoded into 14 air bytes using systematic Hamming(7,4) with a 4×7 bit interleaver (`elrs_fec_encode()` / `elrs_fec_decode()`):

1. Every 2 input bytes (4 nibbles $n_0, n_1, n_2, n_3$) are mapped through `k_hamming_enc[16]` into four 7-bit codewords $c_0, c_1, c_2, c_3$.
2. Those 28 bits are interleaved column-by-column across 4 nibbles (7 columns × 4 rows = 28 bits = 3.5 bytes).
3. Two 2-byte groups (4 input bytes) form 7 air bytes; the full 8-byte OTA4 frame forms 14 air bytes.
4. On receive, `elrs_fec_decode()` de-interleaves each 7-byte block back into four 7-bit codewords and corrects any single-bit error per codeword using the syndrome lookup table `k_hamming_dec[128]`.

### 2.5 FHSS Sequence Generation & Fast Two-Packet Sync Lock

`elrs_fhss_build_sequence()` generates the deterministic permutation table `fhss_seq[]` matching ExpressLRS `FHSS.cpp`:
- `sync_channel = (num_channels / 2)` (`40` on 2.4 GHz, `6` on `EU868`).
- `freq_spread = (256 / num_channels)`.
- The table contains `(256 / max_k) * num_channels` entries (`240` entries across 3×80-channel passes on 2.4 GHz; `247` entries across 19×13-channel passes on `EU868`), seeded by the LCG `rand = (rand * 214013 + 2531011) & 0x7FFFFFFF`.

When acquiring a lock on `sync_channel`:
1. **Explicit `SYNC` packet (`PACKET_TYPE_SYNC = 0x02`):** Carries `fhss_index` (`pkt[1]`), `nonce` (`pkt[2]`), and `UID[3..5]` (`pkt[3..5]`), locking immediately in one packet.
2. **Fast `RCDATA` consecutive-nonce pair lock (for already-connected transmitters):** When a transmitter is already connected to a receiver (`sendSyncSpam` exhausted), it only sends explicit `SYNC` frames every ~64 hops, but still visits `sync_channel` for `fhss_hop_interval = 2` consecutive slots whenever `fhss_seq[fhss_index] == sync_channel`. On 2.4 GHz (`num_channels = 80`), `sync_channel` (`40`) is always at offset `0` of each 80-entry subsequence (`0, 80, 160`). When `ELRS_RX` or `MONITOR` observes two `RCDATA` frames on `sync_channel` separated by `1 * interval_us`, it locks `nonce = (fhss_index * 2) + 1` and steps through `subseq = {0, 80, 160}` on timeout until locked—acquiring the live hop train in `< 160 ms`.
3. **Two-Slot Hop Preservation (`did_hop` Guard):** Because `fhss_hop_interval = 2` stays on the same RF channel for two consecutive `1000 µs` slots (even nonce `N` and odd nonce `N+1`), `elrs_hwtimer_tock_cb()` only calls `lr2021_set_frequency(freq, true)` when `did_hop == true` (even nonces). On odd nonces (`did_hop == false`), the radio remains in continuous RX on the current channel without re-locking the PLL during the incoming preamble.

---

## 3. CLI & Host Tool Reference

### 3.1 Firmware USB CDC Serial Commands (`115200 8N1`)

| Command | Arguments | Description |
|---|---|---|
| `help` | — | Print available serial commands |
| `status` | — | Print firmware mode, chip version (`0x0118`), bind phrase, UID, domain, rate, and packet counters |
| `bind <phrase>` | `<phrase>` (string) | Compute MD5 UID (`elrs_uid_from_phrase()`), rebuild FHSS sequence, and restart active mode |
| `uid <hex>` | `XX:XX:XX:XX:XX:XX` | Set 6-byte ExpressLRS UID directly and rebuild FHSS sequence |
| `domain <d>` | `0..5` or `ism2400`, `ce_lbt`, `au915`, `fcc915`, `eu868`, `in866` | Switch regulatory domain and frequency band |
| `rate <0..10>` | `0..10` | Select ExpressLRS rate profile (see Section 2.2) |
| `power <dbm>` | `-10..22` | Set TX output power in dBm |
| `tlm <0\|1>` | `0` or `1` | Disable (`0`) or enable (`1`) receiver downlink `LINKSTATS` telemetry slots |
| `ch <idx> <us>` | `idx`: `1..16`, `us`: `988..2012` | Set RC channel value in microseconds (`ELRS_TX` mode) |
| `mode rx` | — | Start `ELRS_RX` synchronized FHSS receiver mode |
| `mode tx` | — | Start `ELRS_TX` synchronized FHSS transmitter mode |
| `mode mon` | — | Start passive `MONITOR` mode (hop tracking + `SYNC`/`RCDATA`/`TLM` packet logger) |
| `cw <hz> <dbm>` | `<hz>` frequency, `<dbm>` power | Transmit unmodulated carrier wave for spectrum analysis |
| `stop` | — | Stop radio, disable `TIMER20` and `DIO9` interrupts, and enter `IDLE` |

### 3.2 Host CLI & Test Tool (`tools/lr2021_cli.py`)

```bash
# Discover connected Seeed XIAO nRF54L15 EVK boards and query status
python3 tools/lr2021_cli.py status

# Configure and run passive sniffer (MONITOR mode)
python3 tools/lr2021_cli.py monitor --port /dev/ttyACM5 --phrase lr2021-elrs-demo --domain ce_lbt --rate 4 --duration 10

# Configure and run ELRS_RX mode
python3 tools/lr2021_cli.py rx --port /dev/ttyACM5 --phrase lr2021-elrs-demo --domain ce_lbt --rate 4 --tlm 1 --duration 10

# Configure and run ELRS_TX mode
python3 tools/lr2021_cli.py tx --port /dev/ttyACM7 --phrase lr2021-elrs-demo --domain eu868 --rate 10 --power 10 --duration 10

# Run dual-board automated TX <-> RX verification test
python3 tools/lr2021_cli.py pair-test --tx-port /dev/ttyACM7 --rx-port /dev/ttyACM5 --phrase lr2021-elrs-demo --domain eu868 --rate 10 --duration 6

# Inspect or power-cycle ports on the MEGA4 USB PPPS hub (3-1)
python3 tools/lr2021_cli.py hub --action status
python3 tools/lr2021_cli.py hub --action cycle --hub-port 2
```
