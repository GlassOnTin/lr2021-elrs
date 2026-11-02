-- TNS|RNS-ELRS Mesh|TNE
-- SPDX-License-Identifier: AGPL-3.0-or-later
--
-- EdgeTX / OpenTX Tool Script for LR2021 ExpressLRS + Reticulum (RNS)
-- Place in /SCRIPTS/TOOLS/rns_elrs.lua on the handset SD card.
--
-- Communicates with the LR2021 TX module over CRSF extended frames:
--   0x7A (CRSF_FRAMETYPE_RNS_CMD) : Handset Lua -> TX Module
--   0x7B (CRSF_FRAMETYPE_RNS_TLM) : TX Module   -> Handset Lua
--
-- Features:
--   Page 1: Reticulum 16B SINGLE Identity & Over-the-Air RX Pairing (RNSP)
--   Page 2: RNode LoRa Radio & IFAC Configuration (868/915 MHz & 2.4 GHz)
--   Page 3: Discovered Reticulum Mesh Peers (RNSA Announces, Hops, RSSI, VBatt)
--   Page 4: Targeted Receiver Commands & Mesh Ping/Pong RTT (RNSC)

local CRSF_ADDR_HANDSET  = 0xEA
local CRSF_ADDR_RECEIVER = 0xEC
local CRSF_ADDR_MODULE   = 0xEE
local CRSF_CMD_RNS       = 0x7A
local CRSF_TLM_RNS       = 0x7B
local CRSF_TLM_FM        = 0x21

local SUBCMD_GET_STATUS  = 0x01
local SUBCMD_GET_PEER    = 0x02
local SUBCMD_SET_CFG     = 0x03
local SUBCMD_PAIR_RX     = 0x04
local SUBCMD_SELECT_PEER = 0x05
local SUBCMD_ACTION      = 0x06

local RESP_STATUS = 0x81
local RESP_PEER   = 0x82
local RESP_ACK    = 0x83

local ACT_ANNOUNCE  = 0x01
local ACT_PING_RX   = 0x02
local ACT_LOITER    = 0x03
local ACT_RTL       = 0x04
local ACT_LAND      = 0x05
local ACT_ARM       = 0x06
local ACT_DISARM    = 0x07
local ACT_FORM_VEE  = 0x08
local ACT_FORM_LINE = 0x09
local ACT_FORM_COL  = 0x0A
local ACT_FORM_HOLD = 0x0B

local MODE_NAMES = { [0]="IDLE", [1]="PONG", [2]="ELRS_TX", [3]="ELRS_RX", [4]="MONITOR", [5]="BRIDGE", [6]="RNODE", [7]="RELAY" }
local LOCK_NAMES = { [0]="DISC", [1]="TENT", [2]="LOCKED" }
local ROLE_NAMES = { [1]="TX", [2]="RX", [3]="RLY", [4]="RND" }
local BW_Presets = { 125000, 250000, 500000, 812500 }
local FREQ_PRESETS = { 868000000, 869525000, 915000000, 2440000000 }

local state = {
  page         = 1,
  cursor       = 1,
  editing      = false,
  last_poll_ms = 0,
  poll_peer_ix = 0,
  status_msg   = "Ready",

  app_mode      = 0,
  lock_state    = 0,
  rate_idx      = 3,
  tx_power_dbm  = 10,
  rnode_freq_hz = 868000000,
  rnode_bw_hz   = 125000,
  rnode_sf      = 7,
  rnode_cr      = 5,
  ifac_enabled  = 0,
  peer_count    = 0,
  selected_peer = -1,
  last_rtt_ms   = 0,
  tun_tx_cnt    = 0,
  tun_rx_cnt    = 0,
  relay_fwd_cnt = 0,
  tx_dest_hex   = "00000000000000000000000000000000",
  rx_dest_hex   = "00000000000000000000000000000000",

  cfg_freq_idx  = 1,
  cfg_bw_idx    = 1,
  cfg_sf        = 7,
  cfg_cr        = 5,
  cfg_txpwr     = 14,
  cfg_ifac      = 0,
  cfg_synced    = false,
  ota_rx_direct = false,
  fm_str        = "",

  peers = {}
}

local function bytesToHex(data, startIdx, count)
  local s = ""
  for i = 0, count - 1 do
    local b = data[startIdx + i] or 0
    s = s .. string.format("%02x", b)
  end
  return s
end

local function bytesToAscii(data, startIdx, maxLen)
  local s = ""
  for i = 0, maxLen - 1 do
    local b = data[startIdx + i]
    if not b or b == 0 then break end
    if b >= 32 and b <= 126 then
      s = s .. string.char(b)
    end
  end
  return s
end

local function u32be(data, idx)
  return bit32.lshift(data[idx] or 0, 24)
       + bit32.lshift(data[idx + 1] or 0, 16)
       + bit32.lshift(data[idx + 2] or 0, 8)
       + (data[idx + 3] or 0)
end

local function u16be(data, idx)
  return bit32.lshift(data[idx] or 0, 8) + (data[idx + 1] or 0)
end

local function i8(val)
  if val >= 128 then return val - 256 end
  return val
end

local function sendRnsCmd(subcmd, args)
  local dest = state.ota_rx_direct and CRSF_ADDR_RECEIVER or CRSF_ADDR_MODULE
  local payload = { dest, CRSF_ADDR_HANDSET, subcmd }
  if args then
    for i = 1, #args do
      payload[#payload + 1] = args[i]
    end
  end
  if crossfireTelemetryPush then
    crossfireTelemetryPush(CRSF_CMD_RNS, payload)
  end
end

local function pushConfigToModule(andPairRx)
  local f  = FREQ_PRESETS[state.cfg_freq_idx] or state.rnode_freq_hz
  local bw = BW_Presets[state.cfg_bw_idx] or state.rnode_bw_hz
  local pwrByte = state.cfg_txpwr
  if pwrByte < 0 then pwrByte = 256 + pwrByte end
  local args = {
    bit32.band(bit32.rshift(f, 24), 0xFF),
    bit32.band(bit32.rshift(f, 16), 0xFF),
    bit32.band(bit32.rshift(f, 8), 0xFF),
    bit32.band(f, 0xFF),
    bit32.band(bit32.rshift(bw, 24), 0xFF),
    bit32.band(bit32.rshift(bw, 16), 0xFF),
    bit32.band(bit32.rshift(bw, 8), 0xFF),
    bit32.band(bw, 0xFF),
    state.cfg_sf,
    state.cfg_cr,
    pwrByte,
    state.cfg_ifac
  }
  sendRnsCmd(SUBCMD_SET_CFG, args)
  if andPairRx then
    sendRnsCmd(SUBCMD_PAIR_RX, nil)
    state.status_msg = "CFG + OTA RX Pair Sent"
  else
    state.status_msg = "CFG Applied"
  end
end

local function pollTelemetry()
  if not crossfireTelemetryPop then return end
  for _ = 1, 4 do
    local cmd, data = crossfireTelemetryPop()
    if not cmd or not data then break end
    if cmd == CRSF_TLM_RNS and #data >= 4 then
      local rtype = data[3]
      if rtype == RESP_STATUS and #data >= 58 then
        state.tx_dest_hex   = bytesToHex(data, 4, 16)
        state.rx_dest_hex   = bytesToHex(data, 20, 16)
        state.app_mode      = data[36]
        state.lock_state    = data[37]
        state.rate_idx      = data[38]
        state.ifac_enabled  = data[39]
        state.rnode_freq_hz = u32be(data, 40)
        state.rnode_bw_hz   = u16be(data, 44) * 1000
        state.rnode_sf      = data[46]
        state.rnode_cr      = data[47]
        state.tx_power_dbm  = i8(data[48])
        state.peer_count    = data[49]
        state.selected_peer = i8(data[50])
        state.last_rtt_ms   = u16be(data, 51)
        state.tun_tx_cnt    = u16be(data, 53)
        state.tun_rx_cnt    = u16be(data, 55)
        state.relay_fwd_cnt = u16be(data, 57)

        if not state.cfg_synced then
          for fi = 1, #FREQ_PRESETS do
            if FREQ_PRESETS[fi] == state.rnode_freq_hz then
              state.cfg_freq_idx = fi
              break
            end
          end
          for bi = 1, #BW_Presets do
            if BW_Presets[bi] == state.rnode_bw_hz then
              state.cfg_bw_idx = bi
              break
            end
          end
          state.cfg_sf     = state.rnode_sf
          state.cfg_cr     = state.rnode_cr
          state.cfg_txpwr  = state.tx_power_dbm
          state.cfg_ifac   = state.ifac_enabled
          state.cfg_synced = true
        end
      elseif rtype == RESP_PEER and #data >= 44 then
        local idx = data[4]
        state.peer_count = data[5]
        state.peers[idx] = {
          is_selected = (data[6] ~= 0),
          role        = data[7],
          dest_hex    = bytesToHex(data, 8, 16),
          name        = bytesToAscii(data, 24, 12),
          hops        = data[36],
          rssi_dbm    = i8(data[37]),
          snr_db      = i8(data[38]),
          armed       = (data[39] ~= 0),
          flight_mode = data[40],
          vbatt_mv    = u16be(data, 41),
          age_s       = u16be(data, 43)
        }
      elseif rtype == RESP_ACK and #data >= 5 then
        local ackSubcmd = data[4]
        local status    = data[5]
        state.status_msg = string.format("%s (0x%02X)", status == 0 and "OK" or "ERR", ackSubcmd)
      end
    elseif cmd == CRSF_TLM_FM and #data >= 4 then
      state.fm_str = bytesToAscii(data, 1, 28)
      if state.fm_str ~= "" then
        state.status_msg = state.fm_str
      end
    end
  end

  local now = getTime() * 10
  if (now - state.last_poll_ms) >= 350 then
    state.last_poll_ms = now
    if state.page == 3 and state.peer_count > 0 then
      sendRnsCmd(SUBCMD_GET_PEER, { state.poll_peer_ix })
      state.poll_peer_ix = (state.poll_peer_ix + 1) % state.peer_count
    else
      sendRnsCmd(SUBCMD_GET_STATUS, nil)
    end
  end
end

local function drawHeader(title)
  lcd.clear()
  local isColor = (LCD_W >= 320)
  if isColor then
    lcd.drawText(6, 4, "RNS-ELRS [" .. state.page .. "/4] " .. title, BOLD)
    lcd.drawText(LCD_W - 150, 4, state.status_msg, 0)
    lcd.drawLine(0, 24, LCD_W, 24, SOLID, 0)
  else
    lcd.drawText(1, 0, state.page .. "/4 " .. title, INVERS)
  end
end

local function drawPage1()
  drawHeader("Identity & Pairing")
  local y = (LCD_W >= 320) and 32 or 10
  local dy = (LCD_W >= 320) and 22 or 9

  local mstr = MODE_NAMES[state.app_mode] or "?"
  local lstr = LOCK_NAMES[state.lock_state] or "?"
  lcd.drawText(6, y, string.format("Mode:%s(%s) Peers:%d RTT:%dms", mstr, lstr, state.peer_count, state.last_rtt_ms), 0)
  y = y + dy
  lcd.drawText(6, y, "TX RNS: " .. string.sub(state.tx_dest_hex, 1, 16), 0)
  y = y + dy
  lcd.drawText(6, y, "        " .. string.sub(state.tx_dest_hex, 17, 32), 0)
  y = y + dy
  lcd.drawText(6, y, "RX RNS: " .. string.sub(state.rx_dest_hex, 1, 16), 0)
  y = y + dy
  lcd.drawText(6, y, "        " .. string.sub(state.rx_dest_hex, 17, 32), 0)
  y = y + dy

  local a1 = (state.cursor == 1) and INVERS or 0
  local a2 = (state.cursor == 2) and INVERS or 0
  local a3 = (state.cursor == 3) and INVERS or 0
  local tgt = state.ota_rx_direct and "OTA-RX(0xEC)" or "TX-MOD(0xEE)"
  lcd.drawText(6, y, "[Pair & Sync RX]", a1)
  lcd.drawText((LCD_W >= 320) and 165 or 68, y, "[Announce]", a2)
  lcd.drawText((LCD_W >= 320) and 285 or 106, y, "[" .. tgt .. "]", a3)
end

local function drawPage2()
  drawHeader("RNode LoRa Config")
  local y = (LCD_W >= 320) and 32 or 10
  local dy = (LCD_W >= 320) and 22 or 9
  local x2 = (LCD_W >= 320) and 145 or 68

  local fMhz = (FREQ_PRESETS[state.cfg_freq_idx] or state.rnode_freq_hz) / 1000000.0
  local bwKhz = (BW_Presets[state.cfg_bw_idx] or state.rnode_bw_hz) / 1000.0

  local function attr(idx)
    if state.cursor ~= idx then return 0 end
    return state.editing and (INVERS + BLINK) or INVERS
  end

  lcd.drawText(6, y, string.format("1. Freq: %.3f MHz", fMhz), attr(1))
  y = y + dy
  lcd.drawText(6, y, string.format("2. BW:   %.1f kHz", bwKhz), attr(2))
  y = y + dy
  lcd.drawText(6, y, string.format("3. SF: SF%d", state.cfg_sf), attr(3))
  lcd.drawText(x2, y, string.format("4. CR: 4/%d", state.cfg_cr), attr(4))
  y = y + dy
  lcd.drawText(6, y, string.format("5. Pwr: %ddBm", state.cfg_txpwr), attr(5))
  lcd.drawText(x2, y, string.format("6. IFAC: %s", state.cfg_ifac == 1 and "ON" or "OFF"), attr(6))
  y = y + dy
  lcd.drawText(6, y, "[Apply & Push OTA to RX]", attr(7))
end

local function drawPage3()
  drawHeader("Mesh Peers (" .. state.peer_count .. ")")
  local y = (LCD_W >= 320) and 32 or 10
  local dy = (LCD_W >= 320) and 22 or 9

  if state.peer_count == 0 then
    lcd.drawText(6, y, "No Reticulum peers discovered yet.", 0)
    lcd.drawText(6, y + dy, "Press [Pair & Sync RX] on Page 1.", 0)
    return
  end

  for i = 0, state.peer_count - 1 do
    local p = state.peers[i]
    local selMark = (state.selected_peer == i) and "*" or " "
    local rowAttr = (state.cursor == (i + 1)) and INVERS or 0
    if p then
      local rname = ROLE_NAMES[p.role] or "N"
      local shortDest = string.sub(p.dest_hex, 1, 8) .. ".." .. string.sub(p.dest_hex, 29, 32)
      local vStr = string.format("%.1fV", (p.vbatt_mv or 0) / 1000.0)
      lcd.drawText(6, y, string.format("%s%d:%s %s %dh %ddBm %s", selMark, i, rname, shortDest, p.hops, p.rssi_dbm, vStr), rowAttr)
    else
      lcd.drawText(6, y, string.format("%s[%d] querying...", selMark, i), rowAttr)
    end
    y = y + dy
  end
end

local function drawPage4()
  drawHeader("Targeted RX Cmd")
  local y = (LCD_W >= 320) and 32 or 10
  local dy = (LCD_W >= 320) and 22 or 9

  lcd.drawText(6, y, "Target RX: " .. string.sub(state.rx_dest_hex, 1, 16) .. "..", 0)
  y = y + dy
  lcd.drawText(6, y, string.format("Ping RTT:%dms  TunTX:%d TunRX:%d Rly:%d",
    state.last_rtt_ms, state.tun_tx_cnt, state.tun_rx_cnt, state.relay_fwd_cnt), 0)
  y = y + dy

  local btns = {
    { "Ping RX",   ACT_PING_RX   },
    { "LOITER",    ACT_LOITER    },
    { "RTL",       ACT_RTL       },
    { "LAND",      ACT_LAND      },
    { "ARM",       ACT_ARM       },
    { "DISARM",    ACT_DISARM    },
    { "Form:VEE",  ACT_FORM_VEE  },
    { "Form:LINE", ACT_FORM_LINE },
    { "Form:HOLD", ACT_FORM_HOLD }
  }
  local x = 6
  for i = 1, #btns do
    local bAttr = (state.cursor == i) and INVERS or 0
    lcd.drawText(x, y, "[" .. btns[i][1] .. "]", bAttr)
    if i % 3 == 0 then
      x = 6
      y = y + dy
    else
      x = x + ((LCD_W >= 320) and 110 or 40)
    end
  end
end

local function handleEvent(event)
  if event == 0 or not event then return 0 end

  if not state.editing and (event == EVT_VIRTUAL_NEXT_PAGE or event == EVT_PAGE_BREAK) then
    state.page = (state.page % 4) + 1
    state.cursor = 1
    return 0
  elseif not state.editing and event == EVT_VIRTUAL_PREV_PAGE then
    state.page = ((state.page + 2) % 4) + 1
    state.cursor = 1
    return 0
  end

  local maxCursor = 3
  if state.page == 2 then maxCursor = 7
  elseif state.page == 3 then maxCursor = math.max(1, state.peer_count)
  elseif state.page == 4 then maxCursor = 9 end

  if event == EVT_VIRTUAL_NEXT then
    if state.editing and state.page == 2 then
      if state.cursor == 1 then state.cfg_freq_idx = (state.cfg_freq_idx % #FREQ_PRESETS) + 1
      elseif state.cursor == 2 then state.cfg_bw_idx = (state.cfg_bw_idx % #BW_Presets) + 1
      elseif state.cursor == 3 then state.cfg_sf = math.min(12, state.cfg_sf + 1)
      elseif state.cursor == 4 then state.cfg_cr = math.min(8, state.cfg_cr + 1)
      elseif state.cursor == 5 then state.cfg_txpwr = math.min(22, state.cfg_txpwr + 1)
      elseif state.cursor == 6 then state.cfg_ifac = 1 - state.cfg_ifac end
    else
      state.cursor = (state.cursor % maxCursor) + 1
    end
  elseif event == EVT_VIRTUAL_PREV then
    if state.editing and state.page == 2 then
      if state.cursor == 1 then state.cfg_freq_idx = ((state.cfg_freq_idx - 2) % #FREQ_PRESETS) + 1
      elseif state.cursor == 2 then state.cfg_bw_idx = ((state.cfg_bw_idx - 2) % #BW_Presets) + 1
      elseif state.cursor == 3 then state.cfg_sf = math.max(5, state.cfg_sf - 1)
      elseif state.cursor == 4 then state.cfg_cr = math.max(5, state.cfg_cr - 1)
      elseif state.cursor == 5 then state.cfg_txpwr = math.max(-10, state.cfg_txpwr - 1)
      elseif state.cursor == 6 then state.cfg_ifac = 1 - state.cfg_ifac end
    else
      state.cursor = ((state.cursor - 2) % maxCursor) + 1
    end
  elseif event == EVT_VIRTUAL_ENTER then
    if state.page == 1 then
      if state.cursor == 1 then
        sendRnsCmd(SUBCMD_PAIR_RX, nil)
        state.status_msg = "Pairing RX OTA..."
      elseif state.cursor == 2 then
        sendRnsCmd(SUBCMD_ACTION, { ACT_ANNOUNCE })
        state.status_msg = "Announce Sent"
      else
        state.ota_rx_direct = not state.ota_rx_direct
        state.status_msg = state.ota_rx_direct and "Target: OTA RX" or "Target: TX Mod"
      end
    elseif state.page == 2 then
      if state.cursor == 7 then
        pushConfigToModule(true)
      else
        state.editing = not state.editing
      end
    elseif state.page == 3 and state.peer_count > 0 then
      local sel = state.cursor - 1
      sendRnsCmd(SUBCMD_SELECT_PEER, { sel })
      state.selected_peer = sel
      state.status_msg = "Selected Peer #" .. sel
    elseif state.page == 4 then
      local acts = {
        ACT_PING_RX, ACT_LOITER, ACT_RTL,
        ACT_LAND, ACT_ARM, ACT_DISARM,
        ACT_FORM_VEE, ACT_FORM_LINE, ACT_FORM_HOLD
      }
      local act = acts[state.cursor]
      if act then
        sendRnsCmd(SUBCMD_ACTION, { act })
        state.status_msg = "Sent Action #" .. act
      end
    end
  elseif event == EVT_VIRTUAL_EXIT then
    if state.editing then
      state.editing = false
      return 0
    end
  end

  return 0
end

local function init()
  state.last_poll_ms = 0
  sendRnsCmd(SUBCMD_GET_STATUS, nil)
end

local function run(event)
  handleEvent(event)
  pollTelemetry()
  if state.page == 1 then drawPage1()
  elseif state.page == 2 then drawPage2()
  elseif state.page == 3 then drawPage3()
  else drawPage4() end
  return 0
end

return { init = init, run = run }
