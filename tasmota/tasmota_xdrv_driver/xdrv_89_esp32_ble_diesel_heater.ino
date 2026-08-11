/*
  xdrv_89_esp32_ble_diesel_heater.ino - Chinese diesel heater BLE dashboard sense and control
                                          via BLE_ESP32 support for Tasmota

  Copyright (C) 2026  Les Newell

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

  --------------------------------------------------------------------------------------------
  Version yyyymmdd  Action    Description
  --------------------------------------------------------------------------------------------
  0.0.0.0 20260810  created - initial version. Covers two structurally unrelated heater
                              hardware/protocol families under one command set:
                                - AA55/AA66 (+ their encrypted variants) - Vevor/BYD-style,
                                  protocol ported from github.com/warehog/esphome-diesel-heater-ble
                                - Hcalory MVP1/MVP2 (e.g. HBU1S) - protocol ported from the
                                  ProtocolHcalory class in
                                  github.com/Spettacolo83/homeassistant-diesel-heater
                              Which family (and which sub-variant) a given MAC speaks is
                              detected automatically - see "Protocol detection" below.
*/

/*
Commands (all Tasmota-style: omit the value to query current setting):

DieselHeaterPeriod [n]              - polling period in seconds (0 = off).
                                       Default: device's configured TelePeriod.

DieselHeaterState <mac>             - force an immediate status read (the only query command
                                       that actually hits the wire - see "Caching" below)
DieselHeaterPower <mac> [ON|OFF|TOGGLE]
DieselHeaterMode  <mac> [Temperature|Level|Ventilation]
DieselHeaterTemp  <mac> [0..40]
DieselHeaterLevel <mac> [1..10]
DieselHeaterAuto  <mac> [ON|OFF]    - automatic start/stop. AA55-family sends this as a direct
                                       set; Hcalory's wire command is toggle-only, so for Hcalory
                                       this compares against the last known state and only
                                       sends the toggle when it actually needs to change (see
                                       "Caching" below - this needs a cached state to work from).
DieselHeaterPassword <mac> [0..9999] - Hcalory MVP2 only: PIN sent on the device's next password
                                       handshake (every operation re-sends it - see "Protocol
                                       detection" below on why there's no persistent connection to
                                       piggyback an already-authenticated session on). Doesn't
                                       touch the wire itself.
DieselHeaterError <mac> [<code>]    - error state, from cache (like the other query commands -
                                       never touches the wire on its own). <code> true/on/1
                                       returns the raw numeric code; omitted or anything else
                                       (including false/off/0) returns the description string
                                       (empty when there's no error) - see DHErrorDescription.

<mac> accepts a raw hex address or a BLE_ESP32 alias (BLEAlias <mac>=<name>). The first command
referencing a MAC registers it for periodic polling.

Not every function is meaningful for every protocol - e.g. an AA55-family heater's response
never reports AutoStartStop unless it's the encrypted variant. Unsupported combinations return
a distinct "unsupported" result rather than silently doing nothing.

Protocol detection: on first contact with a MAC, connection is attempted in this order -
Hcalory MVP2 (service 0xBD39), Hcalory MVP1 (0xFFF0), AA55-family (0xFFE0) - matching the
Hcalory reference implementation's own MVP2-first default, and stopping at the first GATT
service that's actually found. AA55 vs AA66 vs their encrypted variants is then determined from
the first response's own header bytes, exactly as the wire protocol already encodes it. Once a
MAC's protocol is confirmed, it's remembered and the cascade is skipped on subsequent commands.

Caching: a query (no value given) always answers from the last cached status, never triggers a
fresh wire read - only DieselHeaterState does that, and the automatic periodic poll. This is
deliberate: reading status is one combined operation returning many fields at once on both
protocols, so per-field wire reads would be wasteful, and issuing several query commands in
quick succession (e.g. from a script) shouldn't each open a new BLE connection.

Write coalescing: repeating the same set-command for the same device (e.g. a GUI slider firing
several DieselHeaterLevel calls as it's dragged) while an earlier call for that same command is
still queued updates that pending call's value in place rather than opening a separate BLE
connection for every intermediate value - see DHQueueOp. Only the most recent value survives;
DieselHeaterState/the automatic poll are unaffected (always a fresh read, per above).

Responses (stat/<topic>/DieselHeater/<mac>):
{
  "cmd":"state",
  "result":"ok",
  "MAC":"001122334455",
  "RSSI":-70,
  "Protocol":"HcaloryMVP2",
  "RunningState":1,
  "RunningStep":3,
  "RunningMode":"Temperature",
  "SetTemp":22,
  "AutoStartStop":false,
  "SupplyVoltage":12.3,
  "CaseTemp":45,
  "CabinTemp":22,
  "TempUnit":0,
  "ErrorCode":0,
  "ErrorText":""
}
"SetTemp" or "SetLevel" appears depending on RunningMode, and neither appears when the heater is
off/turning-off/in-error (the value is meaningless in that state on both protocols - a stale
number would be misleading, so it's omitted rather than shown).
*/

//#define VSCODE_DEV

#ifdef VSCODE_DEV
#define ESP32
#define USE_BLE_ESP32
#define USE_BLE_DIESEL_HEATER
#endif

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32C2 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32P4
#ifdef USE_BLE_DIESEL_HEATER
#ifdef ESP32                       // ESP32 only - no ESP8266 equivalent for this driver
#ifdef USE_BLE_ESP32

#define XDRV_89                    89
#define D_CMND_DIESELHEATER "DieselHeater"

// uncomment for more debug messages
//#define DIESELHEATER_DEBUG

namespace DIESELHEATER_ESP32 {

void CmndDieselHeaterPeriod(void);
void CmndDieselHeaterState(void);
void CmndDieselHeaterPower(void);
void CmndDieselHeaterMode(void);
void CmndDieselHeaterTemp(void);
void CmndDieselHeaterLevel(void);
void CmndDieselHeaterAuto(void);
void CmndDieselHeaterPassword(void);
void CmndDieselHeaterError(void);

const char kDieselHeaterCommands[] = D_CMND_DIESELHEATER"|"
  "Period|"
  "State|"
  "Power|"
  "Mode|"
  "Temp|"
  "Level|"
  "Auto|"
  "Password|"
  "Error";

void (*const DieselHeaterCommands[])(void) = {
  &CmndDieselHeaterPeriod,
  &CmndDieselHeaterState,
  &CmndDieselHeaterPower,
  &CmndDieselHeaterMode,
  &CmndDieselHeaterTemp,
  &CmndDieselHeaterLevel,
  &CmndDieselHeaterAuto,
  &CmndDieselHeaterPassword,
  &CmndDieselHeaterError,
};

/*********************************************************************************************\
 * Protocol identity
\*********************************************************************************************/

enum HeaterProtocol : uint8_t {
  HP_UNKNOWN = 0,
  HP_AA55 = 1,
  HP_AA66 = 2,
  HP_AA55_ENC = 3,
  HP_AA66_ENC = 4,
  HP_HCALORY_MVP1 = 5,
  HP_HCALORY_MVP2 = 6,
};

const char *ProtocolName(uint8_t p) {
  switch (p) {
    case HP_AA55:         return "AA55";
    case HP_AA66:         return "AA66";
    case HP_AA55_ENC:     return "AA55Encrypted";
    case HP_AA66_ENC:     return "AA66Encrypted";
    case HP_HCALORY_MVP1: return "HcaloryMVP1";
    case HP_HCALORY_MVP2: return "HcaloryMVP2";
    default:              return "Unknown";
  }
}
bool IsHcalory(uint8_t p) { return (HP_HCALORY_MVP1 == p) || (HP_HCALORY_MVP2 == p); }
bool IsAA55Family(uint8_t p) { return (p >= HP_AA55) && (p <= HP_AA66_ENC); }

// Error code descriptions, cross-checked against the reference implementation's const.py
// (ERROR_NAMES for AA55-family, HCALORY_ERROR_NAMES for Hcalory - the numeric codes are shared
// across each family's protocol variants, only the byte offset they're read from differs, see
// AA55ParseResponse/HCParseResponse). Not directly confirmed against real hardware - no fault
// has been observed live - so treat these as documentation-derived, not hardware-verified.
// Following this codebase's #ifndef D_X / #define D_X pattern (see e.g. xdrv_81's D_WEBCAM_STATE)
// for Tasmota-translatable strings: a user_config_override.h or a proper language file entry can
// override any of these before this file is compiled.
#ifndef D_DIESELHEATER_AA55_ERR1
#define D_DIESELHEATER_AA55_ERR1  "Startup failure"
#define D_DIESELHEATER_AA55_ERR2  "Lack of fuel"
#define D_DIESELHEATER_AA55_ERR3  "Supply voltage overrun"
#define D_DIESELHEATER_AA55_ERR4  "Outlet sensor fault"
#define D_DIESELHEATER_AA55_ERR5  "Inlet sensor fault"
#define D_DIESELHEATER_AA55_ERR6  "Pulse pump fault"
#define D_DIESELHEATER_AA55_ERR7  "Fan fault"
#define D_DIESELHEATER_AA55_ERR8  "Ignition unit fault"
#define D_DIESELHEATER_AA55_ERR9  "Overheating"
#define D_DIESELHEATER_AA55_ERR10 "Overheat sensor fault"
#endif

#ifndef D_DIESELHEATER_HC_ERR1
#define D_DIESELHEATER_HC_ERR1  "E01 - Ignition failure"
#define D_DIESELHEATER_HC_ERR2  "E02 - Flame out"
#define D_DIESELHEATER_HC_ERR3  "E03 - Overheat"
#define D_DIESELHEATER_HC_ERR4  "E04 - Fan failure"
#define D_DIESELHEATER_HC_ERR5  "E05 - Pump failure"
#define D_DIESELHEATER_HC_ERR6  "E06 - Sensor failure"
#define D_DIESELHEATER_HC_ERR7  "E07 - Low voltage"
#define D_DIESELHEATER_HC_ERR8  "E08 - High voltage"
#define D_DIESELHEATER_HC_ERR9  "E09 - Communication error"
#define D_DIESELHEATER_HC_ERR10 "E10 - CO concentration high"
#define D_DIESELHEATER_HC_ERR11 "E11 - CO concentration critical"
#endif

// Returns "" for no error (code 0) or an unrecognized code, matching DieselHeaterError's
// documented "no error" contract.
const char *DHErrorDescription(uint8_t protocol, uint8_t code) {
  if (IsHcalory(protocol)) {
    switch (code) {
      case 1:  return D_DIESELHEATER_HC_ERR1;
      case 2:  return D_DIESELHEATER_HC_ERR2;
      case 3:  return D_DIESELHEATER_HC_ERR3;
      case 4:  return D_DIESELHEATER_HC_ERR4;
      case 5:  return D_DIESELHEATER_HC_ERR5;
      case 6:  return D_DIESELHEATER_HC_ERR6;
      case 7:  return D_DIESELHEATER_HC_ERR7;
      case 8:  return D_DIESELHEATER_HC_ERR8;
      case 9:  return D_DIESELHEATER_HC_ERR9;
      case 10: return D_DIESELHEATER_HC_ERR10;
      case 11: return D_DIESELHEATER_HC_ERR11;
      default: return "";
    }
  }
  switch (code) {
    case 1:  return D_DIESELHEATER_AA55_ERR1;
    case 2:  return D_DIESELHEATER_AA55_ERR2;
    case 3:  return D_DIESELHEATER_AA55_ERR3;
    case 4:  return D_DIESELHEATER_AA55_ERR4;
    case 5:  return D_DIESELHEATER_AA55_ERR5;
    case 6:  return D_DIESELHEATER_AA55_ERR6;
    case 7:  return D_DIESELHEATER_AA55_ERR7;
    case 8:  return D_DIESELHEATER_AA55_ERR8;
    case 9:  return D_DIESELHEATER_AA55_ERR9;
    case 10: return D_DIESELHEATER_AA55_ERR10;
    default: return "";
  }
}

// user-facing command intents - "verb" in the request builders below
enum DHCmd : uint8_t {
  DHCMD_POLL = 0,      // internal periodic poll, same wire command as STATE, no publish
  DHCMD_STATE = 1,
  DHCMD_POWER = 2,     // argument: 0=off, 1=on, 2=toggle
  DHCMD_MODE = 3,      // argument: 0=level, 1=temperature, 2=ventilation (normalized)
  DHCMD_TEMP = 4,      // argument: target temperature
  DHCMD_LEVEL = 5,     // argument: target level
  DHCMD_AUTO = 6,      // argument: 0=off, 1=on
  DHCMD_PASSWORD = 7,  // internal, Hcalory MVP2 only, no publish
};
const char *cmdnames[] = { "poll", "state", "power", "mode", "temp", "level", "auto", "password" };

/*********************************************************************************************\
 * AA55-family: request/response, ported from github.com/warehog/esphome-diesel-heater-ble
 * (components/diesel_heater_ble/{messages,state}.h)
\*********************************************************************************************/

const char AA55_Svc[] = "0000ffe0-0000-1000-8000-00805f9b34fb";
const char AA55_Char[] = "0000ffe1-0000-1000-8000-00805f9b34fb"; // write + notify, same characteristic

struct aa55_state_t {
  uint8_t rcvCmd;
  uint8_t runningState;
  uint8_t errCode;
  uint8_t runningStep;
  uint16_t altitude;
  uint8_t runningMode;      // raw: 0/1=level, 2=temperature+level (see field notes in DHParseResponseAA55)
  uint8_t setLevel;
  uint8_t setTemp;
  float supplyVoltage;
  uint16_t caseTemp;
  uint16_t cabTemp;

  bool hasExtras;           // encrypted variants only
  uint16_t stTime;
  uint16_t autoTime;
  uint16_t runTime;
  uint8_t isAuto;
  uint8_t language;
  uint8_t tempOffset;
  uint8_t tankVolume;
  uint8_t oilPumpType;
  bool rf433OnOff;
  uint8_t tempUnit;
  uint8_t altiUnit;
  uint8_t automaticHeating;
};

uint8_t AA55DetectClass(const uint8_t *raw, uint8_t len) {
  if (len < 2) return HP_UNKNOWN;
  switch (raw[1]) {
    case 0x55: return HP_AA55;
    case 0x66: return HP_AA66;
    case 0x34: return HP_AA55_ENC;
    case 0x07: return HP_AA66_ENC;
    default:   return HP_UNKNOWN;
  }
}

// XORs (up to) the first 48 bytes, 8 at a time, against ASCII "password" - matches
// ResponseParser::decrypt() exactly, including its "already 0xAA -> no-op" fast path.
void AA55Decrypt(const uint8_t *raw, uint8_t len, uint8_t *out) {
  memcpy(out, raw, len);
  if (raw[0] == 0xAA) return;
  static const uint8_t key[8] = {112, 97, 115, 115, 119, 111, 114, 100}; // "password"
  uint8_t n = (len < 48) ? len : 48;
  for (uint8_t i = 0; i < n; i++) {
    out[i] = (uint8_t)(raw[i] ^ key[i % 8]);
  }
}

// explicit prototype: Arduino's auto-prototype generator scans the concatenated source and
// inserts forward declarations before locally-defined types like aa55_state_t/hc_state_t are
// defined, without reliably tracking namespace/preprocessor-guard context - see xdrv_79's
// CONFIG_NIMBLE_CPP_IDF fix and this project's own history for the same class of bug.
bool AA55ParseResponse(const uint8_t *raw, uint8_t len, aa55_state_t &st);

bool AA55ParseResponse(const uint8_t *raw, uint8_t len, aa55_state_t &st) {
  uint8_t hc = AA55DetectClass(raw, len);
  if (HP_UNKNOWN == hc) return false;

  uint8_t decrypted[48];
  uint8_t dlen = (len < 48) ? len : 48;
  AA55Decrypt(raw, dlen, decrypted);

  memset(&st, 0, sizeof(st));

  if (dlen > 2) st.rcvCmd = decrypted[2];
  if (dlen > 3) st.runningState = decrypted[3];

  // AA66 (unencrypted) error code offset was wrongly 17 - cross-checked against the reference
  // implementation's ProtocolAA55/AA66/AA55Encrypted/AA66Encrypted classes: AA55, AA66 and AA55
  // encrypted all read the error code from byte 4; only AA66 encrypted differs (byte 35).
  if ((HP_AA55 == hc || HP_AA66 == hc || HP_AA55_ENC == hc) && dlen > 4) {
    st.errCode = decrypted[4];
  } else if (HP_AA66_ENC == hc && dlen > 35) {
    st.errCode = decrypted[35];
  }

  if (dlen > 5) st.runningStep = decrypted[5];

  if ((HP_AA55 == hc || HP_AA66 == hc) && dlen > 7) {
    st.altitude = decrypted[6] + (decrypted[7] << 8);
  } else if ((HP_AA55_ENC == hc || HP_AA66_ENC == hc) && dlen > 7) {
    st.altitude = (decrypted[7] + (decrypted[6] << 8)) / 10;
  }

  if (dlen > 8) st.runningMode = decrypted[8];

  if ((HP_AA55 == hc || HP_AA66 == hc) && dlen > 10) {
    if (0x00 == st.runningMode) {
      st.setLevel = decrypted[10] + 1;
    } else if (0x01 == st.runningMode) {
      st.setLevel = decrypted[9];
    } else if (0x02 == st.runningMode) {
      st.setTemp = decrypted[9];
      st.setLevel = decrypted[10] + 1;
    }
  } else if ((HP_AA55_ENC == hc || HP_AA66_ENC == hc) && dlen > 10) {
    st.setLevel = decrypted[10];
    st.setTemp = decrypted[9];
  }

  if ((HP_AA55 == hc || HP_AA66 == hc) && dlen > 12) {
    st.supplyVoltage = (float)(decrypted[11] + (decrypted[12] << 8)) / 10.0f;
  } else if ((HP_AA55_ENC == hc || HP_AA66_ENC == hc) && dlen > 12) {
    st.supplyVoltage = (float)(decrypted[12] + (decrypted[11] << 8)) / 10.0f;
  }

  if ((HP_AA55 == hc || HP_AA66 == hc) && dlen > 16) {
    st.caseTemp = decrypted[13] + (decrypted[14] << 8);
    st.cabTemp = decrypted[15] + (decrypted[16] << 8);
  } else if ((HP_AA55_ENC == hc || HP_AA66_ENC == hc) && dlen > 33) {
    st.caseTemp = decrypted[14] + (decrypted[13] << 8);
    st.cabTemp = (decrypted[33] + (decrypted[32] << 8)) / 10;
  }

  st.hasExtras = false;
  if ((HP_AA55_ENC == hc || HP_AA66_ENC == hc) && dlen > 34) {
    st.stTime = decrypted[20] + (decrypted[19] << 8);
    st.autoTime = decrypted[22] + (decrypted[21] << 8);
    st.runTime = decrypted[24] + (decrypted[23] << 8);
    st.isAuto = decrypted[25];
    st.language = decrypted[26];
    st.tempOffset = decrypted[34];
    st.tankVolume = decrypted[28];
    st.oilPumpType = decrypted[29];
    if (29 < len) {
      if (20 == raw[29]) st.rf433OnOff = false;
      else if (21 == raw[29]) st.rf433OnOff = true;
    }
    st.tempUnit = decrypted[27];
    st.altiUnit = decrypted[30];
    st.automaticHeating = decrypted[31];
    st.hasExtras = true;
  }

  return true;
}

void AA55BuildRequest(uint8_t *buf, uint8_t cmd, uint8_t d1, uint8_t d2) {
  buf[0] = 0xAA; buf[1] = 0x55; buf[2] = 0x0C; buf[3] = 0x22;
  buf[4] = cmd; buf[5] = d1; buf[6] = d2;
  buf[7] = (uint8_t)((0x0C + 0x22 + cmd + d1 + d2) % 256);
}

/*********************************************************************************************\
 * Hcalory MVP1/MVP2: request/response, ported from ProtocolHcalory in
 * github.com/Spettacolo83/homeassistant-diesel-heater
 * (diesel_heater_ble/src/diesel_heater_ble/protocol.py - not that repo's own HCALORY.md doc
 * page, which disagrees with the real code on gear-level range, checksum span, and the entire
 * response byte layout - see this file's version-history header)
\*********************************************************************************************/

const char HC_MVP2_Svc[] = "0000bd39-0000-1000-8000-00805f9b34fb";
const char HC_MVP2_Write_Char[] = "0000bdf7-0000-1000-8000-00805f9b34fb";
const char HC_MVP2_Notify_Char[] = "0000bdf8-0000-1000-8000-00805f9b34fb";

const char HC_MVP1_Svc[] = "0000fff0-0000-1000-8000-00805f9b34fb";
const char HC_MVP1_Write_Char[] = "0000fff2-0000-1000-8000-00805f9b34fb";
const char HC_MVP1_Notify_Char[] = "0000fff1-0000-1000-8000-00805f9b34fb";

#define HC_CMD_SET_GEAR     0x0607
#define HC_CMD_SET_TEMP     0x0706
#define HC_CMD_POWER        0x0E04
#define HC_POWER_QUERY       0x00
#define HC_POWER_ON          0x02
#define HC_POWER_OFF         0x01
#define HC_POWER_AUTO_TOGGLE 0x05
#define HC_POWER_MODE_LEVEL  0x07
#define HC_POWER_MODE_TEMP   0x06

struct hc_state_t {
  uint8_t runningState;
  uint8_t runningStep;
  uint8_t setMode;         // raw: 0=off,1=temperature,2=level,3=ventilation
  bool setValueNone;
  uint8_t setTemp;
  uint8_t setLevel;
  bool autoStartStop;
  float supplyVoltage;
  uint16_t caseTemp;
  uint16_t cabTemp;
  uint8_t highAltitude;
  uint8_t tempUnit;
  uint8_t errorCode;
};

bool HCParseResponse(const uint8_t *data, uint8_t len, hc_state_t &st);

bool HCParseResponse(const uint8_t *data, uint8_t len, hc_state_t &st) {
  if (len < 38) return false;

  memset(&st, 0, sizeof(st));

  uint8_t complete_state_byte = data[20];
  uint8_t status = (complete_state_byte & 0xF0) >> 4;
  uint8_t running_step_raw = complete_state_byte & 0x0F;

  st.runningState = (0x0 == status || 0xF == status) ? 0 : 1;

  if (0x4 == status) {
    st.runningStep = 4; // synthetic cooldown, matches reference exactly
  } else {
    switch (running_step_raw) {
      case 0x1: st.runningStep = 6; break; // fan/ventilation
      case 0x3: st.runningStep = 2; break; // ignition
      case 0x5: st.runningStep = 3; break; // running
      case 0x7: st.runningStep = 0; break; // standby
      default:  st.runningStep = 0; break; // inactive/standby
    }
  }

  st.setMode = data[21];

  uint8_t set_value_raw = data[22];
  if (0x0 == status || 0x4 == status || 0xF == status) {
    st.setValueNone = true;
  } else if (0x1 == st.setMode) {
    st.setTemp = set_value_raw;
  } else {
    uint8_t lvl = set_value_raw;
    if (lvl < 1) lvl = 1;
    if (lvl > 10) lvl = 10;
    st.setLevel = lvl;
  }

  st.autoStartStop = (1 == data[23]);
  st.supplyVoltage = (float)(((uint16_t)data[24] << 8) | data[25]) / 10.0f;
  st.caseTemp = (((uint16_t)data[27] << 8) | data[28]) / 10;
  st.cabTemp = (((uint16_t)data[30] << 8) | data[31]) / 10;
  st.highAltitude = data[18];
  st.tempUnit = data[37];
  st.errorCode = (0xF == status) ? set_value_raw : 0;

  return true;
}

// general command frame: 00 02 00 01 00 01 00 [cmd_hi] [cmd_lo] 00 00 [payload_len] [payload...] [checksum]
// checksum = sum(bytes from index 8 onward) & 0xFF - NOT the whole frame
uint8_t HCBuildCmd(uint8_t *buf, uint16_t cmd_type, const uint8_t *payload, uint8_t payload_len) {
  uint8_t cmd_hi = (cmd_type >> 8) & 0xFF;
  uint8_t cmd_lo = cmd_type & 0xFF;

  buf[0] = 0x00; buf[1] = 0x02;
  buf[2] = 0x00; buf[3] = 0x01;
  buf[4] = 0x00; buf[5] = 0x01;
  buf[6] = 0x00; buf[7] = cmd_hi;
  buf[8] = cmd_lo;
  buf[9] = 0x00; buf[10] = 0x00;
  buf[11] = payload_len;
  for (uint8_t i = 0; i < payload_len; i++) buf[12 + i] = payload[i];

  uint16_t sum = 0;
  for (uint8_t i = 8; i < 12 + payload_len; i++) sum += buf[i];
  buf[12 + payload_len] = (uint8_t)(sum & 0xFF);

  return 12 + payload_len + 1;
}

// MVP2 query-with-timestamp: 00 02 00 01 00 01 00 0A 0A 00 00 05 [HH MM SS DOW] 00 [checksum]
uint8_t HCBuildMvp2QueryCmd(uint8_t *buf) {
  buf[0] = 0x00; buf[1] = 0x02;
  buf[2] = 0x00; buf[3] = 0x01;
  buf[4] = 0x00; buf[5] = 0x01;
  buf[6] = 0x00; buf[7] = 0x0A; buf[8] = 0x0A; buf[9] = 0x00;
  buf[10] = 0x00; buf[11] = 0x05;

  // RtcTime is Tasmota's already-maintained current local time (support_rtc.ino). Its
  // day_of_week uses Tasmota's own convention (1=Sunday), not the protocol's ISO weekday
  // (1=Monday..7=Sunday) - convert.
  buf[12] = RtcTime.hour;
  buf[13] = RtcTime.minute;
  buf[14] = RtcTime.second;
  buf[15] = (1 == RtcTime.day_of_week) ? 7 : (RtcTime.day_of_week - 1);
  buf[16] = 0x00;

  uint16_t sum = 0;
  for (uint8_t i = 8; i < 17; i++) sum += buf[i];
  buf[17] = (uint8_t)(sum & 0xFF);

  return 18;
}

// MVP2 password handshake: 00 02 00 01 00 01 00 0A 0C 00 00 05 01 [D1 D2 D3 D4] [checksum]
// PIN not currently exposed as a command - hardcoded to the reference implementation's own
// default of 1234, which is also the factory-default PIN on an unconfigured heater.
uint8_t HCBuildPasswordHandshake(uint8_t *buf, uint16_t passkey = 0) {
  buf[0] = 0x00; buf[1] = 0x02;
  buf[2] = 0x00; buf[3] = 0x01;
  buf[4] = 0x00; buf[5] = 0x01;
  buf[6] = 0x00; buf[7] = 0x0A;
  buf[8] = 0x0C; buf[9] = 0x00; buf[10] = 0x00;
  buf[11] = 0x05;
  buf[12] = 0x01;
  uint16_t pk = passkey;
  for (int8_t i = 3; i >= 0; i--) {
    buf[13 + i] = pk % 10;
    pk /= 10;
  }

  uint16_t sum = 0;
  for (uint8_t i = 8; i < 17; i++) sum += buf[i];
  buf[17] = (uint8_t)(sum & 0xFF);

  return 18;
}

/*********************************************************************************************\
 * Unified device table
\*********************************************************************************************/

#define DIESELHEATER_NUM_DEVICESLOTS 4
#define DIESELHEATER_RETRIES 4
// largest of the AA55 (8), Hcalory password handshake (18), and Hcalory HC_CMD_POWER frame
// shapes (12-byte header + 9-byte payload + 1-byte checksum = 22) - was wrongly 18 (a real stack
// buffer overflow: every POWER/STATE/MODE/AUTO frame on Hcalory overran this by 4 bytes,
// corrupting the query byte and checksum on every single one of those commands).
#define DIESELHEATER_FRAME_MAX 22

struct dh_device_tag {
  uint8_t addr[7];
  bool known;
  uint8_t protocol;   // HeaterProtocol - HP_UNKNOWN until the detection cascade confirms it
  int8_t RSSI;
  bool stateValid;
  uint16_t hcaloryPasskey; // Hcalory MVP2 only - PIN used to seed the first verify attempt of a
                           // chained op (see DHMvp2ChainCallback), set via DieselHeaterPassword;
                           // 0 (the zero-init default) is itself a real PIN some devices use, not
                           // a "no PIN" sentinel.
  union {
    aa55_state_t aa55;
    hc_state_t hcalory;
  } state;
} DieselHeaterDevices[DIESELHEATER_NUM_DEVICESLOTS];

int DieselHeaterPeriod = 300;
int seconds = 20;
int nextPoll = DIESELHEATER_NUM_DEVICESLOTS;
int opInProgress = 0;
int retries = 0;

#pragma pack( push, 1 )
struct op_t {
  uint8_t addr[7];
  uint8_t cmd;          // DHCmd - the user's intent, not raw wire bytes, so it can be rebuilt
  int16_t argument;     // meaning depends on cmd
  uint8_t protocolTried; // which candidate this attempt used - HP_UNKNOWN means "not yet tried"
};
#pragma pack(pop)

std::deque<DIESELHEATER_ESP32::op_t*> opQueue;

// explicit prototypes: same Arduino auto-prototype pitfall hit twice already in this codebase
// (xdrv_79's CONFIG_NIMBLE_CPP_IDF fix, and AA55ParseResponse/HCParseResponse above) - these
// three all take dh_device_tag directly (by pointer) in their signature. Text must match the
// real definitions exactly (no "struct" elaborated-type-specifier here, unlike the definitions
// don't use one either) or Arduino's ctags-based scanner won't recognize this as the same
// function and will still auto-generate its own broken one.
dh_device_tag *findOrRegisterDevice(const uint8_t *addr);
uint8_t BuildFrame(uint8_t protocolCandidate, uint8_t cmd, int argument, const dh_device_tag *dev, uint8_t *buf);
bool DHOperation(const uint8_t *MAC, uint8_t cmd, int argument, uint8_t protocolCandidate, const dh_device_tag *dev);

const char *addrStr(const uint8_t *addr) {
  static char addrstr[32];
  BLE_ESP32::dump(addrstr, 13, addr, 6);
  return addrstr;
}

dh_device_tag *findOrRegisterDevice(const uint8_t *addr) {
  int free = -1;
  for (int i = 0; i < DIESELHEATER_NUM_DEVICESLOTS; i++) {
    if (DieselHeaterDevices[i].known && !memcmp(DieselHeaterDevices[i].addr, addr, 6)) {
      return &DieselHeaterDevices[i];
    }
    if (!DieselHeaterDevices[i].known && (free == -1)) {
      free = i;
    }
  }
  if (free == -1) {
    AddLog(LOG_LEVEL_ERROR, PSTR("DHT: %s: lost > %d devices"), addrStr(addr), DIESELHEATER_NUM_DEVICESLOTS);
    return nullptr;
  }
  memset(&DieselHeaterDevices[free], 0, sizeof(dh_device_tag));
  memcpy(DieselHeaterDevices[free].addr, addr, 6);
  DieselHeaterDevices[free].known = true;
  DieselHeaterDevices[free].protocol = HP_UNKNOWN;
  return &DieselHeaterDevices[free];
}

// candidates tried in order for a device whose protocol isn't yet confirmed - Hcalory MVP2
// first (matching the reference implementation's own default), then MVP1, then AA55-family.
// AA55 vs AA66 vs encrypted is refined from the response afterwards, not at this stage.
const uint8_t protocolCascade[] = { HP_HCALORY_MVP2, HP_HCALORY_MVP1, HP_AA55 };
#define PROTOCOL_CASCADE_LEN (sizeof(protocolCascade) / sizeof(*protocolCascade))

// builds the wire frame for a given (cmd, argument) as it would be sent to a specific protocol
// candidate. Returns frame length, or 0 if this cmd isn't supported by that protocol (nothing
// currently returns 0 - kept for when a future protocol doesn't support everything).
uint8_t BuildFrame(uint8_t protocolCandidate, uint8_t cmd, int argument, const dh_device_tag *dev, uint8_t *buf) {
  if (IsAA55Family(protocolCandidate) || HP_AA55 == protocolCandidate) {
    switch (cmd) {
      case DHCMD_POLL:
      case DHCMD_STATE:
        AA55BuildRequest(buf, 0x01, 0, 0);
        return 8;
      case DHCMD_POWER:
        AA55BuildRequest(buf, 0x03, (1 == argument) ? 1 : 0, 0);
        return 8;
      case DHCMD_MODE: {
        // AA55 mode command (0x02) takes a raw runningmode value; normalized argument here is
        // 0=level,1=temperature,2=ventilation - AA55-family has no explicit ventilation mode in
        // the reference protocol, so that combination isn't supported.
        if (2 == argument) return 0;
        AA55BuildRequest(buf, 0x02, (1 == argument) ? 2 : 0, 0);
        return 8;
      }
      case DHCMD_TEMP: {
        int t = argument;
        if (t < 0) t = 0;
        if (t > 40) t = 40;
        AA55BuildRequest(buf, 0x04, (uint8_t)t, 0);
        return 8;
      }
      case DHCMD_LEVEL: {
        int l = argument;
        if (l < 1) l = 1;
        if (l > 10) l = 10;
        AA55BuildRequest(buf, 0x04, (uint8_t)(l - 1), 0);
        return 8;
      }
      case DHCMD_AUTO:
        // AA55's automatic start/stop (0x13) is a direct set, unlike Hcalory's toggle
        AA55BuildRequest(buf, 0x13, (1 == argument) ? 1 : 0, 0);
        return 8;
      default:
        return 0;
    }
  }

  if (IsHcalory(protocolCandidate)) {
    switch (cmd) {
      case DHCMD_PASSWORD:
        return HCBuildPasswordHandshake(buf, dev ? dev->hcaloryPasskey : 0);
      case DHCMD_POLL:
      case DHCMD_STATE: {
        // Real capture evidence (issue #34): 0x0A0A is time-sync only - the actual full status
        // reply on MVP2 comes from the same 0x0E04 query MVP1 uses, not from 0x0A0A.
        uint8_t payload[9] = {0,0,0,0,0,0,0,0, HC_POWER_QUERY};
        return HCBuildCmd(buf, HC_CMD_POWER, payload, sizeof(payload));
      }
      case DHCMD_POWER: {
        uint8_t arg = (2 == argument) ? HC_POWER_QUERY /* no hw toggle for power - treat as query, see note below */
                    : (1 == argument) ? HC_POWER_ON : HC_POWER_OFF;
        uint8_t payload[9] = {0,0,0,0,0,0,0,0, arg};
        return HCBuildCmd(buf, HC_CMD_POWER, payload, sizeof(payload));
      }
      case DHCMD_MODE: {
        if (2 == argument) return 0; // no explicit ventilation-mode set command on Hcalory either
        uint8_t arg = (1 == argument) ? HC_POWER_MODE_TEMP : HC_POWER_MODE_LEVEL;
        uint8_t payload[9] = {0,0,0,0,0,0,0,0, arg};
        return HCBuildCmd(buf, HC_CMD_POWER, payload, sizeof(payload));
      }
      case DHCMD_TEMP: {
        int t = argument;
        if (t < 0) t = 0;
        if (t > 40) t = 40;
        uint8_t unitByte = (dev && dev->stateValid) ? dev->state.hcalory.tempUnit : 0;
        uint8_t payload[2] = {(uint8_t)t, unitByte};
        return HCBuildCmd(buf, HC_CMD_SET_TEMP, payload, sizeof(payload));
      }
      case DHCMD_LEVEL: {
        int l = argument;
        if (l < 1) l = 1;
        if (l > 10) l = 10;
        uint8_t payload[1] = {(uint8_t)l};
        return HCBuildCmd(buf, HC_CMD_SET_GEAR, payload, sizeof(payload));
      }
      case DHCMD_AUTO: {
        // Hcalory's wire command is toggle-only - only actually send it if the requested state
        // differs from the last known one. Needs a valid cached state to work from.
        if (!dev || !dev->stateValid) return 0;
        bool want = (1 == argument);
        if (want == dev->state.hcalory.autoStartStop) return 0; // already in the desired state
        uint8_t payload[9] = {0,0,0,0,0,0,0,0, HC_POWER_AUTO_TOGGLE};
        return HCBuildCmd(buf, HC_CMD_POWER, payload, sizeof(payload));
      }
      default:
        return 0;
    }
  }

  return 0;
}

void UUIDsFor(uint8_t protocol, const char **svc, const char **writeChar, const char **notifyChar) {
  switch (protocol) {
    case HP_HCALORY_MVP2:
      *svc = HC_MVP2_Svc; *writeChar = HC_MVP2_Write_Char; *notifyChar = HC_MVP2_Notify_Char;
      break;
    case HP_HCALORY_MVP1:
      *svc = HC_MVP1_Svc; *writeChar = HC_MVP1_Write_Char; *notifyChar = HC_MVP1_Notify_Char;
      break;
    default: // AA55 family - single characteristic for write and notify
      *svc = AA55_Svc; *writeChar = AA55_Char; *notifyChar = AA55_Char;
      break;
  }
}

int DHGenericOpCompleteFn(BLE_ESP32::generic_sensor_t *pStruct);
void DHPublishNoOp(const uint8_t *addr, const char *cmdName);
void DHPublish(const uint8_t *addr, const char *cmdName, bool success);
bool DHNotifyAcceptMvp2Status(BLE_ESP32::generic_sensor_t *op);
bool DHMvp2ChainCallback(BLE_ESP32::generic_sensor_t *op);

// Hcalory MVP2 pushes a short (~19 byte) ack notify before the real status notify - the ack
// alone isn't enough for HCParseResponse (needs 38+ bytes), so for status queries we tell the
// BLE framework to keep waiting rather than accept the first (ack) notify as final.
bool DHNotifyAcceptMvp2Status(BLE_ESP32::generic_sensor_t *op) {
  return op->notifylen >= 38;
}

// Scratch state for the in-flight MVP2 chained op (verify -> real command, one connection).
// A single file-static instance matches this framework's current one-op-in-flight-at-a-time
// model (see BLEOperationTask) - if that ever changes, this needs to move to per-op storage.
// Deliberately holds NO pointers into dh_device_tag/opQueue: DHMvp2ChainCallback runs on the BLE
// task thread, not the main thread, so it must not touch driver state those are read/written
// from (see DHQueueOp's comment on why chaining exists at all).
struct {
  uint8_t stage;             // 0 = verifying password, 1 = real command sent
  uint8_t realFrame[DIESELHEATER_FRAME_MAX];
  uint8_t realFrameLen;
  uint8_t verifyAttempts;
} dhChain;

// chainnextcallback for the MVP2 flow - see the comment on chainnextcallback in xdrv_79 for the
// contract. Called on the BLE task thread right after each accepted notify.
bool DHMvp2ChainCallback(BLE_ESP32::generic_sensor_t *op) {
  if (0 == dhChain.stage) {
    // this is the heater's password-announce/verify-response notify (see file header "Protocol
    // detection") - too short to be anything else is treated the same as "not authenticated yet".
    if (op->notifylen >= 18 && 1 == op->dataNotify[17]) {
      // authenticated - send the real command next, and now require a real (38+ byte) reply.
      memcpy(op->dataToWrite, dhChain.realFrame, dhChain.realFrameLen);
      op->writelen = dhChain.realFrameLen;
      op->notifyacceptcallback = (void *)DHNotifyAcceptMvp2Status;
      dhChain.stage = 1;
      return true;
    }
    // not authenticated - learn the password the heater just told us and re-verify with it.
    // Bounded so a malformed/stuck exchange still finishes (as a failure) instead of looping.
    const uint8_t *n = op->dataNotify;
    if (dhChain.verifyAttempts >= 5 || op->notifylen < 18
        || n[13] > 9 || n[14] > 9 || n[15] > 9 || n[16] > 9) {
      return false; // give up chaining - finishes/reports normally with whatever we have
    }
    uint16_t learned = (uint16_t)(n[13] * 1000 + n[14] * 100 + n[15] * 10 + n[16]);
    op->writelen = HCBuildPasswordHandshake(op->dataToWrite, learned);
    dhChain.verifyAttempts++;
    return true;
  }
  // stage 1: notifyacceptcallback already required this to be a real (38+ byte) reply - done.
  return false;
}

bool DHOperation(const uint8_t *MAC, uint8_t cmd, int argument, uint8_t protocolCandidate, const dh_device_tag *dev) {
  uint8_t frame[DIESELHEATER_FRAME_MAX];
  uint8_t frameLen;
  // MVP2 needs the chained verify-then-real-command flow (see DHQueueOp/DHMvp2ChainCallback) for
  // every real command; DHCMD_PASSWORD itself (no longer queued externally, but BuildFrame still
  // supports it) stays a plain one-shot op.
  bool chained = (HP_HCALORY_MVP2 == protocolCandidate) && (DHCMD_PASSWORD != cmd);

  if (chained) {
    dhChain.stage = 0;
    dhChain.verifyAttempts = 0;
    dhChain.realFrameLen = BuildFrame(protocolCandidate, cmd, argument, dev, dhChain.realFrame);
    if (0 == dhChain.realFrameLen) return false; // not supported / no-op for this cmd
    frameLen = HCBuildPasswordHandshake(frame, dev ? dev->hcaloryPasskey : 0);
  } else {
    frameLen = BuildFrame(protocolCandidate, cmd, argument, dev, frame);
    if (0 == frameLen) return false; // not supported / no-op (e.g. auto toggle already in desired state)
  }

  BLE_ESP32::generic_sensor_t *op = nullptr;
  int res = BLE_ESP32::newOperation(&op);
  if (!res) {
    AddLog(LOG_LEVEL_ERROR, PSTR("DHT: %s: Can't get a newOperation \"%s\" from BLE"), addrStr(MAC), cmdnames[cmd]);
    return false;
  }

  const char *svc, *writeChar, *notifyChar;
  UUIDsFor(protocolCandidate, &svc, &writeChar, &notifyChar);

  NimBLEAddress addr((uint8_t *)MAC, 0); // type 0 is public
  op->addr = addr;
  op->serviceUUID = NimBLEUUID(svc);
  op->characteristicUUID = NimBLEUUID(writeChar);
  op->notificationCharacteristicUUID = NimBLEUUID(notifyChar);

  op->writelen = frameLen;
  memcpy(op->dataToWrite, frame, frameLen);

  op->completecallback = (void *)DHGenericOpCompleteFn;
  // pack cmd (low byte), protocol candidate tried (next byte), and argument (top 16 bits) into context
  op->context = (void *)(uint32_t)(cmd | ((uint32_t)protocolCandidate << 8) | ((uint32_t)(uint16_t)argument << 16));

  if (chained) {
    op->chainnextcallback = (void *)DHMvp2ChainCallback;
  }

  res = BLE_ESP32::extQueueOperation(&op);
  if (!res) {
    BLE_ESP32::freeOperation(&op);
    AddLog(LOG_LEVEL_ERROR, PSTR("DHT: %s: Failed to queue new operation \"%s\" - deleted"), addrStr(MAC), cmdnames[cmd]);
    return false;
  }
  return true;
}

int DHDoOp() {
  if (opInProgress || !opQueue.size()) return 0;

  op_t* op = opQueue[0];
  dh_device_tag *dev = findOrRegisterDevice(op->addr);

  uint8_t candidate;
  if (dev && (HP_UNKNOWN != dev->protocol)) {
    candidate = dev->protocol;
  } else {
    candidate = protocolCascade[0]; // start of the detection cascade
  }

  if (DHOperation(op->addr, op->cmd, op->argument, candidate, dev)) {
    op->protocolTried = candidate;
    opInProgress = 1;
    retries = DIESELHEATER_RETRIES;
    // op stays in the queue (still opQueue[0]) until the completion callback pops it, so a
    // retry/next-candidate attempt can find it again - see DHGenericOpCompleteFn.
    return 1;
  } else {
    // BuildFrame returned "no-op" (unsupported combo, or an auto-toggle already at the desired
    // state) - nothing to send, so just drop this op without touching the BLE stack at all.
    opQueue.pop_front();
    if (DHCMD_AUTO == op->cmd) {
      // "no-op" here specifically means "already in the requested state" - that's success, not
      // failure, so publish accordingly rather than leaving the caller's command hanging.
      DHPublishNoOp(op->addr, cmdnames[op->cmd]);
    } else {
      AddLog(LOG_LEVEL_ERROR, PSTR("DHT: %s: \"%s\" not supported for this heater's protocol"), addrStr(op->addr), cmdnames[op->cmd]);
    }
    delete op;
    return DHDoOp(); // try the next queued op, if any
  }
}

int DHQueueOp(const uint8_t *MAC, uint8_t cmd, int argument) {
  // MVP2's password handshake can't be sent once per session (BLETaskRunTaskDoneOperation
  // disconnects after every single operation, confirmed by reading it - there's no persistent
  // connection to piggyback on) AND authentication itself doesn't survive a disconnect either
  // (confirmed live: a separate pre-op password op authenticates fine, but the next op's fresh
  // connection comes back unauthenticated again). So for the MVP2 candidate specifically,
  // DHOperation builds a single chained op instead - verify (learning the password from the
  // heater's own announcement if needed) then the real command, all on one connection. See
  // DHMvp2ChainCallback.

  // Coalesce rapid repeats of the same set-command for the same device (e.g. a GUI slider
  // firing several DieselHeaterLevel calls as it's dragged) into the still-pending queue entry,
  // instead of opening a separate BLE connection for every intermediate value. STATE/POLL/
  // PASSWORD are excluded - DieselHeaterState is documented to always force a fresh read, and
  // POLL/PASSWORD aren't user-facing set commands.
  // opQueue[0] is skipped whenever an operation is in flight (opInProgress) - its wire frame is
  // already built from a snapshot of argument taken at dispatch time, so mutating it now
  // wouldn't reach the heater; opInProgress==0 implies opQueue is empty (DHDoOp dispatches
  // synchronously on every push/pop), so this is only ever a no-op in that case, not unsafe.
  bool coalescible = (DHCMD_STATE != cmd) && (DHCMD_POLL != cmd) && (DHCMD_PASSWORD != cmd);
  if (coalescible) {
    for (size_t i = (opInProgress ? 1 : 0); i < opQueue.size(); i++) {
      if (!memcmp(opQueue[i]->addr, MAC, 6) && opQueue[i]->cmd == cmd) {
        opQueue[i]->argument = argument;
        AddLog(LOG_LEVEL_DEBUG, PSTR("DHT: %s: coalesced \"%s\" into pending op"), addrStr(MAC), cmdnames[cmd]);
        return opQueue.size();
      }
    }
  }

  op_t* newop = new op_t;
  memcpy(newop->addr, MAC, 6);
  newop->cmd = cmd;
  newop->argument = argument;
  newop->protocolTried = HP_UNKNOWN;
  opQueue.push_back(newop);
  int qlen = opQueue.size();
  AddLog(LOG_LEVEL_DEBUG, PSTR("DHT: %s: Operation \"%s\" queued - len now %d"), addrStr(MAC), cmdnames[cmd], qlen);
  DHDoOp();
  return qlen;
}

void DHPublishNoOp(const uint8_t *addr, const char *cmdName) {
  // used only for "auto toggle already at requested state" - a successful no-op, not a failure
  ResponseClear();
  ResponseAppend_P(PSTR("{\"cmd\":\"%s\",\"result\":\"ok\",\"MAC\":\"%s\",\"note\":\"already set\"}"), cmdName, addrStr(addr));
  char stopic[TOPSZ];
  GetTopic_P(stopic, STAT, TasmotaGlobal.mqtt_topic, PSTR(""));
  strlcat(stopic, PSTR("DieselHeater/"), sizeof(stopic));
  strlcat(stopic, addrStr(addr), sizeof(stopic));
  MqttPublish(stopic, false);
}

void DHPublish(const uint8_t *addr, const char *cmdName, bool success) {
  ResponseClear();
  ResponseAppend_P(PSTR("{\"cmd\":\"%s\""), cmdName);
  ResponseAppend_P(PSTR(",\"result\":\"%s\""), success ? "ok" : "fail");
  ResponseAppend_P(PSTR(",\"MAC\":\"%s\""), addrStr(addr));

  dh_device_tag *dev = findOrRegisterDevice(addr);
  if (dev) {
    ResponseAppend_P(PSTR(",\"RSSI\":%d"), dev->RSSI);
    if (HP_UNKNOWN != dev->protocol) {
      ResponseAppend_P(PSTR(",\"Protocol\":\"%s\""), ProtocolName(dev->protocol));
    }
    if (dev->stateValid) {
      char fbuf[16];
      if (IsHcalory(dev->protocol)) {
        hc_state_t &st = dev->state.hcalory;
        ResponseAppend_P(PSTR(",\"RunningState\":%d"), st.runningState);
        ResponseAppend_P(PSTR(",\"RunningStep\":%d"), st.runningStep);
        ResponseAppend_P(PSTR(",\"RunningMode\":\"%s\""),
          (0x1 == st.setMode) ? "Temperature" : (0x2 == st.setMode) ? "Level" : (0x3 == st.setMode) ? "Ventilation" : "Off");
        if (!st.setValueNone) {
          if (0x1 == st.setMode) {
            ResponseAppend_P(PSTR(",\"SetTemp\":%d"), st.setTemp);
          } else {
            ResponseAppend_P(PSTR(",\"SetLevel\":%d"), st.setLevel);
          }
        }
        ResponseAppend_P(PSTR(",\"AutoStartStop\":%s"), st.autoStartStop ? "true" : "false");
        dtostrfd(st.supplyVoltage, 1, fbuf);
        ResponseAppend_P(PSTR(",\"SupplyVoltage\":%s"), fbuf);
        ResponseAppend_P(PSTR(",\"CaseTemp\":%d"), st.caseTemp);
        ResponseAppend_P(PSTR(",\"CabinTemp\":%d"), st.cabTemp);
        ResponseAppend_P(PSTR(",\"TempUnit\":%d"), st.tempUnit);
        ResponseAppend_P(PSTR(",\"ErrorCode\":%d"), st.errorCode);
        ResponseAppend_P(PSTR(",\"ErrorText\":\"%s\""), DHErrorDescription(dev->protocol, st.errorCode));
      } else {
        aa55_state_t &st = dev->state.aa55;
        ResponseAppend_P(PSTR(",\"RunningState\":%d"), st.runningState);
        ResponseAppend_P(PSTR(",\"RunningStep\":%d"), st.runningStep);
        ResponseAppend_P(PSTR(",\"RunningMode\":\"%s\""),
          (0x02 == st.runningMode) ? "Temperature" : "Level");
        if (0x02 == st.runningMode) {
          ResponseAppend_P(PSTR(",\"SetTemp\":%d"), st.setTemp);
        }
        ResponseAppend_P(PSTR(",\"SetLevel\":%d"), st.setLevel);
        dtostrfd(st.supplyVoltage, 1, fbuf);
        ResponseAppend_P(PSTR(",\"SupplyVoltage\":%s"), fbuf);
        ResponseAppend_P(PSTR(",\"CaseTemp\":%d"), st.caseTemp);
        ResponseAppend_P(PSTR(",\"CabinTemp\":%d"), st.cabTemp);
        ResponseAppend_P(PSTR(",\"ErrorCode\":%d"), st.errCode);
        ResponseAppend_P(PSTR(",\"ErrorText\":\"%s\""), DHErrorDescription(dev->protocol, st.errCode));
        ResponseAppend_P(PSTR(",\"Altitude\":%d"), st.altitude);
        if (st.hasExtras) {
          ResponseAppend_P(PSTR(",\"AutoStartStop\":%s"), st.isAuto ? "true" : "false");
          ResponseAppend_P(PSTR(",\"TempUnit\":%d"), st.tempUnit);
          ResponseAppend_P(PSTR(",\"TankVolume\":%d"), st.tankVolume);
          ResponseAppend_P(PSTR(",\"OilPumpType\":%d"), st.oilPumpType);
        }
      }
    }
  }
  ResponseAppend_P(PSTR("}"));

  char stopic[TOPSZ];
  GetTopic_P(stopic, STAT, TasmotaGlobal.mqtt_topic, PSTR(""));
  strlcat(stopic, PSTR("DieselHeater/"), sizeof(stopic));
  strlcat(stopic, addrStr(addr), sizeof(stopic));
  MqttPublish(stopic, false);
}

int DHGenericOpCompleteFn(BLE_ESP32::generic_sensor_t *op) {
  opInProgress = 0;

  uint8_t addrev[7];
  const uint8_t *native = op->addr.getVal();
  memcpy(addrev, native, 6);
  BLE_ESP32::ReverseMAC(addrev);

  uint32_t ctx = (uint32_t)op->context;
  uint8_t cmd = ctx & 0xFF;
  uint8_t protocolTried = (ctx >> 8) & 0xFF;
  const char *cmdName = (cmd < (sizeof(cmdnames) / sizeof(*cmdnames))) ? cmdnames[cmd] : "invalid";

  dh_device_tag *dev = findOrRegisterDevice(addrev);
  // the queued op_t (still at opQueue[0]) is what drives retries/cascade - only pop it once we
  // stop needing to try again for it.
  op_t *queued = opQueue.size() ? opQueue[0] : nullptr;

  if (op->state <= GEN_STATE_FAILED) {
    // still probing an unconfirmed protocol - advance to the next cascade candidate rather than
    // just retrying the same (almost certainly wrong) one
    if (dev && (HP_UNKNOWN == dev->protocol)) {
      uint8_t nextCandidate = HP_UNKNOWN;
      for (uint8_t i = 0; i + 1 < PROTOCOL_CASCADE_LEN; i++) {
        if (protocolCascade[i] == protocolTried) { nextCandidate = protocolCascade[i + 1]; break; }
      }
      if (HP_UNKNOWN != nextCandidate && retries > 1) {
        retries--;
        if (DHOperation(addrev, cmd, (int16_t)(ctx >> 16), nextCandidate, dev)) {
          if (queued) queued->protocolTried = nextCandidate;
          AddLog(LOG_LEVEL_DEBUG, PSTR("DHT: %s: %s not found, trying %s"), addrStr(addrev), ProtocolName(protocolTried), ProtocolName(nextCandidate));
          opInProgress = 1;
          return 0;
        }
      }
    }
    if (retries > 1) {
      retries--;
      if (DHOperation(addrev, cmd, (int16_t)(ctx >> 16), protocolTried, dev)) {
        AddLog(LOG_LEVEL_ERROR, PSTR("DHT: %s: operation \"%s\" failed - retries left: %d"), addrStr(addrev), cmdName, retries);
        opInProgress = 1;
        return 0;
      }
    }
    retries = 0;
    if (queued) { opQueue.pop_front(); delete queued; }
    if (DHCMD_PASSWORD != cmd) DHPublish(addrev, cmdName, false);
    AddLog(LOG_LEVEL_ERROR, PSTR("DHT: %s: operation \"%s\" failed - no more retries/candidates"), addrStr(addrev), cmdName);
    DHDoOp();
    return 0;
  }

  retries = 0;
  if (queued) { opQueue.pop_front(); delete queued; }

  if (dev) {
    if (HP_UNKNOWN == dev->protocol) {
      if (IsHcalory(protocolTried)) {
        dev->protocol = protocolTried; // MVP1 vs MVP2 is known directly from which UUID worked
      } else if (op->notifylen > 0) {
        uint8_t detected = AA55DetectClass(op->dataNotify, op->notifylen);
        dev->protocol = (HP_UNKNOWN != detected) ? detected : HP_AA55;
      } else {
        dev->protocol = HP_AA55;
      }
      AddLog(LOG_LEVEL_INFO, PSTR("DHT: %s: detected as %s"), addrStr(addrev), ProtocolName(dev->protocol));
    }
    if (op->notifylen > 0) { // the password-handshake op has no meaningful status reply to parse
      if (IsHcalory(dev->protocol)) {
        hc_state_t st;
        if (HCParseResponse(op->dataNotify, op->notifylen, st)) {
          dev->state.hcalory = st;
          dev->stateValid = true;
        }
      } else {
        aa55_state_t st;
        if (AA55ParseResponse(op->dataNotify, op->notifylen, st)) {
          dev->state.aa55 = st;
          dev->stateValid = true;
          dev->protocol = AA55DetectClass(op->dataNotify, op->notifylen); // refine AA55/AA66/encrypted
        }
      }
    }
  }

  // MVP2 password learning/verification now happens inline within one connection - see
  // DHMvp2ChainCallback - rather than as a separate op reported through here.

  if (DHCMD_PASSWORD != cmd) {
    DHPublish(addrev, cmdName, true);
  }
  DHDoOp();
  return 0;
}

/*********************************************************************************************\
 * init / periodic
\*********************************************************************************************/

void DieselHeaterInit(void) {
  memset(&DieselHeaterDevices, 0, sizeof(DieselHeaterDevices));
  DieselHeaterPeriod = Settings->tele_period;
}

void DieselHeaterEverySecond(void) {
  seconds--;
  if (seconds <= 0) {
    if (DieselHeaterPeriod) {
      if (nextPoll >= DIESELHEATER_NUM_DEVICESLOTS) {
        nextPoll = 0;
      }
    }
    seconds = DieselHeaterPeriod;
  }

  if (DieselHeaterPeriod && (nextPoll < DIESELHEATER_NUM_DEVICESLOTS)) {
    int qlen = opQueue.size();
    if ((qlen == 0) && !opInProgress) {
      for (int i = nextPoll; i < DIESELHEATER_NUM_DEVICESLOTS; i++) {
        if (!DieselHeaterDevices[i].known) {
          nextPoll = i + 1;
          continue;
        }
        DHQueueOp(DieselHeaterDevices[i].addr, DHCMD_POLL, 0);
        nextPoll = i + 1;
        break;
      }
    }
  }

  DHDoOp();
}

/*********************************************************************************************\
 * Commands - Tasmota-style: value omitted means "query current cached setting", never triggers
 * a wire read (see file header "Caching" note). Only DieselHeaterState forces a fresh read.
\*********************************************************************************************/

const char *responses[] = { PSTR("Done"), PSTR("queued"), PSTR("invaddr"), PSTR("nostate") };

bool DHParseMac(uint8_t *addrbin, char **rest) {
  char *p = strtok(XdrvMailbox.data, " ");
  *rest = strtok(nullptr, "");
  if (!p) return false;
  return BLE_ESP32::getAddr(addrbin, p) != 0;
}

void CmndDieselHeaterState(void) {
  uint8_t addrbin[7];
  char *rest;
  if (!DHParseMac(addrbin, &rest)) { ResponseCmndChar(responses[2]); return; }
  int res = DHQueueOp(addrbin, DHCMD_STATE, 0);
  ResponseCmndChar(responses[(res > 0) ? 1 : 0]);
}

// Hcalory MVP2 only - PIN used on the device's next password handshake. Omit the value to query
// the currently cached PIN; give one (0-9999) to change it for this device. Doesn't itself touch
// the wire - takes effect on the next queued operation for this MAC.
void CmndDieselHeaterPassword(void) {
  uint8_t addrbin[7];
  char *rest;
  if (!DHParseMac(addrbin, &rest)) { ResponseCmndChar(responses[2]); return; }

  dh_device_tag *dev = findOrRegisterDevice(addrbin);
  if (!dev) { ResponseCmndChar(responses[2]); return; }

  if (rest && rest[0]) {
    int pin = atoi(rest);
    if (pin < 0) pin = 0;
    if (pin > 9999) pin = 9999;
    dev->hcaloryPasskey = (uint16_t)pin;
  }
  ResponseCmndNumber(dev->hcaloryPasskey);
}

// DieselHeaterError <mac> [<code>] - <code> true/on/1 (case-insensitive) returns the raw
// numeric error code; omitted or anything else (false/off/0/unrecognized) returns the
// description (D_DIESELHEATER_* - see DHErrorDescription), empty string when there's no error.
void CmndDieselHeaterError(void) {
  uint8_t addrbin[7];
  char *rest;
  if (!DHParseMac(addrbin, &rest)) { ResponseCmndChar(responses[2]); return; }

  dh_device_tag *dev = findOrRegisterDevice(addrbin);
  if (!dev || !dev->stateValid) { ResponseCmndChar(responses[3]); return; }

  bool wantCode = rest && rest[0]
    && (!strcasecmp(rest, "true") || !strcasecmp(rest, "on") || !strcmp(rest, "1"));

  uint8_t code = IsHcalory(dev->protocol) ? dev->state.hcalory.errorCode : dev->state.aa55.errCode;

  if (wantCode) {
    ResponseCmndNumber(code);
  } else {
    ResponseCmndChar(DHErrorDescription(dev->protocol, code));
  }
}

// shared implementation for the query-or-set commands
void DHQueryOrSet(uint8_t cmd, int (*parseArg)(const char *)) {
  uint8_t addrbin[7];
  char *rest;
  if (!DHParseMac(addrbin, &rest)) { ResponseCmndChar(responses[2]); return; }

  dh_device_tag *dev = findOrRegisterDevice(addrbin);

  if (!rest || !rest[0]) {
    // query - answer from cache only, never touch the wire
    if (!dev || !dev->stateValid) { ResponseCmndChar(responses[3]); return; }
    DHPublish(addrbin, cmdnames[cmd], true);
    ResponseCmndChar(responses[0]);
    return;
  }

  int argument = parseArg(rest);
  if (argument < 0) { ResponseCmndChar(responses[2]); return; } // reusing invaddr for "bad value" too

  // Hcalory MVP2 won't switch directly out of Ventilation via the mode-set command - confirmed
  // live: it's acknowledged ("result":"ok") but has no actual effect. Power-cycling first
  // reliably works, and - also confirmed live - turning off from Ventilation (no combustion to
  // purge) takes effect immediately, unlike the multi-minute cooldown after actually heating, so
  // no extra wait is needed: the existing op queue already serializes strictly one-at-a-time,
  // and Power OFF's own response already reflects the post-off state. Only handling MVP2 here -
  // no test evidence either way for MVP1 or AA55.
  // Relies on DHQueueOp synchronously dispatching (via DHDoOp) whenever nothing else is in
  // flight, and BuildFrame never failing for DHCMD_POWER on a confirmed-MVP2 device - so the
  // "off" push below is always already dispatched (and thus skipped by write coalescing, which
  // only scans pending/non-dispatched entries) before the "on" push is queued. If that ever
  // stops holding, coalescing would wrongly merge these into just "on".
  if ((DHCMD_MODE == cmd) && (0 == argument || 1 == argument) && dev && dev->stateValid
      && (HP_HCALORY_MVP2 == dev->protocol) && (0x3 == dev->state.hcalory.setMode)) {
    DHQueueOp(addrbin, DHCMD_POWER, 0); // off
    DHQueueOp(addrbin, DHCMD_POWER, 1); // on
  }

  int res = DHQueueOp(addrbin, cmd, argument);
  ResponseCmndChar(responses[(res > 0) ? 1 : 0]);
}

int ParsePower(const char *s) {
  if (!strcasecmp(s, "ON") || !strcmp(s, "1")) return 1;
  if (!strcasecmp(s, "OFF") || !strcmp(s, "0")) return 0;
  if (!strcasecmp(s, "TOGGLE") || !strcmp(s, "2")) return 2;
  return -1;
}
int ParseMode(const char *s) {
  if (!strcasecmp(s, "Level") || !strcmp(s, "0")) return 0;
  if (!strcasecmp(s, "Temperature") || !strcmp(s, "1")) return 1;
  if (!strcasecmp(s, "Ventilation") || !strcmp(s, "2")) return 2;
  return -1;
}
int ParseTemp(const char *s) {
  int t = atoi(s);
  return (t >= 0 && t <= 40) ? t : -1;
}
int ParseLevel(const char *s) {
  int l = atoi(s);
  return (l >= 1 && l <= 10) ? l : -1;
}
int ParseAuto(const char *s) {
  if (!strcasecmp(s, "ON") || !strcmp(s, "1")) return 1;
  if (!strcasecmp(s, "OFF") || !strcmp(s, "0")) return 0;
  return -1;
}

void CmndDieselHeaterPower(void) { DHQueryOrSet(DHCMD_POWER, ParsePower); }
void CmndDieselHeaterMode(void)  { DHQueryOrSet(DHCMD_MODE, ParseMode); }
void CmndDieselHeaterTemp(void)  { DHQueryOrSet(DHCMD_TEMP, ParseTemp); }
void CmndDieselHeaterLevel(void) { DHQueryOrSet(DHCMD_LEVEL, ParseLevel); }
void CmndDieselHeaterAuto(void)  { DHQueryOrSet(DHCMD_AUTO, ParseAuto); }

void CmndDieselHeaterPeriod(void) {
  if (XdrvMailbox.data_len > 0) {
    DieselHeaterPeriod = XdrvMailbox.payload;
    if (seconds > DieselHeaterPeriod) {
      seconds = DieselHeaterPeriod;
    }
  }
  ResponseCmndNumber(DieselHeaterPeriod);
}

#ifdef USE_WEBSERVER
const char HTTP_DH_MAC[] = "{s}%s " D_MAC_ADDRESS "{m}%s{e}";
const char HTTP_DH_STATE[] = "{s}%s State{m}%d{e}";
const char HTTP_DH_VOLTAGE[] = "{s}%s Supply{m}%s V{e}";
const char HTTP_DH_CABTEMP[] = "{s}%s Cabin{m}%d " D_UNIT_DEGREE "%c{e}";

void DieselHeaterShow(void) {
  bool first = true;
  for (int i = 0; i < DIESELHEATER_NUM_DEVICESLOTS; i++) {
    dh_device_tag &dev = DieselHeaterDevices[i];
    if (!dev.known) continue;
    if (!first) WSContentSend_P(HTTP_SNS_HR_THIN);
    first = false;

    char label[16];
    snprintf_P(label, sizeof(label), PSTR("DH-%d"), i + 1);
    WSContentSend_P(HTTP_DH_MAC, label, addrStr(dev.addr));
    if (dev.stateValid) {
      if (IsHcalory(dev.protocol)) {
        WSContentSend_P(HTTP_DH_STATE, label, dev.state.hcalory.runningState);
        char fbuf[16];
        dtostrfd(dev.state.hcalory.supplyVoltage, 1, fbuf);
        WSContentSend_P(HTTP_DH_VOLTAGE, label, fbuf);
        WSContentSend_P(HTTP_DH_CABTEMP, label, dev.state.hcalory.cabTemp,
                         dev.state.hcalory.tempUnit ? D_UNIT_FAHRENHEIT[0] : D_UNIT_CELSIUS[0]);
      } else {
        WSContentSend_P(HTTP_DH_STATE, label, dev.state.aa55.runningState);
        char fbuf[16];
        dtostrfd(dev.state.aa55.supplyVoltage, 1, fbuf);
        WSContentSend_P(HTTP_DH_VOLTAGE, label, fbuf);
        WSContentSend_P(HTTP_DH_CABTEMP, label, dev.state.aa55.cabTemp, D_UNIT_CELSIUS[0]);
      }
    }
  }
}
#endif // USE_WEBSERVER

} // namespace DIESELHEATER_ESP32

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xdrv89(uint32_t function) {
  bool result = false;

  switch (function) {
    case FUNC_INIT:
      DIESELHEATER_ESP32::DieselHeaterInit();
      break;
    case FUNC_EVERY_SECOND:
      DIESELHEATER_ESP32::DieselHeaterEverySecond();
      break;
    case FUNC_COMMAND:
      result = DecodeCommand(DIESELHEATER_ESP32::kDieselHeaterCommands, DIESELHEATER_ESP32::DieselHeaterCommands);
      break;
#ifdef USE_WEBSERVER
    case FUNC_WEB_SENSOR:
      DIESELHEATER_ESP32::DieselHeaterShow();
      break;
#endif // USE_WEBSERVER
    case FUNC_ACTIVE:
      result = true;
      break;
  }
  return result;
}

#endif  // USE_BLE_ESP32
#endif  // ESP32
#endif  // USE_BLE_DIESEL_HEATER
#endif  // CONFIG_IDF_TARGET_ESP32...
