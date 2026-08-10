/*
  xdrv_95_esp32_ble_hcalory_heater.ino - Hcalory diesel heater (MVP1/MVP2, e.g. HBU1S) sense
                                          and control via BLE_ESP32 support for Tasmota

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
  0.0.0.0 20260810  created - initial version. Protocol ported from
                              github.com/Spettacolo83/homeassistant-diesel-heater
                              (diesel_heater_ble/src/diesel_heater_ble/protocol.py's
                              ProtocolHcalory class - not from that repo's own HCALORY.md,
                              which turned out to disagree with the actual code in several
                              places: gear levels (doc says 1-6, code uses 1-10 directly per
                              upstream issue #46), the checksum span (doc's generic formula
                              sums the whole frame, the real one only sums from byte 8), and
                              the entire status-response byte layout (doc's generic table
                              doesn't match what parse() actually reads at all - the code
                              represents several rounds of real-hardware fixes the doc was
                              never updated for).
*/

/*
Commands:
e.g.
Hcalory 001122334455 settemp 22

HcaloryPeriod n - set polling period in seconds (default teleperiod at boot, 0 = off)

Hcalory <mac> state           - request current status (also happens automatically every period)
Hcalory <mac> on|off          - power on/off
Hcalory <mac> mode temp|level - switch between temperature-controlled and manual gear-level mode
Hcalory <mac> settemp <n>     - set target temperature (0-40, unit matches heater's own setting)
Hcalory <mac> setlevel <n>    - set target gear level (1-10)
Hcalory <mac> auto            - toggle automatic start/stop (this is a toggle on the real
                                 protocol, not a separate on/off - see upstream issue #43)

The first command referencing a MAC registers it for periodic status polling. Whether a device
is MVP1 (older, service 0xFFF0) or MVP2 (newer, e.g. HBU1S, service 0xBD39) is auto-detected:
MVP2 is tried first (matching the reference implementation's own default), falling back to MVP1
if that GATT service isn't found. MVP2 additionally requires a password handshake (default PIN
1234, not currently exposed as a command - see note below) before it accepts other commands;
since this driver's underlying BLE_ESP32 operation queue disconnects after every single
operation (no persistent connection to piggyback on, unlike the reference Home Assistant
integration), the handshake is queued immediately before every MVP2 command rather than once
per session.

Responses (stat/<topic>/Hcalory/<mac>):
{
  "cmd":"state",
  "result":"ok",
  "MAC":"001122334455",
  "RSSI":-70,
  "MVP":2,
  "RunningState":1,
  "RunningStep":3,
  "SetMode":1,
  "SetTemp":22,
  "AutoStartStop":false,
  "SupplyVoltage":12.3,
  "CaseTemp":45,
  "CabinTemp":22,
  "TempUnit":0,
  "Altitude":0,
  "ErrorCode":0
}
"SetLevel" appears instead of "SetTemp" when SetMode is level (gear) mode. Neither appears when
the heater is off/turning off/in error, matching the protocol's own "set value is meaningless in
that state" behavior - a stale zero would be misleading, so it's omitted instead.
*/

//#define VSCODE_DEV

#ifdef VSCODE_DEV
#define ESP32
#define USE_BLE_ESP32
#define USE_BLE_HCALORY
#endif

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32C2 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32P4
#ifdef USE_BLE_HCALORY
#ifdef ESP32                       // ESP32 only. Use define USE_HM10 for ESP8266 support
#ifdef USE_BLE_ESP32

#define XDRV_95                    95
#define D_CMND_HCALORY "Hcalory"

// uncomment for more debug messages
//#define HCALORY_DEBUG

namespace HCALORY_ESP32 {

void CmndHcalory(void);
void CmndHcaloryPeriod(void);

const char kHcaloryCommands[] PROGMEM = D_CMND_HCALORY"|"
  "|"
  "period";

void (*const HcaloryCommands[])(void) PROGMEM = {
  &CmndHcalory,
  &CmndHcaloryPeriod
};

// index into this table is the cmdtype stored in a queued operation's context
const char *cmdnames[] = {
  "poll",         // 0 - internal periodic poll, same wire command as "state"
  "state",        // 1
  "on",           // 2
  "off",          // 3
  "mode",         // 4
  "settemp",      // 5
  "setlevel",     // 6
  "auto",         // 7
  "password",     // 8 - internal, queued ahead of the above for MVP2
};

const char HC_MVP2_Svc[]         PROGMEM = "0000bd39-0000-1000-8000-00805f9b34fb";
const char HC_MVP2_Write_Char[]  PROGMEM = "0000bdf7-0000-1000-8000-00805f9b34fb";
const char HC_MVP2_Notify_Char[] PROGMEM = "0000bdf8-0000-1000-8000-00805f9b34fb";

const char HC_MVP1_Svc[]         PROGMEM = "0000fff0-0000-1000-8000-00805f9b34fb";
const char HC_MVP1_Write_Char[]  PROGMEM = "0000fff2-0000-1000-8000-00805f9b34fb";
const char HC_MVP1_Notify_Char[] PROGMEM = "0000fff1-0000-1000-8000-00805f9b34fb";

enum HcaloryVersion : uint8_t { HCV_UNKNOWN = 0, HCV_MVP2 = 1, HCV_MVP1 = 2 };

// dpID command types (16-bit), matching HCALORY_CMD_* in the reference implementation
#define HC_CMD_SET_GEAR     0x0607
#define HC_CMD_SET_TEMP     0x0706
#define HC_CMD_POWER        0x0E04   // query/power/mode/auto-toggle - argument-selected, see below
#define HC_CMD_QUERY_STATE  0x0A0A   // MVP2 only

// HC_CMD_POWER argument bytes
#define HC_POWER_QUERY       0x00
#define HC_POWER_ON          0x02
#define HC_POWER_OFF         0x01
#define HC_POWER_AUTO_TOGGLE 0x05
#define HC_POWER_MODE_LEVEL  0x07
#define HC_POWER_MODE_TEMP   0x06

/*********************************************************************************************\
 * Response parsing - ported from ProtocolHcalory.parse() (protocol.py), which the class's own
 * docstring confirms is the MVP2 byte mapping; the reference project uses this same parser for
 * MVP1 responses too (there's only the one parse() method for both), so this driver does the same.
\*********************************************************************************************/

struct hc_state_t {
  uint8_t runningState;    // 0=off, 1=on
  uint8_t runningStep;     // standard mapping: 0=standby,2=ignition,3=running,4=cooldown,6=ventilation
  uint8_t setMode;         // raw: 0=off,1=temperature,2=level,3=ventilation
  bool setValueNone;       // true when heater is off/turning-off/error - set_temp/set_level are meaningless then
  uint8_t setTemp;
  uint8_t setLevel;
  bool autoStartStop;
  float supplyVoltage;
  uint16_t caseTemp;       // unsigned, unlike the AA55-family protocols - Hcalory's parser has no sign byte for this
  uint16_t cabTemp;
  uint8_t highAltitude;    // raw altitude mode byte, 0-2
  uint8_t tempUnit;        // 0=C, 1=F
  uint8_t errorCode;       // only meaningful when runningStatus (high nibble of byte 20) == 0xF
};

// explicit prototype: same Arduino auto-prototype pitfall as xdrv_89's DHParseResponse - this
// function uses the locally-defined hc_state_t directly in its signature, so the generator would
// otherwise insert a broken forward declaration before hc_state_t is defined.
bool HCParseResponse(const uint8_t *data, uint8_t len, hc_state_t &st);

bool HCParseResponse(const uint8_t *data, uint8_t len, hc_state_t &st) {
  if (len < 38) return false;

  memset(&st, 0, sizeof(st));

  uint8_t complete_state_byte = data[20];
  uint8_t status = (complete_state_byte & 0xF0) >> 4;      // high nibble
  uint8_t running_step_raw = complete_state_byte & 0x0F;   // low nibble

  // synthetic COOLDOWN state when status is "turning off" (matches reference exactly)
  uint8_t running_step_hc = (0x4 == status) ? 0x4 /* treated as cooldown below regardless of raw */ : running_step_raw;

  st.runningState = (0x0 == status || 0xF == status) ? 0 : 1;  // OFF or ERROR -> 0, else 1

  // MVP2 steps: 0x0=Inactive,0x1=Fan,0x3=Ignition,0x5=Running,0x7=Standby ; 0x4(TURNING_OFF status)->synthetic cooldown
  // mapped to standard: 0=Standby,2=Ignition,3=Running,4=Cooldown,6=Ventilation/Fan
  if (0x4 == status) {
    st.runningStep = 4; // cooldown
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
    // OFF, TURNING_OFF, or ERROR - set value is meaningless (matches reference exactly)
    st.setValueNone = true;
  } else if (0x1 == st.setMode) {
    st.setTemp = set_value_raw;
  } else {
    // Beta.36: 1-10 gear levels directly, no 1-6 mapping (upstream issue #46)
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

/*********************************************************************************************\
 * Request builders - ported from _build_hcalory_cmd / _build_mvp2_query_cmd /
 * build_password_handshake (protocol.py). These three have genuinely different frame shapes in
 * the reference implementation (not variations of one general form), so they stay separate here
 * too rather than being forced into a single builder.
\*********************************************************************************************/

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

  return 12 + payload_len + 1; // total frame length
}

// MVP2 query-with-timestamp: 00 02 00 01 00 01 00 0A 0A 00 00 05 [HH MM SS DOW] 00 [checksum]
// timestamp is raw (not BCD); DOW is ISO weekday 1=Monday..7=Sunday
uint8_t HCBuildMvp2QueryCmd(uint8_t *buf) {
  buf[0] = 0x00; buf[1] = 0x02;
  buf[2] = 0x00; buf[3] = 0x01;
  buf[4] = 0x00; buf[5] = 0x01;
  buf[6] = 0x00; buf[7] = 0x0A; buf[8] = 0x0A; buf[9] = 0x00;
  buf[10] = 0x00; buf[11] = 0x05;

  // RtcTime is Tasmota's already-maintained current local time (support_rtc.ino) - no need to
  // break down an epoch ourselves. Its day_of_week uses Tasmota's own convention (1=Sunday),
  // not the protocol's ISO weekday (1=Monday..7=Sunday) - convert.
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
uint8_t HCBuildPasswordHandshake(uint8_t *buf, uint16_t passkey = 1234) {
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
 * Device table
\*********************************************************************************************/

#define HCALORY_NUM_DEVICESLOTS 4

struct hc_device_tag {
  uint8_t addr[7];
  bool known;
  uint8_t version;    // HcaloryVersion - HCV_UNKNOWN until first successful connect
  int8_t RSSI;
  hc_state_t state;
  bool stateValid;
} HcaloryDevices[HCALORY_NUM_DEVICESLOTS];

int HcaloryPeriod = 300;
int seconds = 20;
int nextPoll = HCALORY_NUM_DEVICESLOTS;
int opInProgress = 0;
int retries = 0;
#define HCALORY_RETRIES 4

#pragma pack( push, 1 )
struct op_t {
  uint8_t addr[7];
  uint8_t towrite[18];
  uint8_t writelen;
  uint8_t cmdtype;
  uint8_t versionHint;  // HcaloryVersion to try first for this op - HCV_UNKNOWN means "probe"
};
#pragma pack(pop)

std::deque<HCALORY_ESP32::op_t*> opQueue;

const char *addrStr(const uint8_t *addr) {
  static char addrstr[32];
  BLE_ESP32::dump(addrstr, 13, addr, 6);
  return addrstr;
}

hc_device_tag *findOrRegisterDevice(const uint8_t *addr) {
  int free = -1;
  for (int i = 0; i < HCALORY_NUM_DEVICESLOTS; i++) {
    if (HcaloryDevices[i].known && !memcmp(HcaloryDevices[i].addr, addr, 6)) {
      return &HcaloryDevices[i];
    }
    if (!HcaloryDevices[i].known && (free == -1)) {
      free = i;
    }
  }
  if (free == -1) {
    AddLog(LOG_LEVEL_ERROR, PSTR("HCT: %s: lost > %d devices"), addrStr(addr), HCALORY_NUM_DEVICESLOTS);
    return nullptr;
  }
  memset(&HcaloryDevices[free], 0, sizeof(hc_device_tag));
  memcpy(HcaloryDevices[free].addr, addr, 6);
  HcaloryDevices[free].known = true;
  HcaloryDevices[free].version = HCV_UNKNOWN;
  return &HcaloryDevices[free];
}

int HCGenericOpCompleteFn(BLE_ESP32::generic_sensor_t *pStruct);

// version: HCV_MVP2 or HCV_MVP1 (never HCV_UNKNOWN - caller resolves that first)
bool HCOperation(const uint8_t *MAC, const uint8_t *data, int datalen, int cmdtype, uint8_t version) {
  BLE_ESP32::generic_sensor_t *op = nullptr;

  int res = BLE_ESP32::newOperation(&op);
  if (!res) {
    AddLog(LOG_LEVEL_ERROR, PSTR("HCT: %s: Can't get a newOperation \"%s\" from BLE"), addrStr(MAC), cmdnames[cmdtype]);
    retries = 0;
    return false;
  }

  NimBLEAddress addr((uint8_t *)MAC, 0); // type 0 is public
  op->addr = addr;
  if (HCV_MVP1 == version) {
    op->serviceUUID = NimBLEUUID(HC_MVP1_Svc);
    op->characteristicUUID = NimBLEUUID(HC_MVP1_Write_Char);
    op->notificationCharacteristicUUID = NimBLEUUID(HC_MVP1_Notify_Char);
  } else {
    op->serviceUUID = NimBLEUUID(HC_MVP2_Svc);
    op->characteristicUUID = NimBLEUUID(HC_MVP2_Write_Char);
    op->notificationCharacteristicUUID = NimBLEUUID(HC_MVP2_Notify_Char);
  }

  op->writelen = datalen;
  memcpy(op->dataToWrite, data, datalen);

  op->completecallback = (void *)HCGenericOpCompleteFn;
  // pack cmdtype (low byte) and version actually used (next byte) into context
  op->context = (void *)(uint32_t)(cmdtype | ((uint32_t)version << 8));

  res = BLE_ESP32::extQueueOperation(&op);
  if (!res) {
    BLE_ESP32::freeOperation(&op);
    AddLog(LOG_LEVEL_ERROR, PSTR("HCT: %s: Failed to queue new operation \"%s\" - deleted"), addrStr(MAC), cmdnames[cmdtype]);
    retries = 0;
    return false;
  }
  return true;
}

int HCDoOp() {
  if (!opInProgress && opQueue.size()) {
    op_t* op = opQueue[0];
    hc_device_tag *dev = findOrRegisterDevice(op->addr);
    // probe MVP2 first if version is unknown for this device, matching the reference
    // implementation's own default (self._is_mvp2 = True)
    uint8_t version = (dev && dev->version != HCV_UNKNOWN) ? dev->version : HCV_MVP2;
    if (HCOperation(op->addr, op->towrite, op->writelen, op->cmdtype, version)) {
      opQueue.pop_front();
      opInProgress = 1;
      retries = HCALORY_RETRIES;
      delete op;
      return 1;
    } else {
      AddLog(LOG_LEVEL_ERROR, PSTR("HCT: %s: BLE could not start op queue - len %d"), addrStr(op->addr), opQueue.size());
    }
  }
  return 0;
}

// queues the password handshake immediately ahead of a real command when talking (or possibly
// talking, if version isn't known yet) to an MVP2 device - see the file header note on why this
// can't just be sent once per session.
int HCQueueOp(const uint8_t *MAC, const uint8_t *data, int datalen, int cmdtype) {
  hc_device_tag *dev = findOrRegisterDevice(MAC);
  bool mightBeMvp2 = !dev || (dev->version != HCV_MVP1);
  if (mightBeMvp2 && cmdtype != 8 /* not the handshake itself */) {
    uint8_t pw[18];
    uint8_t pwlen = HCBuildPasswordHandshake(pw);
    op_t* pwop = new op_t;
    memcpy(pwop->addr, MAC, 6);
    memcpy(pwop->towrite, pw, pwlen);
    pwop->writelen = pwlen;
    pwop->cmdtype = 8; // "password"
    opQueue.push_back(pwop);
  }

  op_t* newop = new op_t;
  memcpy(newop->addr, MAC, 6);
  memcpy(newop->towrite, data, datalen);
  newop->writelen = datalen;
  newop->cmdtype = cmdtype;
  opQueue.push_back(newop);
  int qlen = opQueue.size();
  AddLog(LOG_LEVEL_DEBUG, PSTR("HCT: %s: Operation \"%s\" queued - len now %d"), addrStr(MAC), cmdnames[cmdtype], qlen);
  HCDoOp();
  return qlen;
}

void HCPublish(const uint8_t *addr, const char *cmdName, bool success) {
  ResponseClear();
  ResponseAppend_P(PSTR("{\"cmd\":\"%s\""), cmdName);
  ResponseAppend_P(PSTR(",\"result\":\"%s\""), success ? "ok" : "fail");
  ResponseAppend_P(PSTR(",\"MAC\":\"%s\""), addrStr(addr));

  hc_device_tag *dev = findOrRegisterDevice(addr);
  if (dev) {
    ResponseAppend_P(PSTR(",\"RSSI\":%d"), dev->RSSI);
    if (HCV_UNKNOWN != dev->version) {
      ResponseAppend_P(PSTR(",\"MVP\":%d"), (HCV_MVP1 == dev->version) ? 1 : 2);
    }
    if (dev->stateValid) {
      hc_state_t &st = dev->state;
      ResponseAppend_P(PSTR(",\"RunningState\":%d"), st.runningState);
      ResponseAppend_P(PSTR(",\"RunningStep\":%d"), st.runningStep);
      ResponseAppend_P(PSTR(",\"SetMode\":%d"), st.setMode);
      if (!st.setValueNone) {
        if (0x1 == st.setMode) {
          ResponseAppend_P(PSTR(",\"SetTemp\":%d"), st.setTemp);
        } else {
          ResponseAppend_P(PSTR(",\"SetLevel\":%d"), st.setLevel);
        }
      }
      ResponseAppend_P(PSTR(",\"AutoStartStop\":%s"), st.autoStartStop ? "true" : "false");
      char fbuf[16];
      dtostrfd(st.supplyVoltage, 1, fbuf);
      ResponseAppend_P(PSTR(",\"SupplyVoltage\":%s"), fbuf);
      ResponseAppend_P(PSTR(",\"CaseTemp\":%d"), st.caseTemp);
      ResponseAppend_P(PSTR(",\"CabinTemp\":%d"), st.cabTemp);
      ResponseAppend_P(PSTR(",\"TempUnit\":%d"), st.tempUnit);
      ResponseAppend_P(PSTR(",\"Altitude\":%d"), st.highAltitude);
      ResponseAppend_P(PSTR(",\"ErrorCode\":%d"), st.errorCode);
    }
  }
  ResponseAppend_P(PSTR("}"));

  char stopic[TOPSZ];
  GetTopic_P(stopic, STAT, TasmotaGlobal.mqtt_topic, PSTR(""));
  strlcat(stopic, PSTR("Hcalory/"), sizeof(stopic));
  strlcat(stopic, addrStr(addr), sizeof(stopic));
  MqttPublish(stopic, false);
}

int HCGenericOpCompleteFn(BLE_ESP32::generic_sensor_t *op) {
  opInProgress = 0;

  uint8_t addrev[7];
  const uint8_t *native = op->addr.getVal();
  memcpy(addrev, native, 6);
  BLE_ESP32::ReverseMAC(addrev);

  uint32_t ctx = (uint32_t)op->context;
  int cmdtype = ctx & 0xFF;
  uint8_t versionTried = (ctx >> 8) & 0xFF;
  const char *cmdName = (cmdtype >= 0 && cmdtype < (int)(sizeof(cmdnames) / sizeof(*cmdnames))) ? cmdnames[cmdtype] : "invalid";

  hc_device_tag *dev = findOrRegisterDevice(addrev);

  if (op->state <= GEN_STATE_FAILED) {
    // if we were probing (device version not yet confirmed) and MVP2 failed, try MVP1 once
    // before falling back to the normal retry-same-thing path
    if (dev && (HCV_UNKNOWN == dev->version) && (HCV_MVP2 == versionTried) && (retries > 1)) {
      retries--;
      if (HCOperation(addrev, op->dataToWrite, op->writelen, cmdtype, HCV_MVP1)) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("HCT: %s: MVP2 not found, trying MVP1"), addrStr(addrev));
        opInProgress = 1;
        return 0;
      }
    }
    if (retries > 1) {
      retries--;
      if (HCOperation(addrev, op->dataToWrite, op->writelen, cmdtype, versionTried)) {
        AddLog(LOG_LEVEL_ERROR, PSTR("HCT: %s: operation \"%s\" failed - retries left: %d"), addrStr(addrev), cmdName, retries);
        opInProgress = 1;
      } else {
        retries = 0;
        HCPublish(addrev, cmdName, false);
      }
    } else {
      retries = 0;
      HCPublish(addrev, cmdName, false);
      AddLog(LOG_LEVEL_ERROR, PSTR("HCT: %s: operation \"%s\" failed - no more retries"), addrStr(addrev), cmdName);
    }
    return 0;
  }

  retries = 0;

  if (dev) {
    if (HCV_UNKNOWN == dev->version) {
      dev->version = (HcaloryVersion)versionTried;
      AddLog(LOG_LEVEL_INFO, PSTR("HCT: %s: detected as MVP%d"), addrStr(addrev), (HCV_MVP1 == dev->version) ? 1 : 2);
    }
    if (op->notifylen > 0) { // the password-handshake op has no meaningful status reply to parse
      hc_state_t st;
      if (HCParseResponse(op->dataNotify, op->notifylen, st)) {
        dev->state = st;
        dev->stateValid = true;
      }
    }
  }

  // the password handshake itself isn't user-visible - only publish for the real command
  if (8 != cmdtype) {
    HCPublish(addrev, cmdName, true);
  }
  return 0;
}

/*********************************************************************************************\
 * init / periodic
\*********************************************************************************************/

void HcaloryInit(void) {
  memset(&HcaloryDevices, 0, sizeof(HcaloryDevices));
  HcaloryPeriod = Settings->tele_period;
}

void HcaloryEverySecond(void) {
  seconds--;
  if (seconds <= 0) {
    if (HcaloryPeriod) {
      if (nextPoll >= HCALORY_NUM_DEVICESLOTS) {
        nextPoll = 0;
      }
    }
    seconds = HcaloryPeriod;
  }

  if (HcaloryPeriod && (nextPoll < HCALORY_NUM_DEVICESLOTS)) {
    int qlen = opQueue.size();
    if ((qlen == 0) && !opInProgress) {
      for (int i = nextPoll; i < HCALORY_NUM_DEVICESLOTS; i++) {
        if (!HcaloryDevices[i].known) {
          nextPoll = i + 1;
          continue;
        }
        uint8_t d[18];
        uint8_t dlen;
        if (HCV_MVP1 == HcaloryDevices[i].version) {
          uint8_t payload[9] = {0,0,0,0,0,0,0,0, HC_POWER_QUERY};
          dlen = HCBuildCmd(d, HC_CMD_POWER, payload, sizeof(payload));
        } else {
          dlen = HCBuildMvp2QueryCmd(d);
        }
        HCQueueOp(HcaloryDevices[i].addr, d, dlen, 0 /* "poll" */);
        nextPoll = i + 1;
        break;
      }
    }
  }

  HCDoOp();
}

/*********************************************************************************************\
 * Commands
\*********************************************************************************************/

int HCSend(const uint8_t *addr, const char *cmd, char *param) {
  uint8_t d[18];
  uint8_t dlen = 0;
  int cmdtype = -1;

  hc_device_tag *dev = findOrRegisterDevice(addr);
  bool isMvp1 = dev && (HCV_MVP1 == dev->version);

  do {
    if (!strcmp(cmd, "state")) {
      cmdtype = 1;
      if (isMvp1) {
        uint8_t payload[9] = {0,0,0,0,0,0,0,0, HC_POWER_QUERY};
        dlen = HCBuildCmd(d, HC_CMD_POWER, payload, sizeof(payload));
      } else {
        dlen = HCBuildMvp2QueryCmd(d);
      }
      break;
    }
    if (!strcmp(cmd, "on")) {
      cmdtype = 2;
      uint8_t payload[9] = {0,0,0,0,0,0,0,0, HC_POWER_ON};
      dlen = HCBuildCmd(d, HC_CMD_POWER, payload, sizeof(payload));
      break;
    }
    if (!strcmp(cmd, "off")) {
      cmdtype = 3;
      uint8_t payload[9] = {0,0,0,0,0,0,0,0, HC_POWER_OFF};
      dlen = HCBuildCmd(d, HC_CMD_POWER, payload, sizeof(payload));
      break;
    }
    if (!strcmp(cmd, "mode")) {
      cmdtype = 4;
      if (!param || !param[0]) return -1;
      uint8_t modeArg = !strcmp(param, "temp") ? HC_POWER_MODE_TEMP
                       : !strcmp(param, "level") ? HC_POWER_MODE_LEVEL : 0xFF;
      if (0xFF == modeArg) return -1;
      uint8_t payload[9] = {0,0,0,0,0,0,0,0, modeArg};
      dlen = HCBuildCmd(d, HC_CMD_POWER, payload, sizeof(payload));
      break;
    }
    if (!strcmp(cmd, "settemp")) {
      cmdtype = 5;
      if (!param || !param[0]) return -1;
      int temp = atoi(param);
      if (temp < 0) temp = 0;
      if (temp > 40) temp = 40;
      // unit byte matches whatever the heater last reported (byte 37) - default to Celsius (0)
      // until we've actually heard from it
      uint8_t unitByte = (dev && dev->stateValid) ? dev->state.tempUnit : 0;
      uint8_t payload[2] = {(uint8_t)temp, unitByte};
      dlen = HCBuildCmd(d, HC_CMD_SET_TEMP, payload, sizeof(payload));
      break;
    }
    if (!strcmp(cmd, "setlevel")) {
      cmdtype = 6;
      if (!param || !param[0]) return -1;
      int level = atoi(param);
      if (level < 1) level = 1;
      if (level > 10) level = 10;
      uint8_t payload[1] = {(uint8_t)level};
      dlen = HCBuildCmd(d, HC_CMD_SET_GEAR, payload, sizeof(payload));
      break;
    }
    if (!strcmp(cmd, "auto")) {
      // this is a toggle on the real protocol, not separate on/off (upstream issue #43)
      cmdtype = 7;
      uint8_t payload[9] = {0,0,0,0,0,0,0,0, HC_POWER_AUTO_TOGGLE};
      dlen = HCBuildCmd(d, HC_CMD_POWER, payload, sizeof(payload));
      break;
    }
  } while (0);

  if (cmdtype < 0) return -1;

  return HCQueueOp(addr, d, dlen, cmdtype);
}

const char *responses[] = {
  PSTR("Done"),
  PSTR("queued"),
  PSTR("ignoredbusy"),
  PSTR("invcmd"),
  PSTR("invaddr"),
};

void CmndHcalory(void) {
  char *data = XdrvMailbox.data;
  char *p = strtok(data, " ");
  if (!p) {
    ResponseCmndChar(responses[3]);
    return;
  }

  uint8_t addrbin[7];
  int addrres = BLE_ESP32::getAddr(addrbin, p);
  if (!addrres) {
    AddLog(LOG_LEVEL_ERROR, PSTR("HCT: addr invalid: %s"), p);
    ResponseCmndChar(responses[4]);
    return;
  }

  char *cmd = strtok(nullptr, " ");
  if (!cmd) {
    ResponseCmndChar(responses[3]);
    return;
  }
  char *param = strtok(nullptr, " ");

  int res = HCSend(addrbin, cmd, param);

  if (res > 0) {
    AddLog(LOG_LEVEL_INFO, PSTR("HCT: Command \"%s\" queued"), cmd);
    ResponseCmndChar(responses[1]);
    return;
  }
  AddLog(LOG_LEVEL_ERROR, PSTR("HCT: Command \"%s\" failed"), cmd);
  ResponseCmndChar(responses[3]);
}

void CmndHcaloryPeriod(void) {
  if (XdrvMailbox.data_len > 0) {
    HcaloryPeriod = XdrvMailbox.payload;
    if (seconds > HcaloryPeriod) {
      seconds = HcaloryPeriod;
    }
  }
  ResponseCmndNumber(HcaloryPeriod);
}

#ifdef USE_WEBSERVER
const char HTTP_HC_MAC[]      PROGMEM = "{s}%s " D_MAC_ADDRESS "{m}%s{e}";
const char HTTP_HC_STATE[]    PROGMEM = "{s}%s State{m}%d{e}";
const char HTTP_HC_VOLTAGE[]  PROGMEM = "{s}%s Supply{m}%s V{e}";
const char HTTP_HC_CABTEMP[]  PROGMEM = "{s}%s Cabin{m}%d " D_UNIT_DEGREE "%c{e}";

void HcaloryShow(void) {
  bool first = true;
  for (int i = 0; i < HCALORY_NUM_DEVICESLOTS; i++) {
    hc_device_tag &dev = HcaloryDevices[i];
    if (!dev.known) continue;
    if (!first) WSContentSend_P(HTTP_SNS_HR_THIN);
    first = false;

    char label[16];
    snprintf_P(label, sizeof(label), PSTR("HC-%d"), i + 1);
    WSContentSend_P(HTTP_HC_MAC, label, addrStr(dev.addr));
    if (dev.stateValid) {
      WSContentSend_P(HTTP_HC_STATE, label, dev.state.runningState);
      char fbuf[16];
      dtostrfd(dev.state.supplyVoltage, 1, fbuf);
      WSContentSend_P(HTTP_HC_VOLTAGE, label, fbuf);
      WSContentSend_P(HTTP_HC_CABTEMP, label, dev.state.cabTemp,
                       dev.state.tempUnit ? D_UNIT_FAHRENHEIT[0] : D_UNIT_CELSIUS[0]);
    }
  }
}
#endif // USE_WEBSERVER

} // namespace HCALORY_ESP32

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xdrv95(uint32_t function) {
  bool result = false;

  switch (function) {
    case FUNC_INIT:
      HCALORY_ESP32::HcaloryInit();
      break;
    case FUNC_EVERY_SECOND:
      HCALORY_ESP32::HcaloryEverySecond();
      break;
    case FUNC_COMMAND:
      result = DecodeCommand(HCALORY_ESP32::kHcaloryCommands, HCALORY_ESP32::HcaloryCommands);
      break;
#ifdef USE_WEBSERVER
    case FUNC_WEB_SENSOR:
      HCALORY_ESP32::HcaloryShow();
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
#endif  // USE_BLE_HCALORY
#endif  // CONFIG_IDF_TARGET_ESP32...
