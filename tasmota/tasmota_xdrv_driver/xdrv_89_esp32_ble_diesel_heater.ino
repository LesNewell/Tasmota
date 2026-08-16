/*
  xdrv_89_esp32_ble_diesel_heater.ino - Chinese diesel heater (5/8kW BLE dashboard) sense and
                                          control via BLE_ESP32 support for Tasmota

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
  0.0.0.0 20260810  created - initial version, protocol ported from
                              github.com/warehog/esphome-diesel-heater-ble
                              (components/diesel_heater_ble/{messages,state,heater}.{h,cpp})
*/

/*
Commands:
e.g.
DieselHeater 001122334455 settemp 22

DieselHeaterPeriod n - set polling period in seconds (default teleperiod at boot, 0 = off)

DieselHeater <mac> state           - request current status (also happens automatically every period)
DieselHeater <mac> raw <cmd> <d1> <d2> - send a raw command (each a 0-255 byte value)
DieselHeater <mac> on|off          - power on/off
DieselHeater <mac> mode <0-2>      - set running mode (0=level, 1=level, 2=temperature - see below)
DieselHeater <mac> settemp <n>     - set target temperature (raw units, as sent to the heater)
DieselHeater <mac> setlevel <n>    - set target level (1-10ish, heater-dependent)
DieselHeater <mac> auto 0|1        - automatic start/stop
DieselHeater <mac> language <n>    - UI language code
DieselHeater <mac> tempunit 0|1    - temperature unit (1 = Celsius)
DieselHeater <mac> altiunit 0|1    - altitude unit (1 = meters)
DieselHeater <mac> tankvolume <n>  - fuel tank volume
DieselHeater <mac> oilpumptype <n> - oil pump type
DieselHeater <mac> offset <n>      - temperature offset

The first command referencing a MAC registers it for periodic status polling.

Responses (stat/<topic>/DieselHeater/<mac>):
{
  "cmd":"state",
  "result":"ok",
  "MAC":"001122334455",
  "RSSI":-70,
  "RunningState":1,
  "ErrorCode":0,
  "RunningStep":3,
  "Altitude":120,
  "RunningMode":2,
  "SetLevel":5,
  "SetTemp":22,
  "SupplyVoltage":12.3,
  "CaseTemp":45,
  "CabinTemp":22.1
}
When the heater's response is one of the encrypted variants, the response also carries the extra
fields (StartTime, AutoTime, RunTime, IsAuto, Language, TempOffset, TankVolume, OilPumpType,
Rf433OnOff, TempUnit, AltitudeUnit, AutomaticHeating) - not all heater firmwares report these.

Protocol notes (see messages.h/state.h in the repo above for the original C++):
  Request frame (8 bytes): AA 55 0C 22 <cmd> <d1> <d2> <checksum>
    checksum = (0x0C + 0x22 + cmd + d1 + d2) % 256
  Response frame: header byte[0]/[1] identify one of 4 variants:
    AA 55            - unencrypted, "short" field layout
    AA 66            - unencrypted, "short" field layout, errcode at a different offset
    AA(^password) 34 - encrypted (XOR each of 6x 8-byte blocks against ASCII "password"),
                        "long" (48-byte) field layout with extra fields
    AA(^password) 07 - as above, other errcode offset variant
  This driver does not know in advance which variant a given heater uses - it is detected from
  the response itself, every time, exactly as the reference implementation does.
*/

//#define VSCODE_DEV

#ifdef VSCODE_DEV
#define ESP32
#define USE_BLE_ESP32
#define USE_BLE_DIESEL_HEATER
#endif

#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32C2 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32P4
#ifdef USE_BLE_DIESEL_HEATER
#ifdef ESP32                       // ESP32 only. Use define USE_HM10 for ESP8266 support
#ifdef USE_BLE_ESP32

#define XDRV_89                    89
#define D_CMND_DIESELHEATER "DieselHeater"

// uncomment for more debug messages
//#define DIESELHEATER_DEBUG

namespace DIESELHEATER_ESP32 {

void CmndDieselHeater(void);
void CmndDieselHeaterPeriod(void);

const char kDieselHeaterCommands[] PROGMEM = D_CMND_DIESELHEATER"|"
  "|"
  "period";

void (*const DieselHeaterCommands[])(void) PROGMEM = {
  &CmndDieselHeater,
  &CmndDieselHeaterPeriod
};

// index into this table is the cmdtype stored in a queued operation's context
const char *cmdnames[] = {
  "poll",       // 0 - internal periodic poll, same wire command as "state"
  "raw",        // 1
  "state",      // 2
  "on",         // 3
  "off",        // 4
  "mode",       // 5
  "settemp",    // 6
  "setlevel",   // 7
  "auto",       // 8
  "language",   // 9
  "tempunit",   // 10
  "altiunit",   // 11
  "tankvolume", // 12
  "oilpumptype",// 13
  "offset",     // 14
};

const char DH_Svc[]        PROGMEM = "0000ffe0-0000-1000-8000-00805f9b34fb";
// single characteristic used for both write and notify
const char DH_rw_Char[]    PROGMEM = "0000ffe1-0000-1000-8000-00805f9b34fb";
const char DH_notify_Char[] PROGMEM = "0000ffe1-0000-1000-8000-00805f9b34fb";

/*********************************************************************************************\
 * Response parsing - ported field-for-field from the reference implementation's
 * ResponseParser::{decrypt,detect_heater_class,parse} (messages.h) and HeaterState (state.h).
 * Deliberately not "cleaned up" against the source layout so it stays easy to diff against it.
\*********************************************************************************************/

enum HeaterClass : uint8_t {
  HC_AA_55 = 0,
  HC_AA_66 = 1,
  HC_AA_55_ENCRYPTED = 2,
  HC_AA_66_ENCRYPTED = 3,
  HC_UNKNOWN = 0xFF
};

struct dh_state_t {
  uint8_t heaterClass;
  uint8_t rcvCmd;
  uint8_t runningState;
  uint8_t errCode;
  uint8_t runningStep;
  uint16_t altitude;
  uint8_t runningMode;
  uint8_t setLevel;
  uint8_t setTemp;
  float supplyVoltage;
  uint16_t caseTemp;
  float cabTemp;

  bool hasExtras;
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

HeaterClass DHDetectClass(const uint8_t *raw, uint8_t len) {
  if (len < 2) return HC_UNKNOWN;
  switch (raw[1]) {
    case 0x55: return HC_AA_55;
    case 0x66: return HC_AA_66;
    case 0x34: return HC_AA_55_ENCRYPTED;   // 0x55 ^ 'p'(0x70... actually XOR of full key, see decrypt below)
    case 0x07: return HC_AA_66_ENCRYPTED;
    default:   return HC_UNKNOWN;
  }
}

// XORs (up to) the first 48 bytes, 8 at a time, against ASCII "password" - matches
// ResponseParser::decrypt() exactly, including its "already 0xAA -> no-op" fast path.
void DHDecrypt(const uint8_t *raw, uint8_t len, uint8_t *out) {
  memcpy(out, raw, len);
  if (raw[0] == 0xAA) return;
  static const uint8_t key[8] = {112, 97, 115, 115, 119, 111, 114, 100}; // "password"
  uint8_t n = (len < 48) ? len : 48;
  for (uint8_t i = 0; i < n; i++) {
    out[i] = (uint8_t)(raw[i] ^ key[i % 8]);
  }
}

bool DHParseResponse(const uint8_t *raw, uint8_t len, dh_state_t &st) {
  HeaterClass hc = DHDetectClass(raw, len);
  if (HC_UNKNOWN == hc) return false;

  uint8_t decrypted[48];
  uint8_t dlen = (len < 48) ? len : 48;
  DHDecrypt(raw, dlen, decrypted);

  memset(&st, 0, sizeof(st));
  st.heaterClass = hc;

  if (dlen > 2) st.rcvCmd = decrypted[2];
  if (dlen > 3) st.runningState = decrypted[3];

  if ((HC_AA_55 == hc || HC_AA_55_ENCRYPTED == hc) && dlen > 4) {
    st.errCode = decrypted[4];
  } else if (HC_AA_66 == hc && dlen > 17) {
    st.errCode = decrypted[17];
  } else if (HC_AA_66_ENCRYPTED == hc && dlen > 35) {
    st.errCode = decrypted[35];
  }

  if (dlen > 5) st.runningStep = decrypted[5];

  if ((HC_AA_55 == hc || HC_AA_66 == hc) && dlen > 7) {
    st.altitude = decrypted[6] + (decrypted[7] << 8);
  } else if ((HC_AA_55_ENCRYPTED == hc || HC_AA_66_ENCRYPTED == hc) && dlen > 7) {
    st.altitude = (decrypted[7] + (decrypted[6] << 8)) / 10;
  }

  if (dlen > 8) st.runningMode = decrypted[8];

  if ((HC_AA_55 == hc || HC_AA_66 == hc) && dlen > 10) {
    if (0x00 == st.runningMode) {
      st.setLevel = decrypted[10] + 1;
    } else if (0x01 == st.runningMode) {
      st.setLevel = decrypted[9];
    } else if (0x02 == st.runningMode) {
      st.setTemp = decrypted[9];
      st.setLevel = decrypted[10] + 1;
    }
  } else if ((HC_AA_55_ENCRYPTED == hc || HC_AA_66_ENCRYPTED == hc) && dlen > 10) {
    st.setLevel = decrypted[10];
    st.setTemp = decrypted[9];
  }

  if ((HC_AA_55 == hc || HC_AA_66 == hc) && dlen > 12) {
    st.supplyVoltage = (float)(decrypted[11] + (decrypted[12] << 8)) / 10.0f;
  } else if ((HC_AA_55_ENCRYPTED == hc || HC_AA_66_ENCRYPTED == hc) && dlen > 12) {
    st.supplyVoltage = (float)(decrypted[12] + (decrypted[11] << 8)) / 10.0f;
  }

  if ((HC_AA_55 == hc || HC_AA_66 == hc) && dlen > 16) {
    st.caseTemp = decrypted[13] + (decrypted[14] << 8);
    st.cabTemp = decrypted[15] + (decrypted[16] << 8);      // no /10 - matches reference exactly
  } else if ((HC_AA_55_ENCRYPTED == hc || HC_AA_66_ENCRYPTED == hc) && dlen > 33) {
    st.caseTemp = decrypted[14] + (decrypted[13] << 8);
    st.cabTemp = (float)(decrypted[33] + (decrypted[32] << 8)) / 10.0f;
  }

  st.hasExtras = false;
  if ((HC_AA_55_ENCRYPTED == hc || HC_AA_66_ENCRYPTED == hc) && dlen > 34) {
    st.stTime = decrypted[20] + (decrypted[19] << 8);
    st.autoTime = decrypted[22] + (decrypted[21] << 8);
    st.runTime = decrypted[24] + (decrypted[23] << 8);
    st.isAuto = decrypted[25];
    st.language = decrypted[26];
    st.tempOffset = decrypted[34];
    st.tankVolume = decrypted[28];
    st.oilPumpType = decrypted[29];
    // matches reference exactly: this specific check is against the RAW (still-encrypted) byte
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

/*********************************************************************************************\
 * Request builder
\*********************************************************************************************/

void DHBuildRequest(uint8_t *buf, uint8_t cmd, uint8_t d1, uint8_t d2) {
  buf[0] = 0xAA;
  buf[1] = 0x55;
  buf[2] = 0x0C;
  buf[3] = 0x22;
  buf[4] = cmd;
  buf[5] = d1;
  buf[6] = d2;
  buf[7] = (uint8_t)((0x0C + 0x22 + cmd + d1 + d2) % 256);
}

/*********************************************************************************************\
 * Device table - much simpler than EQ3's: no advertisement-based discovery, a device is
 * registered the first time a command references its MAC and is then polled periodically.
\*********************************************************************************************/

#define DIESELHEATER_NUM_DEVICESLOTS 4

struct dh_device_tag {
  uint8_t addr[7];
  bool known;
  int8_t RSSI;
  dh_state_t state;
  bool stateValid;
} DieselHeaterDevices[DIESELHEATER_NUM_DEVICESLOTS];

int DieselHeaterPeriod = 300;
int seconds = 20;
int nextPoll = DIESELHEATER_NUM_DEVICESLOTS;
int opInProgress = 0;
int retries = 0;
#define DIESELHEATER_RETRIES 4

#pragma pack( push, 1 )
struct op_t {
  uint8_t addr[7];
  uint8_t towrite[8];
  uint8_t writelen;
  uint8_t cmdtype;
};
#pragma pack(pop)

std::deque<DIESELHEATER_ESP32::op_t*> opQueue;

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
  return &DieselHeaterDevices[free];
}

int DHGenericOpCompleteFn(BLE_ESP32::generic_sensor_t *pStruct);

bool DHOperation(const uint8_t *MAC, const uint8_t *data, int datalen, int cmdtype) {
  BLE_ESP32::generic_sensor_t *op = nullptr;

  int res = BLE_ESP32::newOperation(&op);
  if (!res) {
    AddLog(LOG_LEVEL_ERROR, PSTR("DHT: %s: Can't get a newOperation \"%s\" from BLE"), addrStr(MAC), cmdnames[cmdtype]);
    retries = 0;
    return false;
  }

  NimBLEAddress addr((uint8_t *)MAC, 0); // type 0 is public
  op->addr = addr;
  op->serviceUUID = NimBLEUUID(DH_Svc);
  op->characteristicUUID = NimBLEUUID(DH_rw_Char);
  op->notificationCharacteristicUUID = NimBLEUUID(DH_notify_Char);

  op->writelen = datalen;
  memcpy(op->dataToWrite, data, datalen);

  op->completecallback = (void *)DHGenericOpCompleteFn;
  op->context = (void *)cmdtype;

  res = BLE_ESP32::extQueueOperation(&op);
  if (!res) {
    BLE_ESP32::freeOperation(&op);
    AddLog(LOG_LEVEL_ERROR, PSTR("DHT: %s: Failed to queue new operation \"%s\" - deleted"), addrStr(MAC), cmdnames[cmdtype]);
    retries = 0;
    return false;
  }
  return true;
}

int DHDoOp() {
  if (!opInProgress && opQueue.size()) {
    op_t* op = opQueue[0];
    if (DHOperation(op->addr, op->towrite, op->writelen, op->cmdtype)) {
      opQueue.pop_front();
      opInProgress = 1;
      retries = DIESELHEATER_RETRIES;
      delete op;
      return 1;
    } else {
      AddLog(LOG_LEVEL_ERROR, PSTR("DHT: %s: BLE could not start op queue - len %d"), addrStr(op->addr), opQueue.size());
    }
  }
  return 0;
}

int DHQueueOp(const uint8_t *MAC, const uint8_t *data, int datalen, int cmdtype) {
  op_t* newop = new op_t;
  memcpy(newop->addr, MAC, 6);
  memcpy(newop->towrite, data, datalen);
  newop->writelen = datalen;
  newop->cmdtype = cmdtype;
  opQueue.push_back(newop);
  int qlen = opQueue.size();
  AddLog(LOG_LEVEL_DEBUG, PSTR("DHT: %s: Operation \"%s\" queued - len now %d"), addrStr(MAC), cmdnames[cmdtype], qlen);
  DHDoOp();
  return qlen;
}

void DHPublish(const uint8_t *addr, const char *cmdName, bool success) {
  ResponseClear();
  ResponseAppend_P(PSTR("{\"cmd\":\"%s\""), cmdName);
  ResponseAppend_P(PSTR(",\"result\":\"%s\""), success ? "ok" : "fail");
  ResponseAppend_P(PSTR(",\"MAC\":\"%s\""), addrStr(addr));

  dh_device_tag *dev = findOrRegisterDevice(addr);
  if (dev) {
    ResponseAppend_P(PSTR(",\"RSSI\":%d"), dev->RSSI);
    if (dev->stateValid) {
      dh_state_t &st = dev->state;
      ResponseAppend_P(PSTR(",\"RunningState\":%d"), st.runningState);
      ResponseAppend_P(PSTR(",\"ErrorCode\":%d"), st.errCode);
      ResponseAppend_P(PSTR(",\"RunningStep\":%d"), st.runningStep);
      ResponseAppend_P(PSTR(",\"Altitude\":%d"), st.altitude);
      ResponseAppend_P(PSTR(",\"RunningMode\":%d"), st.runningMode);
      ResponseAppend_P(PSTR(",\"SetLevel\":%d"), st.setLevel);
      ResponseAppend_P(PSTR(",\"SetTemp\":%d"), st.setTemp);
      char fbuf[16];
      dtostrfd(st.supplyVoltage, 1, fbuf);
      ResponseAppend_P(PSTR(",\"SupplyVoltage\":%s"), fbuf);
      ResponseAppend_P(PSTR(",\"CaseTemp\":%d"), st.caseTemp);
      dtostrfd(st.cabTemp, 1, fbuf);
      ResponseAppend_P(PSTR(",\"CabinTemp\":%s"), fbuf);
      if (st.hasExtras) {
        ResponseAppend_P(PSTR(",\"StartTime\":%d"), st.stTime);
        ResponseAppend_P(PSTR(",\"AutoTime\":%d"), st.autoTime);
        ResponseAppend_P(PSTR(",\"RunTime\":%d"), st.runTime);
        ResponseAppend_P(PSTR(",\"IsAuto\":%d"), st.isAuto);
        ResponseAppend_P(PSTR(",\"Language\":%d"), st.language);
        ResponseAppend_P(PSTR(",\"TempOffset\":%d"), st.tempOffset);
        ResponseAppend_P(PSTR(",\"TankVolume\":%d"), st.tankVolume);
        ResponseAppend_P(PSTR(",\"OilPumpType\":%d"), st.oilPumpType);
        ResponseAppend_P(PSTR(",\"Rf433OnOff\":%s"), st.rf433OnOff ? "true" : "false");
        ResponseAppend_P(PSTR(",\"TempUnit\":%d"), st.tempUnit);
        ResponseAppend_P(PSTR(",\"AltitudeUnit\":%d"), st.altiUnit);
        ResponseAppend_P(PSTR(",\"AutomaticHeating\":%d"), st.automaticHeating);
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

  int cmdtype = (int)((uint32_t)op->context);
  const char *cmdName = (cmdtype >= 0 && cmdtype < (int)(sizeof(cmdnames) / sizeof(*cmdnames))) ? cmdnames[cmdtype] : "invalid";

  if (op->state <= GEN_STATE_FAILED) {
    if (retries > 1) {
      retries--;
      if (DHOperation(addrev, op->dataToWrite, op->writelen, cmdtype)) {
        AddLog(LOG_LEVEL_ERROR, PSTR("DHT: %s: operation \"%s\" failed - retries left: %d"), addrStr(addrev), cmdName, retries);
        opInProgress = 1;
      } else {
        retries = 0;
        DHPublish(addrev, cmdName, false);
        AddLog(LOG_LEVEL_ERROR, PSTR("DHT: %s: operation \"%s\" failed to resend"), addrStr(addrev), cmdName);
      }
    } else {
      retries = 0;
      DHPublish(addrev, cmdName, false);
      AddLog(LOG_LEVEL_ERROR, PSTR("DHT: %s: operation \"%s\" failed - no more retries"), addrStr(addrev), cmdName);
    }
    return 0;
  }

  retries = 0;

  dh_device_tag *dev = findOrRegisterDevice(addrev);
  if (dev) {
    dh_state_t st;
    if (DHParseResponse(op->dataNotify, op->notifylen, st)) {
      dev->state = st;
      dev->stateValid = true;
    } else {
      AddLog(LOG_LEVEL_ERROR, PSTR("DHT: %s: could not parse response (%d bytes, header %02X %02X)"),
             addrStr(addrev), op->notifylen,
             op->notifylen > 0 ? op->dataNotify[0] : 0, op->notifylen > 1 ? op->dataNotify[1] : 0);
    }
  }

  DHPublish(addrev, cmdName, true);
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
        uint8_t d[8];
        DHBuildRequest(d, 0x01, 0, 0);
        DHQueueOp(DieselHeaterDevices[i].addr, d, sizeof(d), 0 /* "poll" */);
        nextPoll = i + 1;
        break;
      }
    }
  }

  DHDoOp();
}

/*********************************************************************************************\
 * Commands
\*********************************************************************************************/

int DHSend(const uint8_t *addr, const char *cmd, char *param, char *param2) {
  uint8_t d[8];
  int cmdtype = -1;

  do {
    if (!strcmp(cmd, "raw")) {
      cmdtype = 1;
      if (!param || !param[0] || !param2 || !param2[0]) return -1;
      char *p3 = strtok(nullptr, " "); // third byte, if any
      DHBuildRequest(d, atoi(param), atoi(param2), p3 ? atoi(p3) : 0);
      break;
    }
    if (!strcmp(cmd, "state") || !strcmp(cmd, "poll")) {
      cmdtype = 2;
      DHBuildRequest(d, 0x01, 0, 0);
      break;
    }
    if (!strcmp(cmd, "on")) {
      cmdtype = 3;
      DHBuildRequest(d, 0x03, 1, 0);
      break;
    }
    if (!strcmp(cmd, "off")) {
      cmdtype = 4;
      DHBuildRequest(d, 0x03, 0, 0);
      break;
    }
    if (!strcmp(cmd, "mode")) {
      cmdtype = 5;
      if (!param || !param[0]) return -1;
      DHBuildRequest(d, 0x02, atoi(param), 0);
      break;
    }
    if (!strcmp(cmd, "settemp")) {
      cmdtype = 6;
      if (!param || !param[0]) return -1;
      DHBuildRequest(d, 0x04, atoi(param), 0);
      break;
    }
    if (!strcmp(cmd, "setlevel")) {
      cmdtype = 7;
      if (!param || !param[0]) return -1;
      int level = atoi(param);
      if (level < 1) level = 1;
      DHBuildRequest(d, 0x04, level - 1, 0);
      break;
    }
    if (!strcmp(cmd, "auto")) {
      cmdtype = 8;
      if (!param || !param[0]) return -1;
      DHBuildRequest(d, 0x13, atoi(param) ? 1 : 0, 0);
      break;
    }
    if (!strcmp(cmd, "language")) {
      cmdtype = 9;
      if (!param || !param[0]) return -1;
      DHBuildRequest(d, 0x14, atoi(param), 0);
      break;
    }
    if (!strcmp(cmd, "tempunit")) {
      cmdtype = 10;
      if (!param || !param[0]) return -1;
      DHBuildRequest(d, 0x15, atoi(param) ? 1 : 0, 0);
      break;
    }
    if (!strcmp(cmd, "altiunit")) {
      cmdtype = 11;
      if (!param || !param[0]) return -1;
      DHBuildRequest(d, 0x16, atoi(param) ? 1 : 0, 0);
      break;
    }
    if (!strcmp(cmd, "tankvolume")) {
      cmdtype = 12;
      if (!param || !param[0]) return -1;
      DHBuildRequest(d, 0x17, atoi(param), 0);
      break;
    }
    if (!strcmp(cmd, "oilpumptype")) {
      cmdtype = 13;
      if (!param || !param[0]) return -1;
      DHBuildRequest(d, 0x18, atoi(param), 0);
      break;
    }
    if (!strcmp(cmd, "offset")) {
      cmdtype = 14;
      if (!param || !param[0]) return -1;
      DHBuildRequest(d, 0x20, atoi(param), 0);
      break;
    }
  } while (0);

  if (cmdtype < 0) return -1;

  // registering the device now (rather than waiting for the reply) is what starts periodic polling
  findOrRegisterDevice(addr);

  return DHQueueOp(addr, d, sizeof(d), cmdtype);
}

const char *responses[] = {
  PSTR("Done"),
  PSTR("queued"),
  PSTR("ignoredbusy"),
  PSTR("invcmd"),
  PSTR("invaddr"),
};

void CmndDieselHeater(void) {
  char *data = XdrvMailbox.data;
  char *p = strtok(data, " ");
  if (!p) {
    ResponseCmndChar(responses[3]);
    return;
  }

  uint8_t addrbin[7];
  int addrres = BLE_ESP32::getAddr(addrbin, p);
  if (!addrres) {
    AddLog(LOG_LEVEL_ERROR, PSTR("DHT: addr invalid: %s"), p);
    ResponseCmndChar(responses[4]);
    return;
  }

  char *cmd = strtok(nullptr, " ");
  if (!cmd) {
    ResponseCmndChar(responses[3]);
    return;
  }
  char *param = strtok(nullptr, " ");
  char *param2 = nullptr;
  if (param) {
    param2 = strtok(nullptr, " ");
  }

  int res = DHSend(addrbin, cmd, param, param2);

  if (res > 0) {
    AddLog(LOG_LEVEL_INFO, PSTR("DHT: Command \"%s\" queued"), cmd);
    ResponseCmndChar(responses[1]);
    return;
  }
  AddLog(LOG_LEVEL_ERROR, PSTR("DHT: Command \"%s\" failed"), cmd);
  ResponseCmndChar(responses[3]);
}

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
const char HTTP_DH_MAC[]      PROGMEM = "{s}%s " D_MAC_ADDRESS "{m}%s{e}";
const char HTTP_DH_STATE[]    PROGMEM = "{s}%s State{m}%d{e}";
const char HTTP_DH_VOLTAGE[]  PROGMEM = "{s}%s Supply{m}%s V{e}";
const char HTTP_DH_CABTEMP[]  PROGMEM = "{s}%s Cabin{m}%s " D_UNIT_DEGREE "%c{e}";

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
      WSContentSend_P(HTTP_DH_STATE, label, dev.state.runningState);
      char fbuf[16];
      dtostrfd(dev.state.supplyVoltage, 1, fbuf);
      WSContentSend_P(HTTP_DH_VOLTAGE, label, fbuf);
      dtostrfd(dev.state.cabTemp, 1, fbuf);
      WSContentSend_P(HTTP_DH_CABTEMP, label, fbuf, D_UNIT_CELSIUS[0]);
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
