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

  Credits - protocol details for both heater families were worked out and documented by these
  reference projects; this driver is an independent Tasmota/C++ implementation ported from their
  findings, not a code copy of either:

    warehog - https://github.com/warehog/esphome-diesel-heater-ble
    AA55/AA66 (+ their encrypted variants), ESPHome component

    Spettacolo83 - https://github.com/Spettacolo83/homeassistant-diesel-heater
    Hcalory MVP1/MVP2, Home Assistant integration

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
                              detected automatically - see "Protocol handlers" below.
  0.0.0.1 20260812  refactor - each protocol is its own HeaterProtocolHandler subclass, and
                              each *device* is its own heap-allocated instance of whichever
                              subclass its protocol turned out to be - there's no separate
                              "device" struct/state-union alongside the handler any more, the
                              object simply IS the device: wire-format constants, GATT UUIDs,
                              frame building/response parsing, and now the parsed status fields
                              themselves all live as members (most private/protected) of that
                              one class. A small file-scope DeviceRegistry (dynamic array of
                              base-class pointers) is all that's shared: it maps a MAC to its
                              device object and is what the periodic poll/web status page walk.
                              Detection - trying each known protocol in turn to see which one a
                              new MAC actually speaks - only ever happens once per device (a
                              device can't change protocol mid-session), via a small
                              Prototype-pattern factory (createInstance()) on each class: the
                              cascade order/candidate list (kProtocolHandlers) holds one
                              stateless singleton per protocol used *only* to spin up a fresh
                              instance of the right concrete type once that protocol answers -
                              nothing else ever calls into those singletons directly. Until
                              detection succeeds, a device's registry entry is a placeholder of
                              whatever type was tried first/most-recently and gets discarded and
                              replaced (not mutated - see DHGenericOpCompleteFn) as the cascade
                              advances; once a real response comes back, that object's type is
                              permanent for the rest of this device's life. These heaters'
                              protocols keep evolving and several variants here are still
                              untested against real hardware, so the goal throughout is that
                              adding a future variant means writing one new handler class and
                              adding it to kProtocolHandlers, not editing switch statements or
                              hunting for loose constants scattered throughout this file.
  0.0.0.2 20260815  fix     - query-only Power/Mode/Temp/Level/Auto commands (value omitted) used
                              to reply with a generic "Done"/"queued" ack, same as a set - the
                              actual cached value only went out on the side-channel MQTT status
                              topic, not in the command's own response. Now answered directly via
                              a new HeaterProtocolHandler::queryValue on each protocol class (e.g.
                              DieselHeaterMode <mac> now replies "Temperature" instead of "Done"),
                              "unsupported" for combinations that aren't meaningful right now (see
                              file header "Not every function is meaningful for every protocol") -
                              see DHQueryOrSet.
*/

/*
Commands (all Tasmota-style: omit the value to query current setting - the command's own JSON
response then echoes that one cached field, e.g. DieselHeaterMode <mac> replies
{"DieselHeaterMode":"Temperature"} instead of the generic {"DieselHeaterMode":"Done"} a set
gets - see HeaterProtocolHandler::queryValue/DHQueryOrSet. Still publishes the usual full status
to stat/.../DieselHeater/<mac> either way, same as any completed op - see "Responses" below):

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
DieselHeaterPassword <mac> [0..9999] - Hcalory MVP2 only: PIN used to seed the first verify
                                       attempt of a chained auth exchange (see "Protocol
                                       handlers" below) - the real PIN is self-learned from the
                                       heater's own announcement if this guess is wrong. Doesn't
                                       touch the wire itself.
DieselHeaterError <mac> [<code>]    - error state, from cache (like the other query commands -
                                       never touches the wire on its own). <code> true/on/1
                                       returns the raw numeric code; omitted or anything else
                                       (including false/off/0) returns the description string
                                       (empty when there's no error) - see
                                       HeaterProtocolHandler::errorDescription.

<mac> accepts a raw hex address or a BLE_ESP32 alias (BLEAlias <mac>=<name>). The first command
referencing a MAC registers it (see "Protocol handlers" below) for periodic polling.

Not every function is meaningful for every protocol - e.g. an AA55-family heater's response
never reports AutoStartStop unless it's the encrypted variant. Unsupported combinations return
a distinct "unsupported" result rather than silently doing nothing.

Protocol handlers: each protocol - GATT UUIDs, wire-format constants, frame building, response
parsing, error code descriptions, and now the parsed status itself - is entirely owned by that
protocol's own HeaterProtocolHandler subclass (private/protected members - see
AA55FamilyHandler/HcaloryHandlerBase below). There is one object PER DEVICE, not per protocol:
a device's registry entry is a heap-allocated instance of whichever concrete subclass its
protocol turned out to be, created once (see "Protocol detection" below) and never freed for
the life of the running firmware (this driver has no device-deregistration path). Only a
handful of fields that every device needs regardless of protocol - MAC, RSSI, cached passkey,
whether it's finished detection - live on the shared HeaterProtocolHandler base itself.
AA55FamilyHandler covers AA55/AA66/their encrypted variants together: which exact sub-variant a
device speaks is only knowable from parsing its response (AA55FamilyHandler::DetectClass), and
all four share UUIDs and frame-building either way. HcaloryMvp1Handler/HcaloryMvp2Handler share
their base class's buildFrame/parseResponse/errorDescription/wire-format helpers and status
fields, MVP2 additionally overriding the chained-auth and Ventilation-escape hooks.

Protocol detection: kProtocolHandlers lists one stateless singleton per protocol, in the order
they should be tried on an unrecognised MAC (matching the Hcalory reference implementation's
own MVP2-first default) - but those singletons are used for exactly one thing, a small
Prototype-pattern factory method (createInstance()), never for building/parsing real traffic.
The first time a MAC is referenced (see DHRegisterDevice) it gets a placeholder device object
of kProtocolHandlers[0]'s type; each time that placeholder's protocol probe fails to get a
response, it's discarded and replaced with a fresh instance of the next candidate's type (see
DHGenericOpCompleteFn) - so at any moment a not-yet-detected device's registry entry is *some*
real, fully-formed object, just possibly not yet the right concrete type. The moment a probe
succeeds, that object's type is locked in forever - a device never changes protocol mid-session,
so the cascade never runs again for it. AA55 vs AA66 vs their encrypted variants is refined from
the first response's own header bytes and reflected in that device's own shortName() (see
AA55FamilyHandler below) purely for display/reporting.

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

// Error code descriptions, cross-checked against the reference implementation's const.py
// (ERROR_NAMES for AA55-family, HCALORY_ERROR_NAMES for Hcalory - the numeric codes are shared
// across each family's protocol variants, only the byte offset they're read from differs, see
// AA55FamilyHandler::DecodeResponse/HcaloryHandlerBase::DecodeResponse). Not directly confirmed
// against real hardware - two different real ignition failures were tested live and neither
// ever set the documented error status at all, and the reference project itself has no
// confirmed real capture of it either (only a synthetic test packet) - so these are
// documentation-derived, not hardware-verified, and may not catch real-world ignition/fuel
// faults despite matching the documented wire format.
// Following this codebase's #ifndef D_X / #define D_X pattern (see e.g. xdrv_81's D_WEBCAM_STATE)
// for Tasmota-translatable strings: a user_config_override.h or a proper language file entry can
// override any of these before this file is compiled. Left as file-scope #defines rather than
// moved into their owning classes like everything else - preprocessor macros don't have C++
// class scope, and these specifically need to stay overridable from outside this file.
#ifndef D_DIESELHEATER_AA55_ERR1
#define D_DIESELHEATER_AA55_ERR1  "E01 - Startup failure"
#define D_DIESELHEATER_AA55_ERR2  "E02 - Lack of fuel"
#define D_DIESELHEATER_AA55_ERR3  "E03 - Supply voltage overrun"
#define D_DIESELHEATER_AA55_ERR4  "E04 - Outlet sensor fault"
#define D_DIESELHEATER_AA55_ERR5  "E05 - Inlet sensor fault"
#define D_DIESELHEATER_AA55_ERR6  "E06 - Pulse pump fault"
#define D_DIESELHEATER_AA55_ERR7  "E07 - Fan fault"
#define D_DIESELHEATER_AA55_ERR8  "E08 - Ignition unit fault"
#define D_DIESELHEATER_AA55_ERR9  "E09 - Overheating"
#define D_DIESELHEATER_AA55_ERR10 "E10 - Overheat sensor fault"
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

#define DIESELHEATER_MAX_DEVICESLOTS 4
#define DIESELHEATER_RETRIES 4
// largest of the AA55 (8), Hcalory password handshake (18), and Hcalory "power" frame shapes
// (12-byte header + 9-byte payload + 1-byte checksum = 22) - was wrongly 18 (a real stack
// buffer overflow: every POWER/STATE/MODE/AUTO frame on Hcalory overran this by 4 bytes,
// corrupting the query byte and checksum on every single one of those commands).
#define DIESELHEATER_FRAME_MAX 22

/*********************************************************************************************\
 * Protocol handlers - one object per DEVICE, not per protocol: a device's registry entry is a
 * heap-allocated instance of whichever concrete class its protocol turned out to be (see
 * "Protocol detection" in the file header comment above), and that object owns both the
 * behaviour (how to speak this protocol) and the data (this device's last known status) - there
 * is no separate device struct/state-union alongside it any more.
 *
 * Everything specific to one protocol - GATT UUIDs, wire-format command/response constants,
 * frame building, response decoding, parsed status fields - is a private (or protected, where a
 * subclass needs it too) member of that protocol's own class, not a free function/#define/plain
 * struct field at file scope: nothing outside AA55FamilyHandler can reach in and use its wire
 * format or fields by accident, and names don't need manual prefixing to stay unique (AA55_Svc/
 * HC_CMD_POWER became just Svc/CMD_POWER).
 *
 * kProtocolHandlers (defined right after this class hierarchy) holds one stateless singleton
 * per protocol, listed in detection-cascade order - but those singletons are used for exactly
 * one thing, createInstance() below, a small Prototype-pattern factory that spins up a fresh,
 * real per-device object of the matching concrete type. Nothing else ever calls into them.
\*********************************************************************************************/

class HeaterProtocolHandler {
 public:
  virtual ~HeaterProtocolHandler() {}

  // Display/log name for this device - e.g. "HcaloryMVP2", or "AA55Family" before detection has
  // even settled which of AA55FamilyHandler's four sub-variants this device actually is (see
  // AA55FamilyHandler::shortName). Used for the JSON "Protocol" field as well as log messages.
  virtual const char *shortName() const = 0;

  virtual void uuids(const char **svc, const char **writeChar, const char **notifyChar) const = 0;

  // Builds the wire frame for a given (cmd, argument) into buf, using this device's own cached
  // state where relevant (e.g. Hcalory's temperature-set frame needs to know which unit this
  // device last reported), and returns its length - or 0 if this cmd isn't supported by this
  // protocol (or is a no-op, e.g. an auto-toggle already at the desired state).
  virtual uint8_t buildFrame(uint8_t cmd, int argument, uint8_t *buf) const = 0;

  // Parses a response frame into this device's own cached state. Returns true on success.
  virtual bool parseResponse(const uint8_t *data, uint8_t len) = 0;

  // Raw numeric error code currently cached for this device (0 = no error) and its description
  // (empty string for no/unknown error) - see the D_DIESELHEATER_* defines above.
  virtual uint8_t errorCodeOf() const = 0;
  virtual const char *errorDescription(uint8_t code) const = 0;

  // Appends this protocol's status fields (RunningState, SetTemp, etc.) to the in-progress
  // Tasmota Response buffer - see DHPublish.
  virtual void appendStatusFields() const = 0;

  // "RunningMode" as shown in appendStatusFields' JSON ("Level"/"Temperature"/"Ventilation"/
  // "Off") - factored out so DHQueryOrSet's DHCMD_MODE query (see queryValue) can reuse the exact
  // same mapping rather than duplicating it.
  virtual const char *modeText() const = 0;

  // Formats this device's cached value for a query-only command (no value given on the command
  // line) into buf, for the immediate command response - see DHQueryOrSet. Returns false if this
  // field isn't meaningful right now (e.g. AutoStartStop on an unencrypted AA55/AA66, or Temp/
  // Level when the heater isn't currently in that mode) rather than returning a stale/meaningless
  // value - callers render false as "unsupported", matching this file's documented convention
  // (see file header "Not every function is meaningful for every protocol").
  virtual bool queryValue(uint8_t cmd, char *buf, uint8_t bufSize) const = 0;

  // A handful of fields the web status page shows generically across every protocol - see
  // DieselHeaterShow. Kept separate from appendStatusFields (which emits full protocol-specific
  // JSON) since the web summary only ever wants these four, in a fixed compact layout.
  virtual uint8_t getRunningState() const = 0;
  virtual float getSupplyVoltage() const = 0;
  virtual uint16_t getCabinTemp() const = 0;
  virtual uint8_t getTempUnit() const = 0; // 0=Celsius, 1=Fahrenheit

  // Does this protocol need the chained verify-then-real-command flow (xdrv_79's
  // chainnextcallback) instead of a plain one-shot op? Only Hcalory MVP2 does - its
  // authentication doesn't survive a reconnect, so auth and the real command have to happen on
  // one continuous connection. See DHOperation/DHChainCallback.
  virtual bool needsChainedAuth() const { return false; }
  // Minimum notify length to accept as the real (non-ack) reply once chained-auth's real
  // command has been sent - see DHNotifyAcceptMinLength.
  virtual uint8_t minResponseLen() const { return 0; }
  virtual uint8_t buildAuthFrame(uint16_t passkey, uint8_t *buf) const { return 0; }
  // Given a just-received notify during the auth stage of a chained op: returns 1 if it shows
  // authenticated (real command can be sent next), 0 if not yet authenticated but a PIN was
  // learned from it (written to *learned, re-verify with that), or -1 if the notify is
  // malformed/unusable (give up chaining for this op).
  virtual int8_t authRoundResult(const uint8_t *notify, uint8_t len, uint16_t *learned) const { return -1; }

  // True if cmd/argument is a Mode-set that needs a power-cycle first because of where this
  // device currently is (e.g. Hcalory MVP2 can't switch directly out of Ventilation) - see
  // DHQueryOrSet.
  virtual bool needsVentilationEscape(uint8_t cmd, int argument) const { return false; }

  // Prototype-pattern factory: spins up a fresh, real per-device instance of this same concrete
  // class. Called only on kProtocolHandlers' stateless singletons (see class-level comment
  // above) - never on a real device object, which has no need to make more of itself.
  virtual HeaterProtocolHandler *createInstance() const = 0;

  // Bookkeeping every device carries regardless of protocol - deliberately plain public fields
  // (matching this file's existing struct-like style) rather than per-field accessors, since
  // every free function in this driver legitimately needs to read/write them directly; only the
  // protocol-specific pieces above are what the user asked to keep properly encapsulated.
  uint8_t addr[7] = {0};
  int8_t RSSI = 0;
  bool stateValid = false;
  bool detected = false;      // false = still working through the detection cascade - this
                               // object's concrete type may still be discarded and replaced
                               // (see DHGenericOpCompleteFn) until this becomes true, after
                               // which it's fixed for the rest of this device's life: a device
                               // never changes protocol mid-session.
  uint8_t candidateIndex = 0;  // this device's current position in kProtocolHandlers -
                               // meaningful only pre-detection, so a failed probe knows where
                               // to resume the cascade
  uint16_t hcaloryPasskey = 0; // Hcalory MVP2 only - PIN used to seed the first verify attempt
                               // of a chained op, set via DieselHeaterPassword; harmlessly
                               // unused on every other protocol. 0 (the default) is itself a
                               // real PIN some devices use, not a "no PIN" sentinel.
};

// Covers AA55, AA66 and their encrypted variants together: which exact sub-variant a device
// speaks is only knowable from parsing its response (DetectClass), and all four share UUIDs and
// frame-building, so there's nothing a per-sub-variant split would actually separate -
// DecodeResponse already branches internally on the detected sub-variant for the pieces that do
// differ (byte offsets, whether the encrypted "extras" fields are present).
//
// Wire format ported from github.com/warehog/esphome-diesel-heater-ble
// (components/diesel_heater_ble/{messages,state}.h).
class AA55FamilyHandler : public HeaterProtocolHandler {
 public:
  // Generic "AA55Family" before detection has settled which of the four sub-variants this
  // device is (variant is still Variant::UNKNOWN at that point); the exact name afterwards.
  const char *shortName() const override {
    switch (variant) {
      case Variant::AA55:     return "AA55";
      case Variant::AA66:     return "AA66";
      case Variant::AA55_ENC: return "AA55Encrypted";
      case Variant::AA66_ENC: return "AA66Encrypted";
      default:                return "AA55Family";
    }
  }

  void uuids(const char **svc, const char **writeChar, const char **notifyChar) const override {
    *svc = Svc;
    *writeChar = Char;
    *notifyChar = Char;
  }

  HeaterProtocolHandler *createInstance() const override { return new AA55FamilyHandler(); }

  uint8_t buildFrame(uint8_t cmd, int argument, uint8_t *buf) const override {
    switch (cmd) {
      case DHCMD_POLL:
      case DHCMD_STATE:
        BuildRequest(buf, 0x01, 0, 0);
        return 8;
      case DHCMD_POWER:
        BuildRequest(buf, 0x03, (1 == argument) ? 1 : 0, 0);
        return 8;
      case DHCMD_MODE: {
        // AA55 mode command (0x02) takes a raw runningmode value; normalized argument here is
        // 0=level,1=temperature,2=ventilation - AA55-family has no explicit ventilation mode in
        // the reference protocol, so that combination isn't supported.
        if (2 == argument) return 0;
        BuildRequest(buf, 0x02, (1 == argument) ? 2 : 0, 0);
        return 8;
      }
      case DHCMD_TEMP: {
        int t = argument;
        if (t < 0) t = 0;
        if (t > 40) t = 40;
        BuildRequest(buf, 0x04, (uint8_t)t, 0);
        return 8;
      }
      case DHCMD_LEVEL: {
        int l = argument;
        if (l < 1) l = 1;
        if (l > 10) l = 10;
        BuildRequest(buf, 0x04, (uint8_t)(l - 1), 0);
        return 8;
      }
      case DHCMD_AUTO:
        // AA55's automatic start/stop (0x13) is a direct set, unlike Hcalory's toggle
        BuildRequest(buf, 0x13, (1 == argument) ? 1 : 0, 0);
        return 8;
      default:
        return 0;
    }
  }

  bool parseResponse(const uint8_t *data, uint8_t len) override {
    if (!DecodeResponse(data, len)) return false;
    stateValid = true;
    variant = DetectClass(data, len); // refine AA55/AA66/encrypted sub-variant, see shortName()
    return true;
  }

  uint8_t errorCodeOf() const override { return errCode; }

  const char *errorDescription(uint8_t code) const override {
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

  const char *modeText() const override { return (0x02 == runningMode) ? "Temperature" : "Level"; }

  // See HeaterProtocolHandler::queryValue. Temp is only meaningful in Temperature mode (matches
  // the same 0x02 gate appendStatusFields uses for "SetTemp"); Level and AutoStartStop follow
  // appendStatusFields' own gating ("SetLevel" is unconditional here, "AutoStartStop" only for
  // the encrypted variants, which are the only ones that report it at all).
  bool queryValue(uint8_t cmd, char *buf, uint8_t bufSize) const override {
    switch (cmd) {
      case DHCMD_POWER:
        snprintf_P(buf, bufSize, PSTR("%s"), runningState ? "ON" : "OFF");
        return true;
      case DHCMD_MODE:
        snprintf_P(buf, bufSize, PSTR("%s"), modeText());
        return true;
      case DHCMD_TEMP:
        if (0x02 != runningMode) return false;
        snprintf_P(buf, bufSize, PSTR("%d"), setTemp);
        return true;
      case DHCMD_LEVEL:
        snprintf_P(buf, bufSize, PSTR("%d"), setLevel);
        return true;
      case DHCMD_AUTO:
        if (!hasExtras) return false;
        snprintf_P(buf, bufSize, PSTR("%s"), isAuto ? "ON" : "OFF");
        return true;
      default:
        return false;
    }
  }

  void appendStatusFields() const override {
    char fbuf[16];
    ResponseAppend_P(PSTR(",\"RunningState\":%d"), runningState);
    ResponseAppend_P(PSTR(",\"RunningStep\":%d"), runningStep);
    ResponseAppend_P(PSTR(",\"RunningMode\":\"%s\""), modeText());
    if (0x02 == runningMode) {
      ResponseAppend_P(PSTR(",\"SetTemp\":%d"), setTemp);
    }
    ResponseAppend_P(PSTR(",\"SetLevel\":%d"), setLevel);
    dtostrfd(supplyVoltage, 1, fbuf);
    ResponseAppend_P(PSTR(",\"SupplyVoltage\":%s"), fbuf);
    ResponseAppend_P(PSTR(",\"CaseTemp\":%d"), caseTemp);
    ResponseAppend_P(PSTR(",\"CabinTemp\":%d"), cabTemp);
    ResponseAppend_P(PSTR(",\"ErrorCode\":%d"), errCode);
    ResponseAppend_P(PSTR(",\"ErrorText\":\"%s\""), errorDescription(errCode));
    ResponseAppend_P(PSTR(",\"Altitude\":%d"), altitude);
    if (hasExtras) {
      ResponseAppend_P(PSTR(",\"AutoStartStop\":%s"), isAuto ? "true" : "false");
      ResponseAppend_P(PSTR(",\"TempUnit\":%d"), tempUnit);
      ResponseAppend_P(PSTR(",\"TankVolume\":%d"), tankVolume);
      ResponseAppend_P(PSTR(",\"OilPumpType\":%d"), oilPumpType);
    }
  }

  uint8_t getRunningState() const override { return runningState; }
  float getSupplyVoltage() const override { return supplyVoltage; }
  uint16_t getCabinTemp() const override { return cabTemp; }
  uint8_t getTempUnit() const override { return tempUnit; } // 0 unless hasExtras (encrypted variants)

 private:
  static constexpr const char *Svc = "0000ffe0-0000-1000-8000-00805f9b34fb";
  static constexpr const char *Char = "0000ffe1-0000-1000-8000-00805f9b34fb"; // write + notify, same characteristic

  // Which exact AA55-family sub-variant this device is - the one piece of protocol identity
  // that can't just be a fixed per-class constant, since this single class covers all four.
  enum class Variant : uint8_t { UNKNOWN, AA55, AA66, AA55_ENC, AA66_ENC };
  Variant variant = Variant::UNKNOWN;

  // which exact AA55-family variant a raw response represents, straight from its own header
  // byte - used both to refine 'variant' (see shortName()) and internally by DecodeResponse to
  // pick the right byte offsets for the pieces that differ between variants.
  static Variant DetectClass(const uint8_t *raw, uint8_t len) {
    if (len < 2) return Variant::UNKNOWN;
    switch (raw[1]) {
      case 0x55: return Variant::AA55;
      case 0x66: return Variant::AA66;
      case 0x34: return Variant::AA55_ENC;
      case 0x07: return Variant::AA66_ENC;
      default:   return Variant::UNKNOWN;
    }
  }

  // XORs (up to) the first 48 bytes, 8 at a time, against ASCII "password" - matches
  // ResponseParser::decrypt() exactly, including its "already 0xAA -> no-op" fast path.
  static void Decrypt(const uint8_t *raw, uint8_t len, uint8_t *out) {
    memcpy(out, raw, len);
    if (raw[0] == 0xAA) return;
    static const uint8_t key[8] = {112, 97, 115, 115, 119, 111, 114, 100}; // "password"
    uint8_t n = (len < 48) ? len : 48;
    for (uint8_t i = 0; i < n; i++) {
      out[i] = (uint8_t)(raw[i] ^ key[i % 8]);
    }
  }

  // Decodes into this device's own fields directly. Only resets the fields it populates below -
  // NOT addr/RSSI/detected/candidateIndex/etc, which now live on this same object rather than a
  // separate heap block, so a blanket memset(this,0,...) is no longer safe here.
  bool DecodeResponse(const uint8_t *raw, uint8_t len) {
    Variant hc = DetectClass(raw, len);
    if (Variant::UNKNOWN == hc) return false;

    uint8_t decrypted[48];
    uint8_t dlen = (len < 48) ? len : 48;
    Decrypt(raw, dlen, decrypted);

    rcvCmd = 0;
    runningState = 0;
    errCode = 0;
    runningStep = 0;
    altitude = 0;
    runningMode = 0;
    setLevel = 0;
    setTemp = 0;
    supplyVoltage = 0;
    caseTemp = 0;
    cabTemp = 0;
    hasExtras = false;
    stTime = 0;
    autoTime = 0;
    runTime = 0;
    isAuto = 0;
    language = 0;
    tempOffset = 0;
    tankVolume = 0;
    oilPumpType = 0;
    rf433OnOff = false;
    tempUnit = 0;
    altiUnit = 0;
    automaticHeating = 0;

    if (dlen > 2) rcvCmd = decrypted[2];
    if (dlen > 3) runningState = decrypted[3];

    // AA66 (unencrypted) error code offset was wrongly 17 - cross-checked against the reference
    // implementation's ProtocolAA55/AA66/AA55Encrypted/AA66Encrypted classes: AA55, AA66 and
    // AA55 encrypted all read the error code from byte 4; only AA66 encrypted differs (byte 35).
    if ((Variant::AA55 == hc || Variant::AA66 == hc || Variant::AA55_ENC == hc) && dlen > 4) {
      errCode = decrypted[4];
    } else if (Variant::AA66_ENC == hc && dlen > 35) {
      errCode = decrypted[35];
    }

    if (dlen > 5) runningStep = decrypted[5];

    if ((Variant::AA55 == hc || Variant::AA66 == hc) && dlen > 7) {
      altitude = decrypted[6] + (decrypted[7] << 8);
    } else if ((Variant::AA55_ENC == hc || Variant::AA66_ENC == hc) && dlen > 7) {
      altitude = (decrypted[7] + (decrypted[6] << 8)) / 10;
    }

    if (dlen > 8) runningMode = decrypted[8];

    if ((Variant::AA55 == hc || Variant::AA66 == hc) && dlen > 10) {
      if (0x00 == runningMode) {
        setLevel = decrypted[10] + 1;
      } else if (0x01 == runningMode) {
        setLevel = decrypted[9];
      } else if (0x02 == runningMode) {
        setTemp = decrypted[9];
        setLevel = decrypted[10] + 1;
      }
    } else if ((Variant::AA55_ENC == hc || Variant::AA66_ENC == hc) && dlen > 10) {
      setLevel = decrypted[10];
      setTemp = decrypted[9];
    }

    if ((Variant::AA55 == hc || Variant::AA66 == hc) && dlen > 12) {
      supplyVoltage = (float)(decrypted[11] + (decrypted[12] << 8)) / 10.0f;
    } else if ((Variant::AA55_ENC == hc || Variant::AA66_ENC == hc) && dlen > 12) {
      supplyVoltage = (float)(decrypted[12] + (decrypted[11] << 8)) / 10.0f;
    }

    if ((Variant::AA55 == hc || Variant::AA66 == hc) && dlen > 16) {
      caseTemp = decrypted[13] + (decrypted[14] << 8);
      cabTemp = decrypted[15] + (decrypted[16] << 8);
    } else if ((Variant::AA55_ENC == hc || Variant::AA66_ENC == hc) && dlen > 33) {
      caseTemp = decrypted[14] + (decrypted[13] << 8);
      cabTemp = (decrypted[33] + (decrypted[32] << 8)) / 10;
    }

    hasExtras = false;
    if ((Variant::AA55_ENC == hc || Variant::AA66_ENC == hc) && dlen > 34) {
      stTime = decrypted[20] + (decrypted[19] << 8);
      autoTime = decrypted[22] + (decrypted[21] << 8);
      runTime = decrypted[24] + (decrypted[23] << 8);
      isAuto = decrypted[25];
      language = decrypted[26];
      tempOffset = decrypted[34];
      tankVolume = decrypted[28];
      oilPumpType = decrypted[29];
      if (29 < len) {
        if (20 == raw[29]) rf433OnOff = false;
        else if (21 == raw[29]) rf433OnOff = true;
      }
      tempUnit = decrypted[27];
      altiUnit = decrypted[30];
      automaticHeating = decrypted[31];
      hasExtras = true;
    }

    return true;
  }

  static void BuildRequest(uint8_t *buf, uint8_t cmd, uint8_t d1, uint8_t d2) {
    buf[0] = 0xAA;
    buf[1] = 0x55;
    buf[2] = 0x0C;
    buf[3] = 0x22;
    buf[4] = cmd;
    buf[5] = d1;
    buf[6] = d2;
    buf[7] = (uint8_t)((0x0C + 0x22 + cmd + d1 + d2) % 256);
  }

  // this device's own last-known status (was a separate heap-allocated aa55_state_t)
  uint8_t rcvCmd = 0;
  uint8_t runningState = 0;
  uint8_t errCode = 0;
  uint8_t runningStep = 0;
  uint16_t altitude = 0;
  uint8_t runningMode = 0;   // raw: 0/1=level, 2=temperature+level (see DecodeResponse)
  uint8_t setLevel = 0;
  uint8_t setTemp = 0;
  float supplyVoltage = 0;
  uint16_t caseTemp = 0;
  uint16_t cabTemp = 0;

  bool hasExtras = false;    // encrypted variants only
  uint16_t stTime = 0;
  uint16_t autoTime = 0;
  uint16_t runTime = 0;
  uint8_t isAuto = 0;
  uint8_t language = 0;
  uint8_t tempOffset = 0;
  uint8_t tankVolume = 0;
  uint8_t oilPumpType = 0;
  bool rf433OnOff = false;
  uint8_t tempUnit = 0;
  uint8_t altiUnit = 0;
  uint8_t automaticHeating = 0;
};

// Shared behaviour for Hcalory MVP1/MVP2 - identical buildFrame/parseResponse/errorDescription/
// wire-format constants/status fields (the MVP1-vs-MVP2 query command difference that used to
// exist here is gone now that both use the same "power" query - see the version-history
// header). Subclasses only differ in UUIDs and their fixed protocol id, and MVP2 additionally
// in needing chained auth.
//
// Wire format ported from ProtocolHcalory in
// github.com/Spettacolo83/homeassistant-diesel-heater
// (diesel_heater_ble/src/diesel_heater_ble/protocol.py - not that repo's own HCALORY.md doc
// page, which disagrees with the real code on gear-level range, checksum span, and the entire
// response byte layout - see this file's version-history header).
class HcaloryHandlerBase : public HeaterProtocolHandler {
 public:
  uint8_t buildFrame(uint8_t cmd, int argument, uint8_t *buf) const override {
    switch (cmd) {
      case DHCMD_PASSWORD:
        return BuildPasswordHandshake(buf, hcaloryPasskey);
      case DHCMD_POLL:
      case DHCMD_STATE: {
        // Real capture evidence (issue #34): dpID 0A0A is time-sync only - the actual full
        // status reply comes from this same "power" query on both MVP1 and MVP2.
        uint8_t payload[9] = {0,0,0,0,0,0,0,0, POWER_QUERY};
        return BuildCmd(buf, CMD_POWER, payload, sizeof(payload));
      }
      case DHCMD_POWER: {
        uint8_t arg = (2 == argument) ? POWER_QUERY /* no hw toggle for power - treat as query, see note below */
                    : (1 == argument) ? POWER_ON : POWER_OFF;
        uint8_t payload[9] = {0,0,0,0,0,0,0,0, arg};
        return BuildCmd(buf, CMD_POWER, payload, sizeof(payload));
      }
      case DHCMD_MODE: {
        if (2 == argument) return 0; // no explicit ventilation-mode set command on Hcalory either
        uint8_t arg = (1 == argument) ? POWER_MODE_TEMP : POWER_MODE_LEVEL;
        uint8_t payload[9] = {0,0,0,0,0,0,0,0, arg};
        return BuildCmd(buf, CMD_POWER, payload, sizeof(payload));
      }
      case DHCMD_TEMP: {
        int t = argument;
        if (t < 0) t = 0;
        if (t > 40) t = 40;
        uint8_t unitByte = stateValid ? tempUnit : 0;
        uint8_t payload[2] = {(uint8_t)t, unitByte};
        return BuildCmd(buf, CMD_SET_TEMP, payload, sizeof(payload));
      }
      case DHCMD_LEVEL: {
        int l = argument;
        if (l < 1) l = 1;
        if (l > 10) l = 10;
        uint8_t payload[1] = {(uint8_t)l};
        return BuildCmd(buf, CMD_SET_GEAR, payload, sizeof(payload));
      }
      case DHCMD_AUTO: {
        // Hcalory's wire command is toggle-only - only actually send it if the requested state
        // differs from the last known one. Needs a valid cached state to work from.
        if (!stateValid) return 0;
        bool want = (1 == argument);
        if (want == autoStartStop) return 0; // already in the desired state
        uint8_t payload[9] = {0,0,0,0,0,0,0,0, POWER_AUTO_TOGGLE};
        return BuildCmd(buf, CMD_POWER, payload, sizeof(payload));
      }
      default:
        return 0;
    }
  }

  bool parseResponse(const uint8_t *data, uint8_t len) override {
    if (!DecodeResponse(data, len)) return false;
    stateValid = true;
    return true;
  }

  uint8_t errorCodeOf() const override { return errorCode; }

  const char *errorDescription(uint8_t code) const override {
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

  const char *modeText() const override {
    return (0x1 == setMode) ? "Temperature" : (0x2 == setMode) ? "Level" : (0x3 == setMode) ? "Ventilation" : "Off";
  }

  // See HeaterProtocolHandler::queryValue. Temp/Level gating mirrors appendStatusFields'
  // setValueNone/setMode checks exactly (Level covers both the Level and Ventilation setModes,
  // same as appendStatusFields' "else" branch) - AutoStartStop is always meaningful on Hcalory,
  // unlike AA55Family, so it's never "unsupported" here.
  bool queryValue(uint8_t cmd, char *buf, uint8_t bufSize) const override {
    switch (cmd) {
      case DHCMD_POWER:
        snprintf_P(buf, bufSize, PSTR("%s"), runningState ? "ON" : "OFF");
        return true;
      case DHCMD_MODE:
        snprintf_P(buf, bufSize, PSTR("%s"), modeText());
        return true;
      case DHCMD_TEMP:
        if (setValueNone || 0x1 != setMode) return false;
        snprintf_P(buf, bufSize, PSTR("%d"), setTemp);
        return true;
      case DHCMD_LEVEL:
        if (setValueNone || 0x1 == setMode) return false;
        snprintf_P(buf, bufSize, PSTR("%d"), setLevel);
        return true;
      case DHCMD_AUTO:
        snprintf_P(buf, bufSize, PSTR("%s"), autoStartStop ? "ON" : "OFF");
        return true;
      default:
        return false;
    }
  }

  void appendStatusFields() const override {
    char fbuf[16];
    ResponseAppend_P(PSTR(",\"RunningState\":%d"), runningState);
    ResponseAppend_P(PSTR(",\"RunningStep\":%d"), runningStep);
    ResponseAppend_P(PSTR(",\"RunningMode\":\"%s\""), modeText());
    if (!setValueNone) {
      if (0x1 == setMode) {
        ResponseAppend_P(PSTR(",\"SetTemp\":%d"), setTemp);
      } else {
        ResponseAppend_P(PSTR(",\"SetLevel\":%d"), setLevel);
      }
    }
    ResponseAppend_P(PSTR(",\"AutoStartStop\":%s"), autoStartStop ? "true" : "false");
    dtostrfd(supplyVoltage, 1, fbuf);
    ResponseAppend_P(PSTR(",\"SupplyVoltage\":%s"), fbuf);
    ResponseAppend_P(PSTR(",\"CaseTemp\":%d"), caseTemp);
    ResponseAppend_P(PSTR(",\"CabinTemp\":%d"), cabTemp);
    ResponseAppend_P(PSTR(",\"TempUnit\":%d"), tempUnit);
    ResponseAppend_P(PSTR(",\"ErrorCode\":%d"), errorCode);
    ResponseAppend_P(PSTR(",\"ErrorText\":\"%s\""), errorDescription(errorCode));
  }

  uint8_t getRunningState() const override { return runningState; }
  float getSupplyVoltage() const override { return supplyVoltage; }
  uint16_t getCabinTemp() const override { return cabTemp; }
  uint8_t getTempUnit() const override { return tempUnit; }

 protected:
  // MVP2 password handshake: 00 02 00 01 00 01 00 0A 0C 00 00 05 01 [D1 D2 D3 D4] [checksum] -
  // protected, not private: HcaloryMvp2Handler::buildAuthFrame also needs this directly. Doesn't
  // touch instance state, so stays a plain static helper.
  static uint8_t BuildPasswordHandshake(uint8_t *buf, uint16_t passkey) {
    buf[0] = 0x00;
    buf[1] = 0x02;
    buf[2] = 0x00;
    buf[3] = 0x01;
    buf[4] = 0x00;
    buf[5] = 0x01;
    buf[6] = 0x00;
    buf[7] = 0x0A;
    buf[8] = 0x0C;
    buf[9] = 0x00;
    buf[10] = 0x00;
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

 private:
  enum : uint16_t {
    CMD_SET_GEAR = 0x0607,
    CMD_SET_TEMP = 0x0706,
    CMD_POWER    = 0x0E04,
  };
  enum : uint8_t {
    POWER_QUERY       = 0x00,
    POWER_ON          = 0x02,
    POWER_OFF         = 0x01,
    POWER_AUTO_TOGGLE = 0x05,
    POWER_MODE_LEVEL  = 0x07,
    POWER_MODE_TEMP   = 0x06,
  };

  // Decodes into this device's own fields directly - see the equivalent note on
  // AA55FamilyHandler::DecodeResponse about why this only resets the fields it populates.
  bool DecodeResponse(const uint8_t *data, uint8_t len) {
    if (len < 38) return false;

    runningState = 0;
    runningStep = 0;
    setMode = 0;
    setValueNone = false;
    setTemp = 0;
    setLevel = 0;
    autoStartStop = false;
    supplyVoltage = 0;
    caseTemp = 0;
    cabTemp = 0;
    highAltitude = 0;
    tempUnit = 0;
    errorCode = 0;

    uint8_t complete_state_byte = data[20];
    uint8_t status = (complete_state_byte & 0xF0) >> 4;
    uint8_t running_step_raw = complete_state_byte & 0x0F;

    runningState = (0x0 == status || 0xF == status) ? 0 : 1;

    if (0x4 == status) {
      runningStep = 4; // synthetic cooldown, matches reference exactly
    } else {
      switch (running_step_raw) {
        case 0x1: runningStep = 6; break; // fan/ventilation
        case 0x3: runningStep = 2; break; // ignition
        case 0x5: runningStep = 3; break; // running
        case 0x7: runningStep = 0; break; // standby
        default:  runningStep = 0; break; // inactive/standby
      }
    }

    setMode = data[21];

    uint8_t set_value_raw = data[22];
    if (0x0 == status || 0x4 == status || 0xF == status) {
      setValueNone = true;
    } else if (0x1 == setMode) {
      setTemp = set_value_raw;
    } else {
      uint8_t lvl = set_value_raw;
      if (lvl < 1) lvl = 1;
      if (lvl > 10) lvl = 10;
      setLevel = lvl;
    }

    autoStartStop = (1 == data[23]);
    supplyVoltage = (float)(((uint16_t)data[24] << 8) | data[25]) / 10.0f;
    caseTemp = (((uint16_t)data[27] << 8) | data[28]) / 10;
    cabTemp = (((uint16_t)data[30] << 8) | data[31]) / 10;
    highAltitude = data[18];
    tempUnit = data[37];
    errorCode = (0xF == status) ? set_value_raw : 0;

    return true;
  }

  // general command frame: 00 02 00 01 00 01 00 [cmd_hi] [cmd_lo] 00 00 [payload_len] [payload...] [checksum]
  // checksum = sum(bytes from index 8 onward) & 0xFF - NOT the whole frame
  static uint8_t BuildCmd(uint8_t *buf, uint16_t cmd_type, const uint8_t *payload, uint8_t payload_len) {
    uint8_t cmd_hi = (cmd_type >> 8) & 0xFF;
    uint8_t cmd_lo = cmd_type & 0xFF;

    buf[0] = 0x00;
    buf[1] = 0x02;
    buf[2] = 0x00;
    buf[3] = 0x01;
    buf[4] = 0x00;
    buf[5] = 0x01;
    buf[6] = 0x00;
    buf[7] = cmd_hi;
    buf[8] = cmd_lo;
    buf[9] = 0x00;
    buf[10] = 0x00;
    buf[11] = payload_len;
    for (uint8_t i = 0; i < payload_len; i++) buf[12 + i] = payload[i];

    uint16_t sum = 0;
    for (uint8_t i = 8; i < 12 + payload_len; i++) sum += buf[i];
    buf[12 + payload_len] = (uint8_t)(sum & 0xFF);

    return 12 + payload_len + 1;
  }

 protected:
  // this device's own last-known status (was a separate heap-allocated hc_state_t) - protected,
  // not private: HcaloryMvp2Handler::needsVentilationEscape needs setMode directly.
  uint8_t runningState = 0;
  uint8_t runningStep = 0;
  uint8_t setMode = 0;      // raw: 0=off,1=temperature,2=level,3=ventilation
  bool setValueNone = false;
  uint8_t setTemp = 0;
  uint8_t setLevel = 0;
  bool autoStartStop = false;
  float supplyVoltage = 0;
  uint16_t caseTemp = 0;
  uint16_t cabTemp = 0;
  uint8_t highAltitude = 0;
  uint8_t tempUnit = 0;
  uint8_t errorCode = 0;
};

class HcaloryMvp1Handler : public HcaloryHandlerBase {
 public:
  const char *shortName() const override { return "HcaloryMVP1"; }
  void uuids(const char **svc, const char **writeChar, const char **notifyChar) const override {
    *svc = Svc;
    *writeChar = WriteChar;
    *notifyChar = NotifyChar;
  }
  HeaterProtocolHandler *createInstance() const override { return new HcaloryMvp1Handler(); }

 private:
  static constexpr const char *Svc = "0000fff0-0000-1000-8000-00805f9b34fb";
  static constexpr const char *WriteChar = "0000fff2-0000-1000-8000-00805f9b34fb";
  static constexpr const char *NotifyChar = "0000fff1-0000-1000-8000-00805f9b34fb";
};

class HcaloryMvp2Handler : public HcaloryHandlerBase {
 public:
  const char *shortName() const override { return "HcaloryMVP2"; }
  void uuids(const char **svc, const char **writeChar, const char **notifyChar) const override {
    *svc = Svc;
    *writeChar = WriteChar;
    *notifyChar = NotifyChar;
  }
  HeaterProtocolHandler *createInstance() const override { return new HcaloryMvp2Handler(); }

  bool needsChainedAuth() const override { return true; }
  uint8_t minResponseLen() const override { return 38; }

  uint8_t buildAuthFrame(uint16_t passkey, uint8_t *buf) const override {
    return BuildPasswordHandshake(buf, passkey);
  }

  // The heater proactively announces its OWN current password, unprompted, on first contact
  // (see file header "Protocol detection") - too short to be that announcement at all is treated
  // the same as "not authenticated yet". Confirmed live: the heater accepts any handshake
  // attempt as an acknowledgment regardless of correctness - only a verify using the exact PIN
  // it just told us gets a real authenticated (byte 17 == 1) confirmation back.
  int8_t authRoundResult(const uint8_t *notify, uint8_t len, uint16_t *learned) const override {
    if (len < 18) return -1;
    if (1 == notify[17]) return 1;
    if (notify[13] > 9 || notify[14] > 9 || notify[15] > 9 || notify[16] > 9) return -1;
    *learned = (uint16_t)(notify[13] * 1000 + notify[14] * 100 + notify[15] * 10 + notify[16]);
    return 0;
  }

  // Confirmed live: switching directly out of Ventilation via the mode-set command is
  // acknowledged ("result":"ok") but has no actual effect. Power-cycling first reliably works,
  // and - also confirmed live - turning off from Ventilation (no combustion to purge) takes
  // effect immediately, unlike the multi-minute cooldown after actually heating, so no extra
  // wait is needed - see DHQueryOrSet.
  bool needsVentilationEscape(uint8_t cmd, int argument) const override {
    return (DHCMD_MODE == cmd) && (0 == argument || 1 == argument) && stateValid && (0x3 == setMode);
  }

 private:
  static constexpr const char *Svc = "0000bd39-0000-1000-8000-00805f9b34fb";
  static constexpr const char *WriteChar = "0000bdf7-0000-1000-8000-00805f9b34fb";
  static constexpr const char *NotifyChar = "0000bdf8-0000-1000-8000-00805f9b34fb";
};

// One stateless singleton per protocol - used only as a createInstance() factory, see the
// class-level comment on HeaterProtocolHandler above.
AA55FamilyHandler aa55FamilyHandlerInstance;
HcaloryMvp1Handler hcaloryMvp1HandlerInstance;
HcaloryMvp2Handler hcaloryMvp2HandlerInstance;

// Every available protocol, tried in this order on first contact with an unrecognised device -
// matching the reference implementation's own MVP2-first default. Adding support for a new
// protocol variant means writing one new HeaterProtocolHandler subclass and adding it here -
// nothing else in this file needs to change.
HeaterProtocolHandler *const kProtocolHandlers[] = {
  &hcaloryMvp2HandlerInstance,
  &hcaloryMvp1HandlerInstance,
  &aa55FamilyHandlerInstance,
};
#define NUM_PROTOCOL_HANDLERS (sizeof(kProtocolHandlers) / sizeof(*kProtocolHandlers))

/*********************************************************************************************\
 * Device registry - dynamic array of per-device instance pointers (see "Protocol handlers"
 * above). Devices are allocated once, on first reference, and kept for as long as the firmware
 * keeps running - this driver has no deregistration path - so the total number of heap
 * allocations here is small, fixed by DIESELHEATER_MAX_DEVICESLOTS, and one-time: exactly the
 * kind of heap use that doesn't risk fragmentation.
\*********************************************************************************************/

std::vector<HeaterProtocolHandler*> DeviceRegistry;

int DieselHeaterPeriod = 300;
int seconds = 20;
// out-of-range sentinel (registry is capped at DIESELHEATER_MAX_DEVICESLOTS, so this is never a
// valid index) - holds automatic polling off until the first `seconds<=0` reset, giving the
// system a startup grace period before it starts hammering BLE, regardless of how early a
// device gets registered via a manual command - see DieselHeaterEverySecond.
int nextPoll = DIESELHEATER_MAX_DEVICESLOTS;
int opInProgress = 0;
int retries = 0;

#pragma pack( push, 1 )
struct op_t {
  uint8_t addr[7];
  uint8_t cmd;      // DHCmd - the user's intent, not raw wire bytes, so it can be rebuilt
  int16_t argument; // meaning depends on cmd
};
#pragma pack(pop)

std::deque<DIESELHEATER_ESP32::op_t*> opQueue;

// explicit prototype: Arduino's auto-prototype generator scans the concatenated source and
// inserts forward declarations before locally-defined types are defined, without reliably
// tracking namespace/preprocessor-guard context - see xdrv_79's CONFIG_NIMBLE_CPP_IDF fix and
// this project's own history for the same class of bug.
HeaterProtocolHandler *findOrRegisterDevice(const uint8_t *addr);
bool DHOperation(const uint8_t *MAC, uint8_t cmd, int argument, HeaterProtocolHandler *device);
void DHReplaceDevice(HeaterProtocolHandler *oldDevice, HeaterProtocolHandler *newDevice);

const char *addrStr(const uint8_t *addr) {
  static char addrstr[32];
  BLE_ESP32::dump(addrstr, 13, addr, 6);
  return addrstr;
}

// Finds this MAC's device object, or registers a new placeholder for it (of kProtocolHandlers[0]'s
// type - the detection cascade will discard and replace it with the right concrete type, see
// DHGenericOpCompleteFn, if that guess turns out wrong). Returns nullptr only if
// DIESELHEATER_MAX_DEVICESLOTS is already full.
HeaterProtocolHandler *findOrRegisterDevice(const uint8_t *addr) {
  for (auto *device : DeviceRegistry) {
    if (!memcmp(device->addr, addr, 6)) return device;
  }
  if (DeviceRegistry.size() >= DIESELHEATER_MAX_DEVICESLOTS) {
    AddLog(LOG_LEVEL_ERROR, PSTR("DHT: %s: lost > %d devices"), addrStr(addr), DIESELHEATER_MAX_DEVICESLOTS);
    return nullptr;
  }
  HeaterProtocolHandler *device = kProtocolHandlers[0]->createInstance();
  memcpy(device->addr, addr, 6);
  DeviceRegistry.push_back(device);
  return device;
}

// Swaps oldDevice's registry slot for newDevice and frees oldDevice - used only mid-cascade,
// when a probe against the current candidate protocol fails and detection moves on to try the
// next one (see DHGenericOpCompleteFn). Never called once a device is detected.
void DHReplaceDevice(HeaterProtocolHandler *oldDevice, HeaterProtocolHandler *newDevice) {
  for (auto &slot : DeviceRegistry) {
    if (slot == oldDevice) {
      slot = newDevice;
      break;
    }
  }
  delete oldDevice;
}

int DHGenericOpCompleteFn(BLE_ESP32::generic_sensor_t *pStruct);
void DHPublishNoOp(const uint8_t *addr, const char *cmdName);
void DHPublish(const uint8_t *addr, const char *cmdName, bool success);
bool DHNotifyAcceptMinLength(BLE_ESP32::generic_sensor_t *op);
bool DHChainCallback(BLE_ESP32::generic_sensor_t *op);

// Scratch state for the in-flight chained op (verify -> real command, one connection). A single
// file-static instance matches this framework's current one-op-in-flight-at-a-time model (see
// BLEOperationTask) - if that ever changes, this needs to move to per-op storage.
// Deliberately holds NO pointers into DeviceRegistry/opQueue: DHChainCallback/
// DHNotifyAcceptMinLength run on the BLE task thread, not the main thread, so they must not
// touch driver state those are read/written from (see DHQueueOp's comment on why chaining
// exists at all) - 'device' is a plain pointer to whichever device object the current op is
// running against. Only one op is ever in flight at a time, and a device is only ever
// discarded/replaced (DHReplaceDevice) from DHGenericOpCompleteFn once that op has already
// finished - so this pointer is always valid for as long as anything might actually read it,
// even for a still-undetected device's chained first probe. Safe to read from any thread
// without synchronization, same as the rest of this struct's plain fields.
struct {
  HeaterProtocolHandler *device; // which device this chained op is running against
  uint8_t stage;                 // 0 = verifying password, 1 = real command sent
  uint8_t realFrame[DIESELHEATER_FRAME_MAX];
  uint8_t realFrameLen;
  uint8_t verifyAttempts;
} dhChain;

bool DHNotifyAcceptMinLength(BLE_ESP32::generic_sensor_t *op) {
  return dhChain.device && (op->notifylen >= dhChain.device->minResponseLen());
}

// chainnextcallback for protocols needing chained auth - see the comment on chainnextcallback
// in xdrv_79 for the contract. Called on the BLE task thread right after each accepted notify.
bool DHChainCallback(BLE_ESP32::generic_sensor_t *op) {
  if (!dhChain.device) return false;

  if (0 == dhChain.stage) {
    uint16_t learned = 0;
    int8_t result = dhChain.device->authRoundResult(op->dataNotify, op->notifylen, &learned);
    if (1 == result) {
      // authenticated - send the real command next, and now require a real (non-ack) reply.
      memcpy(op->dataToWrite, dhChain.realFrame, dhChain.realFrameLen);
      op->writelen = dhChain.realFrameLen;
      op->notifyacceptcallback = (void *)DHNotifyAcceptMinLength;
      dhChain.stage = 1;
      return true;
    }
    if (0 == result) {
      // not authenticated - re-verify with the PIN just learned. Bounded so a malformed/stuck
      // exchange still finishes (as a failure) instead of looping.
      if (dhChain.verifyAttempts >= 5) return false;
      op->writelen = dhChain.device->buildAuthFrame(learned, op->dataToWrite);
      dhChain.verifyAttempts++;
      return true;
    }
    return false; // -1: malformed/unusable - give up chaining, finishes/reports normally
  }
  // stage 1: notifyacceptcallback already required this to be a real (non-ack) reply - done.
  return false;
}

bool DHOperation(const uint8_t *MAC, uint8_t cmd, int argument, HeaterProtocolHandler *device) {
  if (!device) return false;

  uint8_t frame[DIESELHEATER_FRAME_MAX];
  uint8_t frameLen;
  // Chained protocols (currently just Hcalory MVP2) need verify-then-real-command on one
  // connection (see DHQueueOp/DHChainCallback); DHCMD_PASSWORD itself (no longer queued
  // externally, but buildFrame still supports it) stays a plain one-shot op.
  bool chained = device->needsChainedAuth() && (DHCMD_PASSWORD != cmd);

  if (chained) {
    dhChain.device = device;
    dhChain.stage = 0;
    dhChain.verifyAttempts = 0;
    dhChain.realFrameLen = device->buildFrame(cmd, argument, dhChain.realFrame);
    if (0 == dhChain.realFrameLen) return false; // not supported / no-op for this cmd
    frameLen = device->buildAuthFrame(device->hcaloryPasskey, frame);
  } else {
    frameLen = device->buildFrame(cmd, argument, frame);
    if (0 == frameLen) return false; // not supported / no-op (e.g. auto toggle already in desired state)
  }

  BLE_ESP32::generic_sensor_t *op = nullptr;
  int res = BLE_ESP32::newOperation(&op);
  if (!res) {
    AddLog(LOG_LEVEL_ERROR, PSTR("DHT: %s: Can't get a newOperation \"%s\" from BLE"), addrStr(MAC), cmdnames[cmd]);
    return false;
  }

  const char *svc, *writeChar, *notifyChar;
  device->uuids(&svc, &writeChar, &notifyChar);

  NimBLEAddress addr((uint8_t *)MAC, 0); // type 0 is public
  op->addr = addr;
  op->serviceUUID = NimBLEUUID(svc);
  op->characteristicUUID = NimBLEUUID(writeChar);
  op->notificationCharacteristicUUID = NimBLEUUID(notifyChar);

  op->writelen = frameLen;
  memcpy(op->dataToWrite, frame, frameLen);

  op->completecallback = (void *)DHGenericOpCompleteFn;
  // pack cmd (low byte) and argument (top 16 bits) into the plain uint32_t context the
  // completion callback recovers everything from - which device this was for isn't packed in
  // here at all, it's recovered by MAC (op->addr) via findOrRegisterDevice instead, same as the
  // BLE framework already does for every other driver.
  op->context = (void *)(uint32_t)(cmd | ((uint32_t)(uint16_t)argument << 16));

  if (chained) {
    op->chainnextcallback = (void *)DHChainCallback;
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
  HeaterProtocolHandler *device = findOrRegisterDevice(op->addr);
  if (!device) {
    // registry full - nowhere to track this op's result, so there's nothing safe to do but
    // drop it (see findOrRegisterDevice, which has already logged why).
    opQueue.pop_front();
    delete op;
    return DHDoOp();
  }

  if (DHOperation(op->addr, op->cmd, op->argument, device)) {
    opInProgress = 1;
    retries = DIESELHEATER_RETRIES;
    // op stays in the queue (still opQueue[0]) until the completion callback pops it, so a
    // retry/next-candidate attempt can find it again - see DHGenericOpCompleteFn.
    return 1;
  } else {
    // buildFrame returned "no-op" (unsupported combo, or an auto-toggle already at the desired
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

  HeaterProtocolHandler *device = findOrRegisterDevice(addr);
  if (device) {
    ResponseAppend_P(PSTR(",\"RSSI\":%d"), device->RSSI);
    if (device->stateValid) {
      ResponseAppend_P(PSTR(",\"Protocol\":\"%s\""), device->shortName());
      device->appendStatusFields();
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
  int argument = (int16_t)(ctx >> 16);
  const char *cmdName = (cmd < (sizeof(cmdnames) / sizeof(*cmdnames))) ? cmdnames[cmd] : "invalid";

  HeaterProtocolHandler *device = findOrRegisterDevice(addrev);
  // the queued op_t (still at opQueue[0]) is what drives retries/cascade - only pop it once we
  // stop needing to try again for it.
  op_t *queued = opQueue.size() ? opQueue[0] : nullptr;

  if (op->state <= GEN_STATE_FAILED) {
    // still probing an unrecognised device - advance to the next candidate rather than just
    // retrying the same (almost certainly wrong) one: discard this placeholder and replace it
    // with a fresh instance of the next candidate's type.
    if (device && !device->detected) {
      uint8_t nextIndex = device->candidateIndex + 1;
      if (nextIndex < NUM_PROTOCOL_HANDLERS && retries > 1) {
        retries--;
        HeaterProtocolHandler *replacement = kProtocolHandlers[nextIndex]->createInstance();
        memcpy(replacement->addr, device->addr, sizeof(replacement->addr));
        replacement->RSSI = device->RSSI;
        replacement->hcaloryPasskey = device->hcaloryPasskey;
        replacement->candidateIndex = nextIndex;
        AddLog(LOG_LEVEL_DEBUG, PSTR("DHT: %s: %s not found, trying %s"), addrStr(addrev),
               device->shortName(), replacement->shortName());
        DHReplaceDevice(device, replacement);
        device = replacement;
        if (DHOperation(addrev, cmd, argument, device)) {
          opInProgress = 1;
          return 0;
        }
      }
    }
    if (device && retries > 1) {
      retries--;
      if (DHOperation(addrev, cmd, argument, device)) {
        AddLog(LOG_LEVEL_ERROR, PSTR("DHT: %s: operation \"%s\" failed - retries left: %d"), addrStr(addrev), cmdName, retries);
        opInProgress = 1;
        return 0;
      }
    }
    retries = 0;
    if (queued) {
      opQueue.pop_front();
      delete queued;
    }
    if (DHCMD_PASSWORD != cmd) DHPublish(addrev, cmdName, false);
    AddLog(LOG_LEVEL_ERROR, PSTR("DHT: %s: operation \"%s\" failed - no more retries/candidates"), addrStr(addrev), cmdName);
    DHDoOp();
    return 0;
  }

  retries = 0;
  if (queued) {
    opQueue.pop_front();
    delete queued;
  }

  if (device) {
    // Detection only ever happens once - a device can't change protocol mid-session, so once
    // 'detected' is set it's never touched again; this response just confirmed the current
    // object's type is the right one.
    if (!device->detected) {
      device->detected = true;
      AddLog(LOG_LEVEL_INFO, PSTR("DHT: %s: detected as %s"), addrStr(addrev), device->shortName());
    }
    if (op->notifylen > 0) { // the password-handshake op has no meaningful status reply to parse
      device->parseResponse(op->dataNotify, op->notifylen);
    }
  }

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
  DieselHeaterPeriod = Settings->tele_period;
}

void DieselHeaterEverySecond(void) {
  seconds--;
  if (seconds <= 0) {
    if (DieselHeaterPeriod) {
      if (nextPoll >= (int)DeviceRegistry.size()) {
        nextPoll = 0;
      }
    }
    seconds = DieselHeaterPeriod;
  }

  if (DieselHeaterPeriod && (nextPoll < (int)DeviceRegistry.size())) {
    if (!opQueue.size() && !opInProgress) {
      DHQueueOp(DeviceRegistry[nextPoll]->addr, DHCMD_POLL, 0);
      nextPoll++;
    }
  }

  DHDoOp();
}

/*********************************************************************************************\
 * Commands - Tasmota-style: value omitted means "query current cached setting", never triggers
 * a wire read (see file header "Caching" note). Only DieselHeaterState forces a fresh read.
\*********************************************************************************************/

const char *responses[] = { PSTR("Done"), PSTR("queued"), PSTR("invaddr"), PSTR("nostate"), PSTR("unsupported") };

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

  HeaterProtocolHandler *device = findOrRegisterDevice(addrbin);
  if (!device) { ResponseCmndChar(responses[2]); return; }

  if (rest && rest[0]) {
    int pin = atoi(rest);
    if (pin < 0) pin = 0;
    if (pin > 9999) pin = 9999;
    device->hcaloryPasskey = (uint16_t)pin;
  }
  ResponseCmndNumber(device->hcaloryPasskey);
}

// DieselHeaterError <mac> [<code>] - <code> true/on/1 (case-insensitive) returns the raw
// numeric error code; omitted or anything else (false/off/0/unrecognized) returns the
// description (see HeaterProtocolHandler::errorDescription), empty string when there's no error.
void CmndDieselHeaterError(void) {
  uint8_t addrbin[7];
  char *rest;
  if (!DHParseMac(addrbin, &rest)) { ResponseCmndChar(responses[2]); return; }

  HeaterProtocolHandler *device = findOrRegisterDevice(addrbin);
  if (!device || !device->stateValid) { ResponseCmndChar(responses[3]); return; }

  bool wantCode = rest && rest[0]
    && (!strcasecmp(rest, "true") || !strcasecmp(rest, "on") || !strcmp(rest, "1"));

  uint8_t code = device->errorCodeOf();

  if (wantCode) {
    ResponseCmndNumber(code);
  } else {
    ResponseCmndChar(device->errorDescription(code));
  }
}

// shared implementation for the query-or-set commands
void DHQueryOrSet(uint8_t cmd, int (*parseArg)(const char *)) {
  uint8_t addrbin[7];
  char *rest;
  if (!DHParseMac(addrbin, &rest)) { ResponseCmndChar(responses[2]); return; }

  HeaterProtocolHandler *device = findOrRegisterDevice(addrbin);

  if (!rest || !rest[0]) {
    // query - answer from cache only, never touch the wire. DHPublish still sends the full
    // status JSON to the usual stat/.../DieselHeater/<mac> topic (as any op completion would);
    // the command's own response below is just this one field, e.g. DieselHeaterMode <mac> with
    // no value replies with the current mode instead of a generic "Done" - see queryValue.
    if (!device || !device->stateValid) { ResponseCmndChar(responses[3]); return; }
    DHPublish(addrbin, cmdnames[cmd], true);
    char valbuf[16];
    if (device->queryValue(cmd, valbuf, sizeof(valbuf))) {
      ResponseCmndChar(valbuf);
    } else {
      ResponseCmndChar(responses[4]);
    }
    return;
  }

  int argument = parseArg(rest);
  if (argument < 0) { ResponseCmndChar(responses[2]); return; } // reusing invaddr for "bad value" too

  // Relies on DHQueueOp synchronously dispatching (via DHDoOp) whenever nothing else is in
  // flight, and buildFrame never failing for DHCMD_POWER on a confirmed protocol - so the "off"
  // push below is always already dispatched (and thus skipped by write coalescing, which only
  // scans pending/non-dispatched entries) before the "on" push is queued. If that ever stops
  // holding, coalescing would wrongly merge these into just "on".
  if (device && device->needsVentilationEscape(cmd, argument)) {
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
  for (size_t i = 0; i < DeviceRegistry.size(); i++) {
    HeaterProtocolHandler *device = DeviceRegistry[i];
    if (!first) WSContentSend_P(HTTP_SNS_HR_THIN);
    first = false;

    char label[16];
    snprintf_P(label, sizeof(label), PSTR("DH-%d"), (int)i + 1);
    WSContentSend_P(HTTP_DH_MAC, label, addrStr(device->addr));
    if (device->stateValid) {
      WSContentSend_P(HTTP_DH_STATE, label, device->getRunningState());
      char fbuf[16];
      dtostrfd(device->getSupplyVoltage(), 1, fbuf);
      WSContentSend_P(HTTP_DH_VOLTAGE, label, fbuf);
      WSContentSend_P(HTTP_DH_CABTEMP, label, device->getCabinTemp(),
                       device->getTempUnit() ? D_UNIT_FAHRENHEIT[0] : D_UNIT_CELSIUS[0]);
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
