# Bluetooth Reconnect Notes

This project started as an ESP32 Classic Bluetooth A2DP source for an ESP32 Feather V2. The target receiver used during debugging was a Hyundai head unit. The important result is that reliable power-loss reconnect requires the ESP32 to expose a minimal HFP Audio Gateway profile in addition to A2DP/AVRCP.

## Current State

- ESP32 runs ESP-IDF under PlatformIO.
- Board environment is `adafruit_feather_esp32_v2`.
- USB serial is UART0 through the board USB-UART bridge.
- Firmware can stream either I2S input or built-in simulated voice samples.
- Python control UI can scan, select discovered devices, connect, enter PIN, and send play/stop/simulate commands.
- The ESP32 stores the last paired Bluetooth address in NVS.
- Bluetooth controller BR/EDR max ACL connections is set to `2`, which is needed for A2DP plus AVRCP.
- The ESP32 now exposes:
  - A2DP source
  - AVRCP controller
  - Minimal HFP Audio Gateway
- For the Hyundai head unit, pairing must include HFP. Selecting HFP on the head unit automatically adds A2DP/media.

## Key Discovery

The Hyundai head unit does not reliably auto-reconnect to an A2DP-only paired device after ESP32 power loss. Manual head-unit initiated media connection can work, but automatic reconnect is tied to the head unit's phone profile policy.

Working behavior:

1. ESP32 advertises phone/media identity and HFP AG.
2. Head unit is paired with HFP enabled.
3. After ESP32 power restore, the head unit reconnects HFP first.
4. The head unit then opens A2DP automatically.

This means HFP is not optional for this target. It is the anchor profile the car uses to remember and reconnect the device.

## Failed Paths

### A2DP-only Active Reconnect

The ESP32 tried to reconnect directly to the saved bonded receiver address using `esp_a2d_source_connect()`. The repeated failure looked like this:

```text
A2DP_CONNECTION:1:6d:e3:4a:05:9d:01
BT_SDP: SDP - Rcvd conn cnf with error: 0x4
BT_HCI: hcif conn complete: hdl 0xfff, st 0x4
ACL_CONNECTED:bda=6d:e3:4a:05:9d:01 status=0x104 handle=4095
BTA_AV_OPEN_EVT::FAILED status: 2
A2DP_CONNECTION:0:6d:e3:4a:05:9d:01
```

Interpretation: this is not a PIN or A2DP codec problem. It is a baseband/page timeout. The head unit is not page-scanning or not accepting the outbound A2DP connection at that time.

### Explicit SDP Probe Before A2DP

Trying `esp_bt_gap_get_remote_services()` before `esp_a2d_source_connect()` produced repeated SDP failures and did not improve reconnect:

```text
AUTO_RECONNECT_SDP_START
REMOTE_SERVICES:bda=6d:e3:4a:05:9d:01 status=0x1 count=0
AUTO_RECONNECT_SDP_FAILED
```

The firmware now lets `esp_a2d_source_connect()` run its own A2DP open sequence instead of probing SDP first.

### Passive A2DP-only Mode

Keeping the ESP32 connectable and non-discoverable allowed manual reconnect from the head unit, but did not cause automatic reconnect unless the head unit had paired the ESP32 as an HFP/phone device.

## Fixes Made

### HFP Audio Gateway

Minimal HFP AG was added so the ESP32 looks like a phone-side device to the head unit. The firmware responds to basic HFP status queries with idle/no-call/network-available state. SCO audio is not used.

Expected logs include:

```text
HFP_AG_PROFILE:state=...
HFP_AG_CONNECTION:state=...
```

### Phone/Media Class Of Device

The firmware sets a phone/media Class of Device:

```text
BT_COD:PHONE_MEDIA service=0x3c0 major=0x02 minor=0x03
```

This matches the desired behavior better than an unspecified default identity.

### ACL Connections

The ESP-IDF A2DP source example notes that A2DP plus AVRCP usually needs two BR/EDR ACL connections. The config was changed from `1` to `2`:

```ini
CONFIG_BTDM_CTRL_BR_EDR_MAX_ACL_CONN=2
CONFIG_BTDM_CTRL_BR_EDR_MAX_ACL_CONN_EFF=2
CONFIG_BTDM_CONTROLLER_BR_EDR_MAX_ACL_CONN=2
CONFIG_BTDM_CONTROLLER_BR_EDR_MAX_ACL_CONN_EFF=2
```

### A2DP Callback Disconnect Crash

A crash occurred when pressing Disconnect on the head unit. Bluedroid calls the A2DP data callback with `data == NULL` and `len == -1` during transmit flush. The callback now handles this case:

```c
if (data == NULL || len <= 0) {
    return 0;
}
```

Without that guard, `memset(data, 0, len)` caused `StoreProhibited`.

## Pairing Procedure

When HFP support or Bluetooth profile identity changes, clear pairing on both sides:

1. Send `CLEAR_PAIRING` to the ESP32.
2. Delete/forget the ESP32 from the Hyundai head unit.
3. Put the head unit into pairing mode.
4. Scan and connect from the Python UI.
5. On the head unit, ensure HFP/phone profile is selected. Media/A2DP should be added automatically.
6. Power-cycle the ESP32 and verify automatic reconnect.

## Useful Logs

Successful reconnect should show HFP first, then A2DP:

```text
HFP_AG_CONNECTION:state=...
A2DP_CONNECTION:2:...
```

If the log only shows A2DP active reconnect attempts with `st 0x4` or `status=0x104`, the head unit is still not treating the ESP32 as an auto-reconnect phone device.

## Current Practical Rule

For this Hyundai target, A2DP-only pairing is insufficient for automatic reconnect after ESP32 power loss. HFP pairing is required, and A2DP follows automatically from the head unit after HFP reconnects.

## AVRCP Simulation Player

The built-in simulated voice samples now behave as a three-track playlist:

1. `Left`
2. `Right`
3. `Both`

In simulation mode, the selected sample loops with one second of silence between repeats. AVRCP Target passthrough commands from the head unit are handled as media-player controls:

Each simulated playlist item is reported as a 15 second track over AVRCP. The short built-in sample repeats for the duration of that 15 second track window, then the existing one second silence is inserted before the same selected track loops again. This avoids exposing unrealistically short media durations to the head unit.

- Play: start the selected simulated track.
- Pause/Stop: stop media.
- Next: select and play the next track.
- Previous: select and play the previous track.

Expected logs:

```text
AVRCP_TG_CONNECTION:connected=1 ...
AVRCP_CMD:key=0x4b state=0
SIM_TRACK:1:Right
```

The ESP-IDF public AVRCP Target API supports passthrough controls and register notifications, but not application-supplied `GET_ELEMENT_ATTR` title metadata or `GET_PLAY_STATUS` responses. In ESP-IDF 5.5.0, Bluedroid rejected those PDUs internally before calling the application.

Current local PlatformIO package patch:

- `/Users/dimitri/.platformio/packages/framework-espidf/components/bt/host/bluedroid/btc/profile/std/avrc/btc_avrc.c` now answers `GET_PLAY_STATUS`, `GET_ELEMENT_ATTR`, and `INFORM_DISPLAY_CHARSET`.
- The same patch also fills `REGISTER_NOTIFICATION` responses for play status, track change, play position, and battery status. ESP-IDF 5.5.0's local target response path only handled volume change by default, so play/pause state changes were silently dropped even when the head unit had registered for them.
- The firmware exports weak-hook replacements from `src/main.c` for current title, track index, play status, song length, and song position.
- Metadata currently reports title (`Left`, `Right`, `Both`), artist (`ESP32 A2DP`), track number, and playing time.
- Firmware now tracks `STOPPED`, `PAUSED`, and `PLAYING` separately instead of deriving all non-playing states from `audio_mode`. This keeps head-unit play/pause and stop buttons closer to AVRCP semantics.

This is a local framework package patch. A PlatformIO framework update can overwrite it, so keep this note if the radio loses metadata again after package updates.

See `bluedroid-avrcp-patch.md` for the exact stock Bluedroid changes needed to reproduce the patch on another machine.
