# Bluedroid AVRCP Target Patch

This project currently depends on a local ESP-IDF Bluedroid patch because the stock PlatformIO ESP-IDF 5.5.0 package does not expose enough AVRCP Target hooks for the Hyundai head unit test case.

Patch targets:

```text
/Users/dimitri/.platformio/packages/framework-espidf/components/bt/host/bluedroid/btc/profile/std/avrc/btc_avrc.c
/Users/dimitri/.platformio/packages/framework-espidf/components/bt/host/bluedroid/bta/av/bta_av_act.c
```

On another machine, use the same relative path under that machine's PlatformIO package directory:

```text
~/.platformio/packages/framework-espidf/components/bt/host/bluedroid/btc/profile/std/avrc/btc_avrc.c
~/.platformio/packages/framework-espidf/components/bt/host/bluedroid/bta/av/bta_av_act.c
```

## Purpose

The stock file rejected these AVRCP Target PDUs before the app could provide data:

- `AVRC_PDU_GET_PLAY_STATUS`
- `AVRC_PDU_GET_ELEMENT_ATTR`
- `AVRC_PDU_INFORM_DISPLAY_CHARSET`
- `AVRC_PDU_LIST_PLAYER_APP_ATTR`

It also accepted registration for `ESP_AVRC_RN_PLAY_STATUS_CHANGE` and `ESP_AVRC_RN_TRACK_CHANGE`, but `btc_avrc_tg_send_rn_rsp()` only populated volume-change responses. Play/pause and track-change notifications were therefore dropped as unhandled events.

A second car test exposed two more Bluedroid issues:

- `bta_av_proc_meta_cmd()` logged metadata PDUs such as `0x30` (`GET_PLAY_STATUS`) and `0x11` (`LIST_PLAYER_APP_ATTR`) as unhandled before the BTC AVRCP handler could respond.
- `esp_avrc_tg_set_rn_evt_cap()` returned `ESP_ERR_NOT_SUPPORTED` (`0x106`) because the stock target allowed only volume-change notifications.

## Change 1: Include `stdio.h`

Add `stdio.h` near the top of `btc_avrc.c` because the metadata response uses `snprintf()`:

```c
#include "common/bt_target.h"
#include <stdio.h>
#include <string.h>
```

## Change 2: Add Weak App Hooks

Add these weak functions after the static forward declarations, before the static variables. The firmware overrides them from `src/main.c`.

```c
__attribute__((weak)) UINT8 esp_avrc_tg_app_play_status(void)
{
    return AVRC_PLAYSTATE_PAUSED;
}

__attribute__((weak)) UINT32 esp_avrc_tg_app_song_len_ms(void)
{
    return 0;
}

__attribute__((weak)) UINT32 esp_avrc_tg_app_song_pos_ms(void)
{
    return 0;
}

__attribute__((weak)) UINT8 esp_avrc_tg_app_track_index(void)
{
    return 0;
}

__attribute__((weak)) const char *esp_avrc_tg_app_track_title(void)
{
    return "ESP32";
}
```

Firmware-side overrides currently return:

- play status: stopped, playing, or paused
- song length: `15000` ms for the simulated tracks
- song position: current position in the simulated 15 second track
- track index: `0`, `1`, or `2`
- title: `Left`, `Right`, or `Both`

## Change 3: Add Metadata Helpers

Add these helpers after `handle_rc_get_play_status_rsp()` and before `handle_rc_metamsg_cmd()`:

```c
static BOOLEAN app_metadata_attr_requested(const tAVRC_GET_ELEM_ATTRS_CMD *cmd, UINT32 attr_id)
{
    if (cmd->num_attr == 0) {
        return TRUE;
    }

    for (UINT8 i = 0; i < cmd->num_attr; i++) {
        if (cmd->attrs[i] == attr_id) {
            return TRUE;
        }
    }
    return FALSE;
}

static void app_metadata_add_attr(tAVRC_ATTR_ENTRY *attrs, UINT8 *num_attr,
                                  UINT32 attr_id, const char *value)
{
    if (value == NULL) {
        value = "";
    }
    attrs[*num_attr].attr_id = attr_id;
    attrs[*num_attr].name.charset_id = AVRC_CHARSET_ID_UTF8;
    attrs[*num_attr].name.str_len = (UINT16)strlen(value);
    attrs[*num_attr].name.p_str = (UINT8 *)value;
    (*num_attr)++;
}
```

## Change 4: Replace Rejection of Status/Metadata PDUs

In `btc_rc_upstreams_evt()`, stock code rejected `AVRC_PDU_GET_PLAY_STATUS`, `AVRC_PDU_GET_ELEMENT_ATTR`, and `AVRC_PDU_INFORM_DISPLAY_CHARSET` together with the player-app-setting PDUs.

Replace that part with explicit handlers:

```c
case AVRC_PDU_GET_PLAY_STATUS: {
    tAVRC_RESPONSE avrc_rsp;
    memset(&avrc_rsp, 0, sizeof(tAVRC_RESPONSE));
    avrc_rsp.get_play_status.opcode = opcode_from_pdu(AVRC_PDU_GET_PLAY_STATUS);
    avrc_rsp.get_play_status.pdu = AVRC_PDU_GET_PLAY_STATUS;
    avrc_rsp.get_play_status.status = AVRC_STS_NO_ERROR;
    avrc_rsp.get_play_status.song_len = esp_avrc_tg_app_song_len_ms();
    avrc_rsp.get_play_status.song_pos = esp_avrc_tg_app_song_pos_ms();
    avrc_rsp.get_play_status.play_status = esp_avrc_tg_app_play_status();
    send_metamsg_rsp(btc_rc_cb.rc_handle, label, ctype, &avrc_rsp);
}
break;
case AVRC_PDU_GET_ELEMENT_ATTR: {
    char track_num[8];
    char playing_time[16];
    tAVRC_ATTR_ENTRY attrs[4];
    UINT8 num_attr = 0;
    tAVRC_RESPONSE avrc_rsp;
    snprintf(track_num, sizeof(track_num), "%u",
             (unsigned)(esp_avrc_tg_app_track_index() + 1));
    snprintf(playing_time, sizeof(playing_time), "%u",
             (unsigned)esp_avrc_tg_app_song_len_ms());

    if (app_metadata_attr_requested(&pavrc_cmd->get_elem_attrs, AVRC_MEDIA_ATTR_ID_TITLE)) {
        app_metadata_add_attr(attrs, &num_attr, AVRC_MEDIA_ATTR_ID_TITLE,
                              esp_avrc_tg_app_track_title());
    }
    if (app_metadata_attr_requested(&pavrc_cmd->get_elem_attrs, AVRC_MEDIA_ATTR_ID_ARTIST)) {
        app_metadata_add_attr(attrs, &num_attr, AVRC_MEDIA_ATTR_ID_ARTIST,
                              "ESP32 A2DP");
    }
    if (app_metadata_attr_requested(&pavrc_cmd->get_elem_attrs, AVRC_MEDIA_ATTR_ID_TRACK_NUM)) {
        app_metadata_add_attr(attrs, &num_attr, AVRC_MEDIA_ATTR_ID_TRACK_NUM,
                              track_num);
    }
    if (app_metadata_attr_requested(&pavrc_cmd->get_elem_attrs, AVRC_MEDIA_ATTR_ID_PLAYING_TIME)) {
        app_metadata_add_attr(attrs, &num_attr, AVRC_MEDIA_ATTR_ID_PLAYING_TIME,
                              playing_time);
    }

    memset(&avrc_rsp, 0, sizeof(tAVRC_RESPONSE));
    avrc_rsp.get_elem_attrs.opcode = opcode_from_pdu(AVRC_PDU_GET_ELEMENT_ATTR);
    avrc_rsp.get_elem_attrs.pdu = AVRC_PDU_GET_ELEMENT_ATTR;
    avrc_rsp.get_elem_attrs.status = AVRC_STS_NO_ERROR;
    avrc_rsp.get_elem_attrs.num_attr = num_attr;
    avrc_rsp.get_elem_attrs.p_attrs = attrs;
    send_metamsg_rsp(btc_rc_cb.rc_handle, label, ctype, &avrc_rsp);
}
break;
case AVRC_PDU_INFORM_DISPLAY_CHARSET: {
    tAVRC_RESPONSE avrc_rsp;
    memset(&avrc_rsp, 0, sizeof(tAVRC_RESPONSE));
    avrc_rsp.rsp.opcode = opcode_from_pdu(AVRC_PDU_INFORM_DISPLAY_CHARSET);
    avrc_rsp.rsp.pdu = AVRC_PDU_INFORM_DISPLAY_CHARSET;
    avrc_rsp.rsp.status = AVRC_STS_NO_ERROR;
    send_metamsg_rsp(btc_rc_cb.rc_handle, label, ctype, &avrc_rsp);
}
break;
```

Return an empty player-app-settings attribute list for `AVRC_PDU_LIST_PLAYER_APP_ATTR`. Some car media players ask for it before deciding whether to show media controls.

```c
case AVRC_PDU_LIST_PLAYER_APP_ATTR: {
    tAVRC_RESPONSE avrc_rsp;
    memset(&avrc_rsp, 0, sizeof(tAVRC_RESPONSE));
    avrc_rsp.list_app_attr.opcode = opcode_from_pdu(AVRC_PDU_LIST_PLAYER_APP_ATTR);
    avrc_rsp.list_app_attr.pdu = AVRC_PDU_LIST_PLAYER_APP_ATTR;
    avrc_rsp.list_app_attr.status = AVRC_STS_NO_ERROR;
    avrc_rsp.list_app_attr.num_attr = 0;
    send_metamsg_rsp(btc_rc_cb.rc_handle, label, ctype, &avrc_rsp);
}
break;
```

Leave the remaining unsupported player-app-setting PDUs rejected:

```c
case AVRC_PDU_LIST_PLAYER_APP_VALUES:
case AVRC_PDU_GET_CUR_PLAYER_APP_VALUE:
case AVRC_PDU_GET_PLAYER_APP_ATTR_TEXT:
case AVRC_PDU_GET_PLAYER_APP_VALUE_TEXT: {
    send_reject_response (btc_rc_cb.rc_handle, label, pavrc_cmd->pdu, AVRC_STS_BAD_CMD);
}
break;
```

## Change 5: Fill Registered Notification Responses

In `btc_avrc_tg_send_rn_rsp()`, extend the `switch (event_id)` so registered notification responses are not limited to volume:

```c
switch (event_id) {
case ESP_AVRC_RN_VOLUME_CHANGE:
    avrc_rsp.reg_notif.param.volume = param->volume;
    break;
case ESP_AVRC_RN_PLAY_STATUS_CHANGE:
    avrc_rsp.reg_notif.param.play_status = param->playback;
    break;
case ESP_AVRC_RN_TRACK_CHANGE:
    memcpy(avrc_rsp.reg_notif.param.track, param->elm_id,
           sizeof(avrc_rsp.reg_notif.param.track));
    break;
case ESP_AVRC_RN_PLAY_POS_CHANGED:
    avrc_rsp.reg_notif.param.play_pos = param->play_pos;
    break;
case ESP_AVRC_RN_BATTERY_STATUS_CHANGE:
    avrc_rsp.reg_notif.param.battery_status = param->batt;
    break;
// todo: implement other event notifications
default:
    BTC_TRACE_WARNING("%s : Unhandled event ID : 0x%x", __FUNCTION__, event_id);
    return;
}
```

This is the part that fixes play/pause UI drift. Without it, the app can receive `REGISTER_NOTIFICATION` and send `esp_avrc_tg_send_rn_rsp()`, but Bluedroid returns before sending play-status or track-change payloads.

## Change 6: Allow Target RN Events Beyond Volume

In `btc_avrc.c`, expand the target allowed registered-notification bitmask. Stock PlatformIO ESP-IDF allows only volume change (`0x2000`), so this project's `esp_avrc_tg_set_rn_evt_cap()` call fails with `0x106` when trying to advertise play-status and track-change support.

Replace:

```c
const static uint16_t cs_rn_allowed_evt = \
        0x2000;
```

with:

```c
const static uint16_t cs_rn_allowed_evt = \
        0x2066; /* play_status, track_chg, pos_chg, batt_status, volume_chg */
```

The important bits are:

- bit `1`: `ESP_AVRC_RN_PLAY_STATUS_CHANGE`
- bit `2`: `ESP_AVRC_RN_TRACK_CHANGE`
- bit `5`: `ESP_AVRC_RN_PLAY_POS_CHANGED`
- bit `6`: `ESP_AVRC_RN_BATTERY_STATUS_CHANGE`
- bit `13`: `ESP_AVRC_RN_VOLUME_CHANGE`

## Change 7: Forward Metadata PDUs Through BTA

In `bta_av_act.c`, update `bta_av_proc_meta_cmd()` so Bluedroid does not warn and treat these commands as unhandled before BTC can process them.

Add these cases near `AVRC_PDU_SET_PLAYER_APP_VALUE`:

```c
case AVRC_PDU_LIST_PLAYER_APP_ATTR:
case AVRC_PDU_GET_ELEMENT_ATTR:
case AVRC_PDU_GET_PLAY_STATUS:
case AVRC_PDU_INFORM_DISPLAY_CHARSET:
    break;
```

Without this change, runtime logs can show:

```text
BT_APPL: bta_av_proc_meta_cmd unhandled RC vendor PDU: 0x11
BT_APPL: bta_av_proc_meta_cmd unhandled RC vendor PDU: 0x30
```

`0x11` is `LIST_PLAYER_APP_ATTR`; `0x30` is `GET_PLAY_STATUS`.

## Verification

After patching another machine, rebuild so PlatformIO recompiles both patched Bluedroid files:

```bash
/Users/dimitri/.platformio/penv/bin/pio run -e adafruit_feather_esp32_v2
```

Expected build output should include:

```text
Compiling .pio/build/adafruit_feather_esp32_v2/bt/host/bluedroid/bta/av/bta_av_act.c.o
Compiling .pio/build/adafruit_feather_esp32_v2/bt/host/bluedroid/btc/profile/std/avrc/btc_avrc.c.o
```

Useful runtime logs:

```text
AVRCP_TG_RN:play_status,track
AVRCP_REGISTER_NOTIFICATION:event=0x01
AVRCP_REGISTER_NOTIFICATION:event=0x02
AVRCP_PLAY_STATUS_CHANGED:1
AVRCP_PLAY_STATUS_CHANGED:2
AVRCP_PLAY_STATUS_CHANGED:0
```

Status values:

- `0`: stopped
- `1`: playing
- `2`: paused

These logs should be gone:

```text
AVRCP_TG_RN_FAILED:0x106
bta_av_proc_meta_cmd unhandled RC vendor PDU: 0x11
bta_av_proc_meta_cmd unhandled RC vendor PDU: 0x30
```

## Maintenance Warning

This patch modifies PlatformIO's installed ESP-IDF package, not the project source tree. PlatformIO package reinstall/update can overwrite it. If metadata or play/pause synchronization regresses after a dependency update, reapply this patch first.

## HFP AG Multi-Call Testing Patch

For call hold, swap, and merge testing, this project also patches Bluedroid HFP AG so the application can see `AT+CHLD` from the car.

File:

```text
/Users/olegsokolov/.platformio/packages/framework-espidf/components/bt/host/bluedroid/btc/profile/std/hf_ag/btc_hf_ag.c
```

Changes:

- include `<stdio.h>`
- add `BTA_AG_FEAT_3WAY` to `BTC_HF_FEATURES` so the car sees call hold/multiparty support
- handle `BTA_AG_AT_CHLD_EVT`
- forward it to the app as `ESP_HF_UNAT_RESPONSE_EVT` with a command string like `+CHLD=2`

The app uses a fixed call table with up to 6 simulated calls. `+CLCC` is generated from the table with stable indexes 1-6, direction, status, number, and multiparty flag.

Design note: keep the call table as the source of truth and treat call-control commands as short phone-like transactions. A real phone does not apply every repeated plain `AT+CHLD` command as an independent user action while the first state transition is still settling. The simulator should process the first plain `CHLD=0/1/2/3`, update indicators and `+CLCC`, and ignore/coalesce immediate repeats while the transaction is pending. The pending transaction is completed when the car asks for `CLCC` and receives the updated table, or after a timeout if no `CLCC` arrives. If a car sends `CHLD=2` followed immediately by `CHLD=1` for its End action, restore the pre-transaction table and apply `CHLD=1` from that original state. This is intentional behavior, not a temporary workaround. Indexed commands such as `CHLD=11` and `CHLD=22` should stay explicit call-table operations.

The app maps:

- `+CHLD=0`: release waiting or held call
- `+CHLD=1`: release active call and accept waiting/held call
- `+CHLD=2`: hold active call and accept/swap waiting/held call
- `+CHLD=3`: merge calls
- `+CHLD=1x`: release call index `x`
- `+CHLD=2x`: make call index `x` private/active and hold the others

Project UART commands for manual testing:

```text
CALL_IN
CALL_IN:<number>
CALL_IN2
CALL_IN2:<number>
CALL_IN:<number> repeated adds another incoming/waiting call until the 6-call table is full
CALL_ANSWER
CALL_REJECT
CALL_END
CALL_SWAP
CALL_MERGE
CALL_RELEASE_WAITING
CALL_RELEASE_ACTIVE
```

The Python control app exposes buttons for second incoming call, swap/hold, merge, release waiting/held, and release active.
