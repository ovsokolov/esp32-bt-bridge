# ESP-IDF Bluedroid Patches

These patches are generated against ESP-IDF `6.0.1`, the version reported by the current PlatformIO `framework-espidf` package.

They patch PlatformIO's installed framework package, not this project's source tree. A PlatformIO framework reinstall or update can overwrite these changes.

## Files

- `framework-espidf-6.0.1-bluedroid-avrcp-target.patch`
  - Adds AVRCP Target app hooks for play status, song length, song position, track title, artist, and track index.
  - Allows the target to answer `GET_PLAY_STATUS`, `GET_ELEMENT_ATTR`, `INFORM_DISPLAY_CHARSET`, and `LIST_PLAYER_APP_ATTR`.
  - Enables and fills register-notification responses used by the car UI.
  - Lets `bta_av_act.c` pass these metadata PDUs through instead of logging them as unhandled.

- `framework-espidf-6.0.1-bluedroid-hfp-ag-chld.patch`
  - Adds HFP AG three-way-call feature advertising.
  - Forwards car `AT+CHLD` commands to the application as `ESP_HF_UNAT_RESPONSE_EVT` strings like `+CHLD=2`.

## Apply

From this repo root:

```bash
FRAMEWORK=/Users/olegsokolov/.platformio/packages/framework-espidf
patch -d "$FRAMEWORK" -p1 < patches/framework-espidf-6.0.1-bluedroid-avrcp-target.patch
patch -d "$FRAMEWORK" -p1 < patches/framework-espidf-6.0.1-bluedroid-hfp-ag-chld.patch
```

Then rebuild both projects so PlatformIO recompiles the patched Bluedroid files:

```bash
/Users/olegsokolov/.platformio/penv/bin/pio run -d ESP-A2DP
/Users/olegsokolov/.platformio/penv/bin/pio run -d ESP-A2DP-HF
```

