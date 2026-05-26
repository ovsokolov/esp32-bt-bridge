# ESP32 Bluetooth Bridge Context

This repository contains both sides of the ESP32 Bluetooth bridge:

- `ESP-A2DP-HF`: HF/phone side. A2DP sink, HFP HF, AVRCP CT.
- `ESP-A2DP`: AG/car side. A2DP source, HFP AG, AVRCP TG.

The bridge forwards media audio over wired I2S and control/metadata over UART between the two ESP32 boards.

## Current Audio Wiring

A2DP I2S pins:

- BCLK: GPIO5
- WS/LRCLK: GPIO25
- HF DOUT: GPIO19
- AG DIN: GPIO21

HFP SCO PCM pins currently share the same physical clock/data pins:

- HF is PCM master.
- AG is PCM slave.
- CLK: GPIO5
- FSYNC: GPIO25
- HF DOUT -> AG DIN: GPIO19 -> GPIO21
- AG DOUT -> HF DIN: GPIO19 -> GPIO21
- Common GND is required.

## Current Known-Good Areas

- A2DP audio uses ring buffers and larger I2S DMA buffers to reduce underruns.
- HFP SCO audio is configured for CVSD 8 kHz PCM.
- PCM frame sync is currently Mono LF (`CONFIG_BTDM_CTRL_PCM_FSYNCSHP_MONO_MODE_LF=y`, `EFF=1`) on both projects.
- A2DP is suspended while HFP calls exist and resumes when calls end.
- AVRCP metadata now retries after track changes, auto-advance, play-start, and position reset cases.

## Useful Tools

The UI/control tool source lives in:

- `ESP-A2DP/tools/bridge_control.py`
- `ESP-A2DP-HF/tools/bridge_control.py`

Helper scripts:

- `ESP-A2DP/tools/run_bridge_control.sh`
- `ESP-A2DP-HF/tools/run_bridge_control.sh`
- `ESP-A2DP/tools/a2dp_control.py`
- `ESP-A2DP-HF/tools/a2dp_control.py`

## Build Commands

AG:

```sh
cd ESP-A2DP
/Users/olegsokolov/.platformio/penv/bin/pio run
```

HF:

```sh
cd ESP-A2DP-HF
/Users/olegsokolov/.platformio/penv/bin/pio run
```

## Latest Test Tags

- HF: `hf-avrcp-auto-track-refresh-20260526`
- AG: `ag-avrcp-metadata-refresh-20260526`

