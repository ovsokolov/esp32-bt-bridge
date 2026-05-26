# ESP-A2DP

ESP-IDF / PlatformIO project for an ESP32 Feather V2 acting as a Classic Bluetooth A2DP source.
For car head units, the firmware also exposes a minimal HFP Audio Gateway profile because some receivers only auto-reconnect devices paired as phones.

## Hardware

- Board: Adafruit ESP32 Feather V2 (`adafruit_feather_esp32_v2`)
- USB serial: UART0 through the board USB-UART bridge at 115200 baud
- Onboard RGB NeoPixel: GPIO0 by default (`RGB_LED_GPIO` in `platformio.ini`)
- I2S input defaults:
  - BCLK: GPIO26
  - WS/LRCK: GPIO25
  - DIN: GPIO34

Change I2S pins in `platformio.ini` if your microphone/ADC wiring differs.

## Firmware

Commands are newline-delimited over UART:

- `PLAY` starts streaming from I2S input.
- `STOP` sends silence and pauses the stream.
- `SIMULATE` streams the built-in voice test clips copied from `I2S_generator`.
- `SCAN` scans for Classic Bluetooth devices and prints `DEVICE:<bda>:<name>` lines.
- `CONNECT:<bda>` connects to the selected Bluetooth sink.
- `PIN:<digits>` replies to a legacy pairing PIN request.
- `STATUS` prints current Bluetooth/audio state.

The firmware stores Bluetooth bonding data in NVS. After a successful pairing, the ESP32 attempts to reconnect to the last sink after boot.
For the Hyundai head unit used during development, HFP must be selected during pairing; the head unit then reconnects HFP first and opens A2DP automatically.
The onboard NeoPixel blinks green while no pairing is stored and stays solid blue once paired.

See `bluetooth-reconnect-notes.md` for the reconnect investigation and current profile behavior.

Build/upload when PlatformIO is available:

```bash
pio run -e adafruit_feather_esp32_v2
pio run -e adafruit_feather_esp32_v2 -t upload
```

## Host control UI

Install dependency:

```bash
python3 -m pip install pyserial
```

Run:

```bash
python3 tools/a2dp_control.py
```

The UI decodes ESP-IDF log lines, lists discovered devices from `DEVICE:` lines, provides PIN entry, and has Scan, Connect Device, Play, Stop, Simulate, and Status buttons.
