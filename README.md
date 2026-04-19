# SpeeduinoConsole

PlatformIO and VS Code project for reading Speeduino data and showing it on a 20x4 character LCD.

This repository was cloned from the original GitHub project so the existing Git history stays intact, and the files are now organized for day-to-day development in VS Code.

## Current Lomax Version

- Uses Speeduino's lowercase `"n"` command instead of the older `"a"` request, so the helper receives the larger fixed secondary-serial data block.
- Uses the helper Arduino's hardware `Serial1` port (`RX1 = 19`, `TX1 = 18`) instead of a `SoftwareSerial` port.
- Uses a stricter packet read loop that validates the `n2` header, tracks payload length, rejects incomplete or overlong packets, and recovers faster from bad serial data.

## Project Layout

- `src/main.cpp`: main Arduino application
- `platformio.ini`: PlatformIO environment for an Arduino Mega 2560 helper board
- `docs/howToGetStarted.txt`: wiring and setup notes for the current Lomax hardware
- `docs/images/`: reference photos and diagrams

## Features

- Uses Speeduino's enhanced `"n"` command for the larger real-time data block
- Expects the Speeduino secondary serial protocol to be set to `Generic (Fixed List)`
- Reads Speeduino through helper-board `Serial1` on pins 19/18
- Detects invalid, incomplete, and unexpectedly long packets before updating the display
- Displays fuel pressure on the last line
- Optionally reads GPS over helper-board `Serial2` for speed and clock display
- Keeps the firmware logic in a single `main.cpp` while the project is still evolving

## GPS Time And Speed

GPS support is enabled by default in `platformio.ini` with `-DENABLE_GPS=1`.
The current hardware uses an Adafruit Mini GPS PA1010D in UART mode.

Wiring on the helper Arduino Mega 2560:

- GPS `TXO` -> helper Arduino `RX2` pin 17
- GPS `RXI` -> helper Arduino `TX2` pin 16, optional for the current read-only use
- GPS `VIN`/`5V` and `GND` -> suitable helper-board power and common ground

The GPS clock starts from UTC. A manually selected UTC offset is stored in EEPROM
address `0` as a value from `0` to `23`. If EEPROM contains any other value at
startup, the firmware initializes it to `2`, which means `UTC+02`.

The LCD GPS field normally shows time and speed when a valid GPS fix is fresh:

```text
HH:MM ssskm/h
```

If GPS data is expected but no recent data is received, the GPS field shows
`GPS?`. If there is no valid fix yet, the field is intentionally quiet.

### Adjusting The Clock Offset

The clock offset is adjusted with the throttle position reported by Speeduino.
Speeduino's `TPS` value is scaled from `0` to `200`.

To enter clock adjustment:

1. Turn ignition on and leave the engine stopped.
2. Wait until GPS has a valid fix/time.
3. Make sure the TPS value has first been low, below `20`.
4. Press the throttle to `TPS >= 190` and hold it for 20 seconds.

When adjustment mode is active, the GPS field changes to:

```text
UTC+02 HH:MM
```

While adjusting:

- `TPS <= 100`: pause, so the displayed time can be checked
- `TPS > 100`: step the offset forward by one hour repeatedly
- 10 seconds without `TPS > 100`: save the offset to EEPROM and return to the normal GPS field

The step interval is calculated from the current TPS value on a straight line
between `(TPS 100, 3000 ms)` and `(TPS 200, 500 ms)`. In other words, a little
throttle steps slowly and full throttle steps quickly.

Examples:

| TPS | Step interval |
| --- | ---: |
| 120 | 2500 ms |
| 140 | 2000 ms |
| 160 | 1500 ms |
| 180 | 1000 ms |
| 200 | 500 ms |

## Development

1. Open this folder in VS Code, or open `speeduino_console.code-workspace`.
2. Install the recommended extensions when VS Code asks for them.
3. Build and upload with the `megaatmega2560` PlatformIO environment.
4. If you move the project to a non-Mega helper board, update the serial-port choice in `src/main.cpp` because the current wiring expects hardware `Serial1`.

## Dependency

The LCD support uses a vendored subset of fmalpartida's `New-LiquidCrystal` library in `lib/NewLiquidCrystal`, which keeps the original behavior while avoiding unused upstream sources that break AVR builds in PlatformIO.
