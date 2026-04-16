# Baseline display and packet references

This document captures the pre-refactor behavior of `src/main.cpp`.
It is a reference for later decode/render tests and GPS work. It does not
describe a new desired layout.

## Timing baseline

Current constants:

- `PACKET_READ_TIMEOUT = 100UL`
- `UNEXPECTED_BYTES_WAITING_INTERVAL = 5UL`
- `POLLING_INTERVAL = 1000UL`

Current loop cadence:

1. Save `cycleStart = millis()`.
2. Call `requestAndReadPacket()`.
3. Render the LCD when the packet is OK, or render only the packet status code
   on row 3 column 19 when the packet read fails.
4. Wait until `millis() - cycleStart >= POLLING_INTERVAL` using `delay(100)`
   inside the wait loop.

## LCD layout baseline

The display is a 20x4 character LCD. Columns below are zero-based and inclusive.

| Row | Columns | Current content |
| --- | --- | --- |
| 0 | 0-7 | RPM, formatted with `"%4drpm "` |
| 0 | 8-11 | ignition advance, formatted from `"%3d\xDF "`; the trailing space at column 12 is overwritten by coolant |
| 0 | 12-15 | coolant temperature, formatted with `"%3dC"` |
| 0 | 16-19 | intake air temperature, formatted with `"%3dC"` |
| 1 | 0-7 | MAP, formatted with `"%4dkPa "` |
| 1 | 8-11 | idle valve, formatted with `"%3d "` |
| 1 | 12-15 | TPS, formatted with `"%3d "` |
| 1 | 16-19 | corrections, formatted with `"%3d%%"` |
| 2 | 0-7 | AFR, formatted as tenths with suffix `"afr "` |
| 2 | 8-13 | battery voltage, formatted as tenths with suffix `"V "` |
| 2 | 14-19 | engine status text from `engineStatus()` |
| 3 | 0-17 | fuel pressure, formatted as tenths with suffix `"bar            "` for the normal one-digit example |
| 3 | 19 | blank on OK packets, packet status code on errors |

The HD44780 degree marker is byte `0xDF`. In examples below it is written as
`<DF>` for readability; that token represents one LCD character cell.

## Reference payload A

Representative payload A is not a captured ECU packet. It is a sparse,
zero-filled `n2` fixed-list payload chosen to exercise the existing display
formatting.

Packet shape:

- command response header byte 0: `'n'` / `0x6E`
- command response header byte 1: `'2'` / `0x32`
- payload length byte: `123` / `0x7B`
- payload bytes: 123 bytes, default `0x00`, with the offsets below set

| Offset constant | Value | Expected decoded value |
| --- | ---: | --- |
| `RPM_LOW` | `0xB6` | combined RPM = 950 |
| `RPM_HIGH` | `0x03` | combined RPM = 950 |
| `ADVANCE` | `14` | 14 degrees |
| `COOLANT_WITH_OFFSET` | `122` | 82 C |
| `IAT_WITH_OFFSET` | `64` | 24 C |
| `MAP_LOW` | `45` | combined MAP = 45 kPa |
| `MAP_HIGH` | `0` | combined MAP = 45 kPa |
| `IDLE_VALVE` | `34` | 34 |
| `TPS` | `2` | 2 |
| `CORRECTIONS` | `100` | 100% |
| `O2` | `147` | 14.7 afr |
| `BATTERY10` | `125` | 12.5 V |
| `ENGINE_STATUS` | `0b00001001` | `W` and `R` active |
| `FUEL_PRESSURE` | `50` | 3.4 bar after current conversion |

Fuel pressure conversion for this fixture:

```text
(50 * 6895 + 5000) / 10000 = 34 tenths = 3.4 bar
```

## Reference frame A: OK packet after a cleared display

The following is the visible 20x4 LCD content expected from reference payload A
after a fresh display clear. `<DF>` is one character cell.

```text
 950rpm  14<DF> 82C 24C
  45kPa  34   2 100%
14.7afr 12.5V     WR
3.4bar              
```

As exact row strings:

```text
row 0: " 950rpm  14<DF> 82C 24C"
row 1: "  45kPa  34   2 100%"
row 2: "14.7afr 12.5V     WR"
row 3: "3.4bar              "
```

## Reference frame B: packet error after frame A

On a packet read error, current code does not rebuild the display. It writes only
the single status digit to row 3 column 19. For example, if status code `4`
occurs immediately after reference frame A, the visible content becomes:

```text
 950rpm  14<DF> 82C 24C
  45kPa  34   2 100%
14.7afr 12.5V     WR
3.4bar             4
```

This behavior should remain unchanged during the first behavior-preserving
refactor steps.
