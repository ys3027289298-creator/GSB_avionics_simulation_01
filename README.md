# Rocket Firmware Simulator

Test your rocket flight computer firmware against a realistic simulated flight — no hardware required, no launch required.

## What this is

If you're building a flight computer for high-power rocketry, your only real way to test apogee detection, pyro channel firing, and deployment sequence logic has traditionally been to launch a rocket and hope it works. That's expensive, weather-dependent, and you only get one shot per test.

This tool runs your firmware against a physically realistic simulated flight powered by [RocketPy](https://github.com/RocketPy-Team/RocketPy). Your firmware reads simulated sensor data (barometer, accelerometer, gyroscope, GPS) exactly as it would read real hardware, with realistic sensor noise based on actual datasheet specifications. At the end of the simulated flight, you get a pass/fail report comparing what your firmware decided against the ground truth physics.

```
=== TEST REPORT ===
Apogee detection:   PASS
  True apogee:      2937.0m at T+18.78s
  Detected apogee:  2935.4m at T+18.27s
  Time error:       0.51s

Pyro channel 1:
  PASS
  Fired at:       T+18.27s, alt=2935.4m
  Expected near:  T+18.78s
  Time error:     0.51s

Pyro channel 2:
  PASS
  Fired at:       T+24.11s, alt=298.3m
  Expected at:    alt=300.0m
  Distance error: 1.70m

=== SEQUENCE VALIDATION ===
PYRO1 single-fire check:    PASS — fired once
PYRO2 single-fire check:    PASS — fired once
PYRO1 -> PYRO2 order:       PASS (PYRO1 at T+18.27s, PYRO2 at T+24.11s)
PYRO1/PYRO2 separation:     PASS (gap=5.84s, min required=2.0s)
```

## Two modes

### Software-in-the-loop (SIL)
Your firmware compiles and runs natively on your laptop. No hardware needed at all. Fastest iteration cycle — rebuild and retest in seconds.

### Hardware-in-the-loop (HIL)
Your firmware runs on your actual flight computer hardware, but receives simulated sensor data over USB serial instead of reading real sensors. Tests the actual binary that will fly, on the actual chip it will run on, with real hardware timing and interrupts.

```
[RocketPy sim server on laptop]
        ↕ USB Serial
[Your actual flight computer]
        ↑ HAL intercepts sensor reads
        ↑ feeds simulated data instead of real sensors
```

## How it works

Your firmware is written against a small hardware abstraction layer (HAL) instead of communicating with sensor chips directly. On a real flight computer, the HAL talks to actual hardware. In simulation, a different implementation of the same HAL talks to this simulator instead.

```
Your firmware
        ↕  HAL_ReadPressure(), HAL_ReadAccelX(), etc.
Simulator (Python + RocketPy)
        ↕
Realistic 6-DOF flight physics + datasheet-accurate sensor noise
```

The simulator streams sensor data to your firmware over TCP (SIL) or USB serial (HIL), listens for events your firmware reports back (`APOGEE`, `PYRO1`, `PYRO2`), and compares them against RocketPy's known-correct physics at the end of the flight.

The server accepts whichever connection arrives first — TCP for a native SIL build, serial for real hardware. First come, first served.

## Requirements

- Python 3.9+
- For SIL mode: [PlatformIO](https://platformio.org/) installed (CLI or VS Code extension) and a C++ compiler:
  - **Windows**: [MSYS2](https://www.msys2.org/) with `mingw-w64-x86_64-gcc` installed, added to your PATH
  - **macOS**: Xcode Command Line Tools (`xcode-select --install`)
  - **Linux**: `gcc`/`g++` (usually already installed)
- For HIL mode: any microcontroller with a UART (Arduino, ESP32, STM32, RP2040, etc.)

## Setup

```bash
pip install -r requirements.txt
```

### SIL setup

Add the native build environment to your `platformio.ini`:

```ini
[env:native]
platform = native
build_src_filter = +<main.cpp> +<hal_sim.cpp> +<native_main.cpp>
build_flags = -lws2_32 -mconsole
```

> `-lws2_32` and `-mconsole` are required on Windows. Omit both on macOS/Linux.

Make sure your firmware reads sensors through the HAL functions defined in `Hal.h` rather than talking to hardware registers directly.

### HIL setup

1. Copy `hal_sim.cpp` and `Hal.h` into your firmware project
2. Add a platform serial implementation — two functions specific to your hardware:

```cpp
int hal_serial_write(const char* buf, int len);
int hal_serial_read(char* buf, int len);
```

**Arduino / ESP32:**

```cpp
#include <Arduino.h>

int hal_serial_write(const char* buf, int len) {
    return Serial.write((const uint8_t*)buf, len);
}

int hal_serial_read(char* buf, int len) {
    int i = 0;
    while (i < len) {
        while (!Serial.available());
        buf[i++] = Serial.read();
        if (buf[i-1] == '\n') break;
    }
    return i;
}
```

**STM32 (HAL):**

```cpp
int hal_serial_write(const char* buf, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, HAL_MAX_DELAY);
    return len;
}

int hal_serial_read(char* buf, int len) {
    int i = 0;
    uint8_t c;
    while (i < len) {
        HAL_UART_Receive(&huart2, &c, 1, HAL_MAX_DELAY);
        buf[i++] = c;
        if (c == '\n') break;
    }
    return i;
}
```

3. In your startup code, initialise serial at 115200 baud and call `HAL_Init()`
4. Call `HAL_Update()` wherever your firmware currently reads sensors
5. Flash normally — your build system is untouched

## Usage

```bash
python server.py --config config.json
```

The server will:
1. Set up the RocketPy flight from your config
2. Ask which serial port to listen on (or skip for TCP only)
3. Wait for a connection — TCP from a native SIL build, or serial from real hardware
4. Stream live sensor data into your firmware
5. Print a final pass/fail test report and sequence validation

### Real-time vs fast mode

By default the simulator runs in real-time at 100Hz, mirroring actual flight duration. Set `REALTIME = False` in `server.py` to run as fast as the firmware can consume data — useful for rapid iteration. The raw generation rate benchmarks at ~38kHz so even high-frequency avionics won't bottleneck the sim.

## Configuring your rocket

All rocket, motor, flight, noise, and validation parameters live in a single JSON config file. No Python editing required.

```json
{
    "environment": {
        "latitude": 51.5,
        "longitude": -3.18,
        "elevation": 100
    },
    "motor": {
        "thrust_source": [[0,0],[0.1,800],[3.5,200],[3.6,0]],
        "dry_mass": 0.5,
        "burn_time": 3.6,
        "..."
    },
    "rocket": {
        "radius": 0.05,
        "mass": 2.0,
        "nose": { "length": 0.3, "kind": "ogive", "position": 1.2 },
        "fins": { "n": 4, "root_chord": 0.12, "tip_chord": 0.06, "span": 0.08, "position": 0.1 },
        "parachute": { "name": "main", "cd_s": 10.0, "trigger": 300 }
    },
    "sensor_noise": {
        "pressure_std": 2.4,
        "baro_std": 0.3,
        "accel_std_g": 0.05,
        "gyro_std_deg": 0.0038,
        "gps_std_meters": 1.5
    },
    "sim": {
        "dt": 0.01,
        "socket_host": "localhost",
        "socket_port": 9000
    },
    "validation": {
        "apogee_time_tolerance_s": 1.0,
        "pyro1_time_tolerance_s": 1.0,
        "pyro2_alt_tolerance_m": 10.0,
        "min_pyro_separation_s": 2.0,
        "pyro1": true,
        "pyro2": true
    },
    "telemetry": {
        "enabled": true,
        "server": "https://your-telemetry-server.com/session"
    }
}
```

See `config.json` in the repo for a complete working example.

## Single deploy vs dual deploy

**Single deploy** — set `pyro2: false` in the validation config. The tool tests apogee detection and pyro channel 1 only.

**Dual deploy** — set `pyro2: true`. The tool tests:
- Apogee detection (pyro 1 fires at apogee — drogue)
- Pyro 2 fires at the configured trigger altitude (main)
- Full sequence validation (correct order, no duplicate fires, minimum separation between channels)

The sequence validator catches three real bug classes that can destroy a rocket:
- **Duplicate pyro fires** — a channel firing more than once
- **Wrong deployment order** — main deploying before drogue
- **Insufficient channel separation** — pyro channels firing too close together

## Telemetry

By default the tool sends anonymous session statistics to a telemetry server after each run — mode (SIL or HIL), duration, packets sent, and pass/fail results per channel. No personally identifiable information is collected. No file paths, IP addresses, or usernames are ever sent.

To opt out, set `"enabled": false` in the telemetry section of your config:

```json
"telemetry": {
    "enabled": false
}
```

## Sensor noise model

Sensor noise is modelled using real datasheet specifications so bugs that only appear under realistic noise aren't hidden by a clean simulation:

| Sensor | Spec | Std dev at 100Hz |
|---|---|---|
| Barometer (MS5607) | 0.024 mbar RMS @ OSR4096 | 2.4 Pa |
| Accelerometer (ADXL375) | 5 mg/√Hz | ~0.49 m/s² |
| Gyroscope (ICM-45686) | 3.8 mdps/√Hz | ~0.00066 rad/s |
| GPS | ~2.5m CEP | ~1.5m |

## Validated against real flight data

This tool has been tested against real flight logs from [Altimeter Cloud](https://www.altimetercloud.com). On clean flight data, apogee detection came back accurate to within 1m of the recorded true apogee — consistent with synthetic simulation results. A second flight with a barometer spike correctly produced a larger detection error, demonstrating the tool faithfully reflects real sensor anomalies rather than hiding them.

## What this is not

This is **not** a replacement for [OpenRocket](https://openrocket.info/) or RocketPy — those answer "will this rocket fly well?" This tool answers a different question: "does my flight computer's *code* make the right decisions during a flight?" The two are complementary.

This also doesn't replace test flights. It catches logic bugs early so your test flights are spent on the genuinely hard edge cases — pressure artifacts at transonic speeds, vibration resonance from your electronics sled — rather than basic state machine errors.

## Status

- ✅ Apogee detection testing
- ✅ Dual pyro channel testing (drogue + main)
- ✅ Deployment sequence validation (order, duplicates, separation)
- ✅ Datasheet-accurate sensor noise per sensor
- ✅ JSON config file — no Python editing required
- ✅ Validated against real Altimeter Cloud flight logs
- ✅ HIL mode — real hardware over USB serial
- ✅ Generic UART HAL — works on Arduino, ESP32, STM32, RP2040, anything with a UART
- ✅ Anonymous usage telemetry with opt-out
- ⬜ Staging / multi-stage logic
- ⬜ CI / GitHub Actions integration
- ⬜ CMake / non-PlatformIO build system support

## Why this exists

Most amateur and university rocketry teams test flight computer firmware by launching a rocket and hoping it works, or with ad-hoc bench tests — teams have independently built hardware vacuum chambers for barometer testing and spin rigs for accelerometer testing. There's no software equivalent of [ArduPilot's SITL](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html) for high-power rocketry. This is an attempt to build that.

## Contributing / trying this on your own firmware

If you're part of a university rocketry team or building your own flight computer and want to try this, [open an issue](../../issues) or reach out — happy to help wire it up to your codebase.

## License

MIT
