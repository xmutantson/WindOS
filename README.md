# WindOS

Wind-tunnel control firmware for the Arduino Mega 2560 and Gale prototype.
`Main.ino` is the firmware entry point; `WindOSCore.h` contains shared timing,
feedback-validation, actuator-limit, and EEPROM-record logic. The historical
manual, experimental notes, and board fabrication files remain in this repository.

## Operator behavior

- Boot requests the configured 700 us ESC stop pulse before initializing sensors.
- Starting requires fresh, steady airflow feedback and a positive run duration.
  The seven-second arming phase requests the stop pulse. Release the initiating
  button press, then click to cancel arming.
- The running menu's **ABORT** action latches a stop. Missing, invalid, timed-out,
  or stale airflow feedback also stops the run. Feedback expires after 3,000 ms;
  unsuccessful/unsteady updates cannot increase throttle during that grace period.
- A stopped screen requires **release, then click** to return to the menu. A new
  **Start** action and run setup are required to rearm. Sensor recovery alone does
  not restart the motor. The main menu reset action follows this same stop flow.
- Trial duration starts after arming. Opening sensor or run menus does not pause
  acquisition or elapsed time. Between trials, the stop pulse is requested for
  ten seconds; clicking cancels the remaining sequence. Completion latches a stop.
- Running commands are bounded to 1200-1700 us normally. The existing debug mode
  permits up to 1900 us. These are inherited ESC settings, not universal values.

A software stop command does not establish that the propeller has stopped. The
manual's independent emergency action is removing propeller-battery power.
Before powered use, complete the checks in
[hardware validation](docs/HARDWARE_VALIDATION.md), including ESC response,
worst-case loop latency, sensor firmware compatibility, and power isolation.
This revision supersedes the manual's description of ABORT as a processor reset.

## Calibration and measurement validity

Load-cell reads check readiness and use one conversion per call. An unavailable
channel is not reported as measured zero. If no startup tare was possible, the
first ready conversion establishes zero and is not emitted as a measurement.

Calibration collects 16 finite conversions with a positive reference mass, with
a ten-second timeout. The existing hanging-mass polarity is retained. Factors
must be finite with absolute value between 1 and 1,000,000. This numeric check does
not establish physical accuracy. A replacement factor is applied only after
writing and verifying a versioned, checksummed EEPROM record. Failed verification
preserves any previously valid runtime factor and validity flag, then latches a
stop. EEPROM writes are not atomic: a power loss or partial write may invalidate
the persisted record; the next boot rejects it rather than assuming validity.

Records occupy bytes 32-43 and 48-59. Legacy raw factors at addresses 0 and 8 are
not trusted or migrated automatically. Recalibrate with a known reference after
upgrading. The display shows `CAL` for uncalibrated channels and `--` for an
unavailable calibrated reading.

Pressure readings retain fractions, clear failed readings, and reload the
selected BMP sensor's factory coefficients before each measurement because the
pinned driver shares coefficient storage across instances. Reads remain
sequential; a CSV row is not a simultaneous hardware sample.

## CSV interface

Default serial speed is 115200 baud; the existing serial menu can change it.
A run sequence emits one header and 20 fields per sample row. The first field
contains `Trial Number: N` only on the first row of each trial; later rows leave
it empty. Column 16 always contains the trial ID. Trial time begins at `0.000`
after arming, in seconds with three decimal places. Samples are scheduled every
1,000 ms; delayed iterations emit the current time, not fabricated catch-up rows.
The end-of-trial check stops before another sample, so no terminal row is promised.

| Columns (1-based) | Meaning |
| --- | --- |
| 1 | Trial annotation (first row only) |
| 2 | Elapsed seconds within this trial |
| 3-9 | Pressure channels 1-7, mbar (hPa), two decimal places |
| 10 | Airflow, m/s, two decimal places |
| 11 | Displayed target, m/s (legacy integer governor band plus 0.5) |
| 12 | Temperature, Celsius |
| 13-14 | Load cells 1-2, grams, two decimal places |
| 15 | Commanded motor percentage: `(pulse_us - 1200) / 7`; not measured power |
| 16 | Trial ID |
| 17 | Airflow status: `ok`, `held`, or `invalid` |
| 18 | Pressure validity mask, channels 1-7 in order (`1` valid), or `disabled` |
| 19-20 | Load statuses: `ok`, `uncalibrated`, `unavailable`, or `disabled` |

Invalid/disabled measurements are empty fields. `held` airflow is the last valid
sample still inside its freshness window, not a new measurement. Status columns
are an intentional schema extension; update parsers that assumed 15 columns.
Startup/menu diagnostic output may precede the CSV header. More decimal places
do not imply increased sensor accuracy.

## Offline tests

Requires Python 3.8+ and an existing C++11 compiler such as GCC. No downloads,
Arduino installation, serial connection, or hardware are used:

```text
python tests/run_tests.py
python tests/run_tests.py --syntax-only
python tests/run_tests.py --cxx /path/to/g++
```

The runner compiles the actual sketch with mock peripherals and Arduino-style
function declarations. Core checks cover bounds, calibration records, deadlines,
and rollover. The 21 isolated sketch cases cover boot, stop acknowledgement,
arming and running cancellation, airflow faults/status-read failures/timeouts,
trial sequencing, pressure channels, load readiness, calibration persistence,
CSV structure, and encoder direction. Failed checks print to stderr and exit
nonzero without assertion crash dialogs. Compiler runtime DLLs take precedence
over unrelated Python-distribution DLLs in child-process PATH.

Scratch builds use Python's temporary directory and are cleaned on exit. Set
`TEMP`, `TMP`, and `TMPDIR` to an owned scratch directory before running if disk
placement matters. Build artifacts and downloaded libraries under `.build/` are
ignored by Git.

## Reproducible Mega 2560 compile

Verified configuration: Arduino CLI **1.3.1**, `arduino:avr` **1.8.6**, board
`arduino:avr:mega` (ATmega2560), bundled AVR GCC **7.3.0-atmel3.6.1-arduino7**.
Library archive URLs and SHA-256 hashes are in
[the dependency lock file](tools/arduino-libraries.lock.json):

| Library | Version |
| --- | --- |
| ClimateGuard CG Anem | 1.0.3 |
| HX711 Arduino Library (bogde) | 0.7.5 |
| Adafruit Unified Sensor | 1.1.15 |
| Adafruit BMP085 Unified | 1.1.3 |
| LiquidCrystal | 1.0.7 |
| Servo | 1.2.2 |

Wire and EEPROM come from the pinned AVR core. CG Anem 1.0.3 uses `CG_Anem` and
status bit `STUP`; older releases expose incompatible APIs. The firmware performs
an additional explicit status read because the driver's normal status helper can
report a failed read as steady. The installed sensor revision/protocol must still
be checked against this library; legacy overvoltage/calibration flags are not
exposed by its current API.

With that toolchain already installed, stage source and libraries locally:

```text
python tools/prepare_sketch.py
python tools/prepare_libraries.py --download
arduino-cli compile --fqbn arduino:avr:mega --libraries .build/libraries --build-path .build/avr --warnings all .build/arduino/WindOS
```

`--download` explicitly permits missing library archives to be fetched from
Arduino. Omit it to verify and stage cached archives offline. Checksums are checked
before extraction. Neither helper installs a toolchain, modifies global libraries,
uploads firmware, or connects to a board. Set `ARDUINO_DIRECTORIES_DATA` to the
installed core/toolchain directory, `ARDUINO_DIRECTORIES_DOWNLOADS` and
`ARDUINO_DIRECTORIES_USER` to owned local directories, and
`ARDUINO_BUILD_CACHE_PATH` to a local cache when avoiding default system locations.

The compile on 2026-10-02 used 56,314/253,952 flash bytes and 3,494/8,192 static SRAM
bytes, leaving 4,698 bytes for stack/heap. These are linker figures, not measured
worst-case runtime memory use. `--warnings all` reports existing firmware warnings
(unused diagnostic variables, signed timing comparisons, and condition precedence)
and upstream library/core warnings. The build succeeds. No powered hardware test,
physical stop qualification, or measurement-accuracy certification was performed.
