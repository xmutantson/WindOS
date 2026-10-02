# Hardware validation and reproducibility

The automated host tests exercise firmware logic using simulated hardware. They do
not establish that an ESC stops, a physical sensor is accurate, or a PCB is ready
to manufacture. This document records what must be verified before accepting a
firmware build for the Gale prototype or adapting it to another tunnel.

## Existing hardware evidence

- `Wind Tunnel Manual V1.2.pdf` describes the Arduino Mega 2560 installation and
  operation. Its Appendix B describes ABORT as a reset. That description is
  superseded by the firmware's latched stop behavior; use the current README for
  the changed interface and CSV format.
- `Wind Tunnel Motherboard.kicad_pcb` is a KiCad board file. Its current text has
  no footprint or pad declarations and only net 0. A connectivity-aware schematic
  and BOM are not included. This limits electrical review; it does not by itself
  prove that the manufactured prototype is defective.
- `WindOS Rev 2.zip` contains seven Gerber layers and two drill files. It is a PCB
  fabrication archive, not a second firmware release. Its correspondence to a
  manufactured board has not been checked.
- The annotated ODT and manual Appendix D provide historical experimental
  results. The manual acknowledges sparse measurements and uncertain angle of
  attack. These are examples, not a stated measurement-accuracy specification.

## Decisions requiring hardware evidence

| Item | Existing configuration or behavior | Evidence needed |
| --- | --- | --- |
| ESC pulses | 700 us stop/arm, 1200 us running idle, 1700 us normal upper limit; the existing debug setting permits 1900 us | Confirm the actual ESC model, calibration, accepted pulse range, lost-signal behavior, and response to every stop transition. A requested pulse is not proof of zero rotation. |
| Airflow validity | Firmware checks status, numeric range, and age of successful reads | Confirm the installed CG-Anem hardware/firmware revision, warm-up behavior, valid operating range, normal read latency, and an appropriate timeout. A sensor returning a plausible but stuck value cannot be detected from read age alone. |
| Timing | Arming, running, and inter-trial waits are separate phases | Measure actual loop latency with the intended sensors and serial baud rate. Slow serial output and blocking device drivers can delay software handling. |
| Load cells | Calibration factors are validated; unavailable values must not be represented as measured zero | Establish scale polarity, reference masses, tare procedure, repeatability, and drift for the installed cells. Do not infer valid calibration from a finite factor alone. |
| Pressure | Fractional sensor values and validity are retained | Verify sensor/channel mapping, offsets, precision, and pressure-tap plumbing. More output decimal places do not increase physical sensor accuracy. |
| USB/power behavior | The manual reports backfeeding and display corruption under some connection sequences | Review the actual wiring and power arrangement. Firmware changes do not resolve electrical backfeeding. |

The pinned Adafruit BMP085 Unified 1.1.3 driver keeps factory compensation
coefficients in file-static storage shared by every sensor instance. The firmware
must reload the selected channel's coefficients before each measurement. Its
`getEvent()` success value alone is not a transport-health check: the driver
returns true unconditionally. Channel selection, device initialization, timeout
flags where available, and numeric checks improve detection, but cannot prove a
plausible returned value is physically correct. See the
[pinned driver source](https://github.com/adafruit/Adafruit_BMP085_Unified/blob/1.1.3/Adafruit_BMP085_U.cpp).

The software stop is not a substitute for independently removing motor power.
The existing manual identifies disconnecting the propeller battery as the
emergency shut-off. A hardware interlock, watchdog strategy, and power-isolation
design require an electrical/mechanical review; they are not supplied by this
patch.

## Acceptance sequence

1. Run the offline tests and compile with the documented, pinned toolchain and
   libraries. Record flash and SRAM usage for the Mega 2560. Host compilation
   alone cannot establish AVR memory fit or library compatibility.
2. With motor power disconnected, inspect the controller signal using appropriate
   instrumentation. Exercise boot, start, arming cancellation, running abort,
   missing/invalid airflow feedback, trial completion, and inter-trial
   cancellation. Record the requested and observed pulses and recovery sequence.
   No transition out of a stopped state may resume a run without the documented
   operator action.
3. In a controlled hardware test, verify ESC stop behavior and operator access to
   independent power removal. Record worst-case stop latency at the supported
   serial settings. Software tests do not qualify this physical behavior.
4. Check sensor faults and channel identity independently. Verify that absent or
   failed pressure/load channels have invalid or empty data fields, and that
   airflow faults cannot produce an increased motor command.
5. Run repeated trials with menu navigation, cancellation, and restarted runs.
   Confirm timestamps, trial IDs, duration, and header/row column counts in the
   captured data. Verify that arming time is excluded from measurement time.
6. Check calibration with reference masses and compare pressure/airflow readings
   against suitable references. Publish repeatability and uncertainty before
   making an accuracy claim.

Record firmware commit, toolchain/library versions, hardware revisions, ESC model
and settings, calibration identity, test conditions, observed results, and any
exceptions for each acceptance run. Do not mark an unperformed check as passed.

## Reproducible hardware package still needed

- Schematic with named nets and a board generated from that schematic.
- BOM with manufacturer part numbers, ratings, and approved alternatives.
- Connector pinouts and a sensor-to-multiplexer channel table.
- Assembly, inspection, and power-distribution instructions, including the
  documented USB power concern.
- KiCad project/library versions, design-rule settings, DRC results, and a
  fabrication archive tied to a source revision.
- Machine-readable experimental data and analysis steps, including specimen
  geometry, angle measurement, reference conditions, and uncertainty.

These deliverables require the actual hardware design and measurements. The
existing Gerber files should not be reverse-engineered into an assumed validated
schematic or BOM.

## Offline verification recorded 2026-10-02

The complete sketch passed 21 isolated host behavior cases plus the core regression
executable using MinGW GCC 13.2.0. The mocked syntax check and Mega 2560 compile
also passed. The AVR configuration and exact dependency archive checksums are
recorded in README.md and tools/arduino-libraries.lock.json. Arduino CLI 1.3.1,
AVR core 1.8.6, and the pinned libraries produced 56,314 bytes of flash and 3,494
bytes of static SRAM (4,698 bytes remain for stack/heap). Compilation reports
firmware and upstream warnings; it is not a zero-warning build.

CG Anem 1.0.3 exposes CG_Anem and the STUP status bit, unlike the mixed historical
API used by the original sketch. The firmware explicitly checks status-read
success and Wire timeout flags before refreshing airflow feedback. A plausible
but stuck reading, electrical failure that still returns plausible data, or an
incompatible sensor firmware protocol still requires hardware investigation.
The earlier library's overvoltage/taring public flags are not present in this API.
Confirm the installed sensor revision before accepting this pinned combination.

All verification above was compile-only or simulated. No board upload, live motor,
sensor, power, radio, or physical safety test was performed. Acceptance steps 2-6
remain unverified. Static memory fit does not establish runtime stack/heap margin.
