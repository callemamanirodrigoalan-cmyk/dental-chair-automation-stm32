# Dental Chair Automation — Cup Filler & Spittoon Controller (STM32F103)

Non-blocking firmware for automating two independent functions of a dental chair: an
automatic **cup filler** (llenavasos) and an automatic **spittoon** (escupidera). Built on an
STM32F103C8T6 ("Blue Pill") with two time-of-flight distance sensors on **two separate I²C
buses**, a 2-digit 7-segment configuration UI, and a defensive state machine designed to keep
the solenoid valves in a safe state under sensor faults and power glitches.

> Real-world medical-equipment automation: the design priorities were **reliability and fail-safe
> behavior**, not just "making it work once on the bench".

---

## What it does

- Detects a cup placed under the faucet and opens a solenoid valve to fill it, closing it
  automatically when the cup is removed or the target level is reached.
- Detects the patient's presence at the spittoon and triggers rinse water with a configurable
  activation delay.
- Each subsystem can be enabled independently (cup only, spittoon only, or both), selected from
  an on-device menu — no recompilation needed.
- All parameters (ranges, hysteresis, delays, installed sensors) are stored in EEPROM and
  survive power loss.

---

## Engineering highlights

These are the parts worth looking at in `src/`:

| Area | What was done | Why it matters |
|---|---|---|
| **Dual I²C** | VL6180X on I²C1 (PB6/PB7), VL53L0X on I²C2 (PB10/PB11) | Both sensors ship at address `0x29`; putting each on its own bus avoids address conflicts **without an I²C multiplexer or address remapping** |
| **Non-blocking reads** | Sensors run in continuous mode; the loop polls a "sample ready" flag instead of waiting | Zero `delay()` in the main loop — the display ISR and valve logic never stall waiting for a measurement |
| **Two-stage filtering** | Median-of-3 followed by an EMA (exponential moving average) | Rejects single-sample spikes *and* smooths jitter, with a tunable `EMA_ALPHA` for responsiveness |
| **Defensive state machine** | 6-state valve model (IDLE → OPENING → OPEN → CLOSING → CLOSED → FAULT) with valve-feedback confirmation | Every open/close is **confirmed via a feedback line**, with timeouts that escalate to a FAULT state instead of silently leaving a valve open |
| **Frozen-sensor recovery** | On a stale sensor: XSHUT hardware reset + I²C bus recovery + re-init, all **without resetting the MCU** | A hung sensor self-heals in runtime; after repeated failures the channel is disabled so it can't degrade the other sensor |
| **I²C bus recovery** | 9 manual SCL clock pulses to free a slave holding SDA low, then a STOP condition (standard NXP technique) | Recovers from a partial/aborted transaction that would otherwise lock the bus |
| **Watchdog + safe boot** | IWDG (~4 s timeout) armed before touching sensors; valves force-closed at startup if found open | The system fails *closed* and self-restarts rather than hanging in an unsafe state |
| **Cold-start robustness** | Sensor init retried for several hundred ms while feeding the watchdog | Handles the slow power-rail/boot settling seen after the unit has been unpowered for hours |

---

## Hardware

- **MCU:** STM32F103C8T6 (STM32duino / libmaple core)
- **Cup sensor:** VL6180X ToF (I²C1, range ~0–80 mm)
- **Spittoon sensor:** VL53L0X ToF (I²C2, range ~0–200 mm)
- **Outputs:** 2× solenoid valve drivers, each with a feedback input for open/close confirmation
- **UI:** 2-digit 7-segment display (multiplexed via a timer ISR) + 2 push-buttons
- **Sensor reset:** independent XSHUT line per sensor for clean power-on state
- **Custom PCB:** designed in EasyEDA / KiCad, fabricated in **both through-hole (DIP) and SMD**
  versions (see `/hardware`)

---

## On-device configuration

A two-button menu on the 7-segment display lets the installer set, without a computer:

- Installed sensors (both / cup only / spittoon only)
- Cup fill range (lower/upper, in 0.1 cm steps)
- Cup hysteresis and activation delay
- Spittoon range (lower/upper) and activation delay
- Save / discard, with on-screen confirmation

Values are range-checked on load; a corrupted or first-boot EEPROM falls back to safe defaults.

---

## Repository layout

```
/src        Firmware (.ino / .cpp / .h)
/hardware   PCB design files and board images (DIP + SMD)
/docs       Wiring, pin map, configuration menu guide
```

## Pin map (summary)

| Function | Pin |
|---|---|
| Cup valve out / feedback | PB1 / PA8 |
| Spittoon valve out / feedback | PB13 / PB12 |
| Cup sensor XSHUT (VL6180X) | PB8 |
| Spittoon sensor XSHUT (VL53L0X) | PB9 |
| I²C1 (cup) SCL / SDA | PB6 / PB7 |
| I²C2 (spittoon) SCL / SDA | PB10 / PB11 |
| 7-seg digits / segments | PA0, PC15 / PA1–PA6, PC13, PC14, PA3 (DP) |
| Buttons | PA7, PB0 |

---

## Skills demonstrated

Embedded C/C++ · STM32 (STM32duino/libmaple) · I²C (multi-bus, bus recovery) · ToF sensors
(VL6180X, VL53L0X) · non-blocking/event-driven firmware · digital filtering (median + EMA) ·
finite state machines · watchdog & fail-safe design · EEPROM persistence · multiplexed display
driving via timer ISR · custom PCB design (EasyEDA/KiCad, DIP & SMD).

---

*This firmware was developed for a real dental chair retrofit. Hardware photos and a short demo
video of the system in operation will be added.*
