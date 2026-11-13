# Design

## Purpose and boundaries

This repository builds ESPHome firmware for an ESP32 that exchanges data with
an LG Therma V heat pump and, in the standard profiles, a Modbus power meter.
It exposes telemetry and operating controls through the ESPHome/Home Assistant
API and runs a local heating-control loop.

The ESP32 firmware does not implement the heat pump's internal compressor or
refrigerant controls. It reads the heat pump's Modbus interface and changes
its exposed registers/coils. Home Assistant provides selected room inputs and,
for profiles that configure it, an optional target-power input. Networking,
OTA, sensor publishing, and hardware drivers are supplied by ESPHome.

This is a custom automation, not a certified safety controller. It relies on
the configured register map, sensor quality, and installation-specific
settings. Verify behavior and safety limits on the actual hardware before
enabling automatic control.

## Repository map

| Path | Responsibility |
| --- | --- |
| `LGtherma_daniel.yaml` | Daniel profile: pin substitutions, shared packages, API, Home Assistant inputs, and 1-Wire probes. |
| `LGtherma_daniel_newpcb.yaml` | Daniel profile variant with new PCB pin assignments and its own API configuration. |
| `LGtherma_marco.yaml` | Marco profile: distinct tuning, room-temperature entity, and water-temperature adapters. |
| `packages/generic.yaml` | Shared ESP32/ESPHome setup, custom C++ includes, thermostat, weather compensation, entities, derived sensors, and scheduler. |
| `packages/LGheatpump.yaml` | LG Modbus UART/controller, register-backed sensors and controls, and calculated heat-pump metrics. |
| `packages/Powermeter.yaml` | Power-meter Modbus UART/controller and electrical/energy sensors. |
| `packages/Powermeter_dummy.yaml` | Template sensor substitute that reports zero active power. |
| `packages/state_machine.c` | Heating/cooling operation state machine and minimum-power avoidance logic, included by `generic.yaml`. |
| `customize/custom.h` | `Customize` interface and shared pointer. |
| `customize/custom_daniel.h` | Daniel tuning and idle-pump policy. |
| `customize/custom_marco.h` | Marco tuning and no-op idle policy. |
| `customize/lookup.h` | Clamped, piecewise-linear lookup helper used for refrigerant estimates. |
| `tools/steady_windows.py` | Offline analysis of the Home Assistant recorder: steady running windows, floor eps/UA and sensor offsets, for calibrating the power target parameters. |
| `CLAUDE.md` | Working guide for coding agents: conventions, entity map, validation and deployment. |

## Configuration composition

Each device YAML defines substitutions and composes three packages:

1. `packages/Powermeter.yaml` for the power meter.
2. `packages/LGheatpump.yaml` for the heat pump.
3. `packages/generic.yaml` for the shared controller and ESPHome runtime.

The `customize` substitution selects `customize/custom_${customize}.h`. The
selected header implements the same `Customize` fields and idle callback;
the boot hook in `generic.yaml` constructs the selected implementation.
Device profiles also supply GPIO substitutions and any sensors that differ by
installation. `packages/Powermeter_dummy.yaml` is not selected by the listed
profiles; it is an alternative package for a configuration intentionally
without a physical meter.

The custom profile parameters are consumed by the control loop:

| Parameter | Daniel | Marco | Meaning |
| --- | ---: | ---: | --- |
| `water_hyst_pos` | 2.0 °C | 4.0 °C | Positive water-temperature hysteresis used when starting/limiting operation. |
| `water_hyst_neg` | -4.0 °C | -4.0 °C | Negative hysteresis used to lower targets during operation. |
| `max_stooklijn_correction_pos` | 4.0 °C | 3.0 °C | Upper bound on room-error correction to the weather-curve target. |
| `max_stooklijn_correction_neg` | -10.0 °C | -5.0 °C | Lower bound on room-error correction to the weather-curve target. |
| `base_stooklijn_flow` | 17.5 L/min | 30.0 L/min | Reference flow used to normalize the weather-curve adjustment. |

When idle, the Daniel callback turns heat enable off if the floor sensor is
below 23 °C or room air is warmer than the floor; otherwise it turns heat
enable on to circulate water. The Marco callback is a no-op.

The common runtime targets an ESP32 `nodemcu-32s` board using ESP-IDF. It
configures Wi-Fi, a fallback access point, captive portal, logging, OTA, and
the ESPHome API. Most Wi-Fi and OTA credentials are loaded through ESPHome
secrets. The API encryption setting differs by profile, so review that profile
before deployment.

### Device-profile differences

| Profile | Device ID / tuning | Hardware and input differences |
| --- | --- | --- |
| Daniel | `lgtherma`; `customize: daniel`; thermostat starts at 19.0 °C with 0.1 °C steps; max water target 60 °C. | Heat-pump UART GPIO23/22; power-meter UART GPIO19/18; 1-Wire on GPIO26; Home Assistant room air/floor and target-power entities; two DS18B20 readings. |
| Daniel new PCB | `lgtherma_test`; `customize: daniel`; thermostat starts at 19.0 °C with 0.1 °C steps; max water target 60 °C. | Heat-pump UART GPIO18/19; power-meter UART GPIO22/23; 1-Wire on GPIO14; Home Assistant room air/floor and target-power entities; two DS18B20 readings. API encryption is commented out and the API password is empty in this profile. |
| Marco | `esp-lgthermav`; `customize: marco`; thermostat starts at 20.5 °C with 0.5 °C steps; max water target 40 °C. | Heat-pump UART GPIO19/18; power-meter UART GPIO23/22; room temperature uses `sensor.th_woonkamer_temperature`; floor temperature is a fixed template value; water inlet/outlet values are adapted from the LG Modbus readings; target-power input is fixed at 0 W. |

Treat these values as installation-specific examples. Confirm GPIO mapping,
sensor addresses/entities, and device identity for the hardware being flashed.
All three profiles define the `wp_target_power` sensor ID required by the
shared controller. Daniel and new-PCB read the Home Assistant
`input_number.wp_target_power` entity and multiply its kW value by 1000 to
produce watts. Marco publishes a fixed zero-watt template value, disabling
target-power control for that profile.

## Hardware and protocol interfaces

Both serial links use 9600 baud, 8 data bits, one stop bit, and an ESPHome
Modbus controller that polls at five-second intervals with a 200 ms command
throttle.

### LG Therma V

- Modbus device address: `0x2`.
- UART parity: none.
- Holding registers expose operating mode, control method, target water
  temperature, energy state, and several registers currently labelled
  unknown.
- Coils control heat/cool enable, DHW enable, and silent mode.
- Discrete inputs report pump/compressor activity, defrost, DHW, silent mode,
  and error status.
- Read registers expose error/mode status, return and outlet water
  temperatures, internal and outdoor temperatures, flow, pressure, compressor
  speed, and several readings still marked unknown.
- Temperature values are generally scaled by 0.1; pressure readings are
  scaled by 0.01. The water target register uses a factor of 10.

Register labels and scaling are implementation assumptions. In particular,
keep undocumented readings marked as unknown until verified against the
specific heat-pump model and interface documentation.

### Power meter

- Modbus device address: `0x1`.
- UART parity: even.
- Holding registers are decoded as FP32 for per-phase and total voltage,
  frequency, current, active power, and consumed energy.
- Power registers are multiplied by 1000 before publication as watts.
- The energy sensor is published in kWh and ignores a reading lower than its
  previous value, logging the rejected value.

The dummy package publishes zero for total active power; it does not emulate
real electrical measurements.

## Runtime data flow

```text
LG heat-pump Modbus registers/coils ─┐
                                     ├─> sensors and controls ─> controller
Power-meter Modbus registers ────────┘                            │
                                                                  ├─> LG target/enable writes
Home Assistant room/target inputs ────────────────────────────────┤
1-Wire temperatures (display, cooling) ──────────────────────────┤
                                                                  └─> ESPHome API entities
```

The shared package supplies most computed control and telemetry entities;
device YAMLs provide physical and Home Assistant input adapters. The room-air
template uses the Home Assistant room-air reading while the API is connected,
and falls back to the heat pump's internal temperature if the API is
disconnected.

## Control behavior

### Thermostat and weather compensation

The adjustable thermostat compares the room-air input to the configured room
setpoint. It has a 0.5 °C off/on hysteresis while heat is requested and uses a
15-minute timer before changing the local heat-request switch. When the
Home-Assistant target-power input is above 0.05, the thermostat request is
enabled; when the target returns to zero, normal room-temperature control
resumes. A weather-curve target above 26 °C also suppresses a thermostat
stop request in the implemented logic.

The weather curve (`stooklijn`) is a linear interpolation between:

- minimum outdoor temperature and maximum water temperature; and
- curve-start outdoor temperature and minimum water temperature.

Outdoor temperature is clamped to this range. A configurable offset is added,
then the result is clamped between minimum and maximum water temperatures. A
change in outdoor temperature triggers recalculation. The profile's maximum
water temperature substitution caps the UI control range.

While running in heating mode, the state machine further adjusts the curve
target using return-water temperature, measured flow, and a profile-specific
baseline flow. A bounded correction based on room-temperature error and the
thermostat error gain is added before the final target is sent to the heat
pump.

### Main operating state machine

`packages/state_machine.c` runs every 20 seconds. On startup it waits for
approximately 60 seconds while inferring an initial state from operation
mode, compressor status, and water target. Its primary states are:

```text
Idle -> Starting -> EarlyRun -> Running -> Stopping -> Afterrun -> Idle
                         ^          |
                         └─ Defrost ┘

Cooling -> CoolingPowerCtl -> Cooling
```

- **Idle:** checks for a thermostat heat request; otherwise applies the
  profile-specific idle behavior.
- **Starting:** enables heat and raises the water target until the compressor
  starts or the heat request is removed.
- **EarlyRun:** applies a lower target and enables silent mode during a
  startup period of up to 15 minutes.
- **Running:** adjusts the target, observes defrost, may run minimum-power
  avoidance, and changes silent mode based on measured power and filtered
  temperatures.
- **Defrosting:** uses running control while recognizing a defrost interval;
  returns to running after the configured eight-minute interval.
- **Stopping:** disables heat and sets the water target to 20 °C; waits for
  the compressor to stop.
- **Afterrun:** re-enables heat to circulate water for the configured
  minimum-off interval, then returns to idle.
- **Cooling / CoolingPowerCtl:** cooling is manual unless a target-power
  request is present. Target-power cooling adjusts the water target using
  measured electrical power and water-side temperature difference.

The state machine changes the LG target only when the requested value differs
from the current setting by more than 0.7 °C; the Modbus number's configured
step is 0.1 °C.

### Power target control

When a power target is set, the controller works from the LG Modbus outlet and
return temperatures, the LG flow rate and the power meter only. It does not use
the 1-Wire probes or Home Assistant sensors. `Target Power effective` follows
`wp_target_power`, but drops to 0 W when the Home Assistant API has been
disconnected for 10 minutes; the normal thermostat then takes over.

**Plant model.** The floor loop is treated as a heat exchanger between the
outlet water and the floor, fed by a pump in ΔT control:

```text
Q    = eps · m·cp · (To − Tfloor)       eps = 1 − exp(−UA / m·cp)
m·cp = clamp(Q / ΔTpump, m_min·cp, m_max·cp)
P    = Pbase + Q · (a + b · (To − Toutdoor)) + Psilent · silent
```

The pump holds the water ΔT at `Power target pump dT` (5.3 K including the
0.39 K offset) by changing flow between `Power target minimum flow` (17 L/min)
and `Power target maximum flow` (35 L/min). Below about 1.25 kW the flow sits at
the minimum and ΔT varies; above it extra heat comes mostly from extra flow and
the outlet rises slowly. The electrical model has a fixed base load and a
compressor COP that falls with the lift between outlet and outdoor temperature;
silent mode saves about 230 W at the same heat. Its constants are per
installation (`cop_*` in `customize/custom_<profile>.h`), fitted on 313 steady
20-minute windows of Oct – Nov 2026 (rms 102 W).

The setpoint for target power `Pt` takes the floor temperature from the current
operating point (`Estimated floor temperature` = To − dT / eps, or the last valid
value), then iterates: heat from the electrical model at the current outlet
guess, flow from the pump rule, outlet from the heat exchanger. Silent mode is
assumed on for targets below 1 kW.

Backtest on pairs of steady windows at most 3 hours apart with at least 300 W
between them (predict the second window's outlet from the first window's
operating point and the second's power): mean absolute error 0.56 K, against
1.75 K for the previous formula `To + (Pt/P − 1) · dT / eps` (above 1.5 kW:
0.70 against 2.85 K). With the COP model fitted on October only, November pairs
gave 0.51 against 1.31 K. The previous formula kept COP and flow at their
current values, so from a low-power point (high COP, minimum flow) it
overestimated every step: on 8 Nov it asked 34 °C for 2 kW at 10:45 and then
33 °C for 2.75 kW at 11:00, where about 30 and 31 were right.

**Sensors.** The LG return sensor reads about 0.39 K above the outlet sensor
during pump-only circulation, so `Water dT offset` (default 0.39 K) is added to
the LG dT, which is smoothed over 3 minutes. The 1-Wire probes were not used:
their error grows with falling outdoor temperature (up to about 1.5 K on dT)
and they produced `eps ≥ 1` in about a quarter of steady periods.

**Setpoint.** The setpoint is calculated once when the target changes by
100 W or more, and when power control starts in `Running` with a target
already set. It is then held. When the LG has been in ES2 for 15 minutes and
the outlet has moved less than 0.5 K over the last 15 minutes, the setpoint is
re-trimmed toward the calculated value by at most 1 K. The setpoint is rounded
to whole degrees and clamped to 20 °C – `Power target maximum
watertemperatuur`.

**Outlet lead.** The calculated setpoint is where the outlet settles once the
floor has warmed up, which can be well above the outlet early in a run. The LG
accumulates setpoint − outlet: a setpoint the outlet does not reach builds up a
shortfall that it lets out as a runaway (9 Nov 12:26 at 0.7 K, 10 Nov 04:46 at
5–6 K, below), and a setpoint at the outlet lets the compressor sink to its
minimum (9 Nov 23:39). The written setpoint is therefore at most the outlet
plus a lead that depends on the energy state, rounded down: 1.5 K in ES5,
0.5 K in ES2, 0 in ES7. It follows the outlet up until it reaches the
calculated setpoint. The boost is exempt, and during defrost the last value is
held. Above about 1.25 kW, 1 K is about 350–400 W, more than the
deadband, so the energy state bridges the remaining gap.

**Boost.** The model setpoint is where the LG should settle, but the LG only
speeds the compressor up when its own speed estimate for the setpoint is well
above the current speed (see "The LG's own compressor model"). On a step up of
more than the deadband the controller therefore writes a boost setpoint of
`Toutdoor + (compressor + boost margin − 15.6) / 1.54`, at least outlet + 1 K,
when that is above the model setpoint. `Power target boost margin` defaults to
12 compressor steps (0 turns the boost off). The boost is fixed at the start of
the step and ends when the meter shows the target minus half the deadband, or
after 30 minutes; the model setpoint then applies again.

On Daniel the boost margin is 0 (boost off) since 13 Nov. On 13 Nov 12:00–13:30
boost setpoints 3–7 K above the outlet did not move the compressor (38–44) for
90 minutes; at 13:31 ES5 came on and it went 45 → 85 in 4 minutes. On 12 Nov a
2-minute boost to 33 at the start of `Running` preceded a runaway to 92. Both
fit the stored-shortfall pattern (below).

**Setpoint floor.** When the outlet is above the target, the written setpoint is
kept at least at outlet + `water_hyst_neg` + 1 (outlet − 3 K for Daniel). The
floor follows a rising outlet immediately and a falling one at 0.15 K/min, so
the 0.7 K steps of the outlet sensor no longer make the setpoint flicker
between two values. `Starting` and `EarlyRun` never write less than 20 °C, the
LG minimum (writing less made it rewrite every run).

**Energy state.** This handles the fast part, using filtered power against the
target:

| Condition | Energy state |
| --- | ---: |
| More than the deadband short | ES5 |
| More than the deadband over | ES7 |
| Within the deadband (`Power target deadband`, default 300 W) | ES2 |

Below 750 W (filtered) the shortfall threshold is 100 W instead of the
deadband: there the compressor runs at or near its minimum speed (15, about
540 W) with a poor COP (2.05 against 2.6 at about 1 kW, night of 9 Nov at 1–2 °C). On
9 Nov the deadband let a 750 W target run at 540 W for 81 minutes.

ES6 is not used (since 13 Nov). It was meant for small steps from a low
compressor speed, but between 10 and 13 Nov it never moved the compressor: 66
minutes at 15 (12 Nov 01:12, 517 W against 750 W), 2 hours at 44 (12 Nov
03:00, 1.94 kW against 2.5 kW) and 2 hours at 38–44 (13 Nov 11:15).

The compressor sits on a few speeds (about 15, 27, 36, 45, 55, 64, 73, 85, 92
in these runs) and rarely moves to the next one on ES5 alone: on 11 Nov
02:00–06:30 ES5 held 47 (2.1 kW against 2.5 kW) for 4.5 hours with the
setpoint capped at outlet + 1.5 K. Open problem.

**Stuck energy state.** When ES5/ES6 or ES7 has been active for 15 minutes and
is still wanted, the LG is holding the compressor against the setpoint and the
energy state alone does not reach the target. The setpoint then steps 1 K in
that direction, every 15 minutes while it lasts, relative to the outlet
(rounded down): down for ES7 but not below outlet − 1 K, up for ES5 from at
least the outlet and not past outlet + 1.5 K. Not during a boost or a brake. On
12 Nov, ES7 steps took the setpoint to 21 with the outlet at 23.5; afterwards
four upward steps (an hour at 517 W) were needed before it was above the outlet
again. On
9 Nov this would have ended 45 minutes at 3.7 kW against 2.5 kW (ES7) and
47 minutes stuck at 2.0 kW (ES5, outlet 30.8 above setpoint 30).

ES8 is never used, because it stopped the compressor in every test. Leaving
ES5/ES6/ES7 for ES2 happens immediately. Starting a new boost or reduction
requires 5 minutes in ES2 first, so the LG can settle, except right after a
target change. The direction (increase or decrease) only reverses through ES2.
During defrost the energy state and setpoint are held, and the control
continues afterwards. When the target returns to 0 or the run ends, ES2 is
set.

**Silent mode** is forced off in `EarlyRun` and `Running` while the target is
above `Power target silent mode limit` (default 2000 W), and while the
controller pushes upward (boost or ES5) to a target above 1 kW, because
silent mode caps the compressor at about 1 kW.

**Silent brake.** Silent mode on pulls a running compressor down within a minute
(85 → 56 on 8 Nov 12:00), much faster than ES7. The controller holds it on:

- on a step down of more than the deadband;
- when the compressor runs away upward: power more than the deadband above the
  target and the compressor 10 or more above its lowest speed of the last
  5 minutes. Not in the first 15 minutes after a compressor start or 10 minutes
  after a defrost, whose own ramps silent mode does not cap. A runaway brake
  also lowers the setpoint to 1 K below the old value and not above the outlet
  (rounded down), so the LG has no demand left to climb toward when the brake
  releases. Before this (9 Nov 12:29–12:46) every release after 120 s started
  the same climb again, and the 1-minute check missed the third one.

The brake is released when the meter shows at most the target plus half the
deadband, but not before `Power target silent brake minimum time` (default
120 s, 0 turns the brake off), or at once on a new step up. A step down is not braked while the evaporator
is freezing (`temp18`/`temp20` ≤ 0); a runaway is, because the runaway itself
chills the evaporator. On 12 Nov `temp18` went below 0 at 10:52 with 5.7 °C
outdoor, which blocked the brake, and the compressor stayed at 76–92
(3.5–4.4 kW against 2.5 kW) for 30 minutes, with the setpoint 1–3 K below the
outlet. The brake ends at a defrost. Every
release lets the LG pick its own speed again, which can start a new runaway;
the runaway check then brakes again. The brake replaced a 60 s silent mode kick
(c474b38, 7–8 Nov 2026) that toggled silent mode regardless of direction: it
turned silent mode on during a step up, and its release at 11:46 on 8 Nov let
the compressor run from 46 to 85 against a 0.75 kW target.

**Calibration.** `Floor heat exchanger efficiency` (eps) and `Estimated floor
temperature` are published. The Daniel profiles also publish `Floor UA
measured`, a 30-minute average calculated from the floor sensor, for comparison
with `Power target floor UA`. Its default comes from the `floor_ua_initial`
substitution (1700 W/K, measured on the Daniel installation for Oct 27 – Nov 6).
The floor sensor measures the surface, so the measured UA drops while the floor
is warm. `Power target state` shows Off / Boosting / Braking / Increasing /
Decreasing / Holding / Limited (compressor at 80 or more and still short) /
Defrost / Waiting for model. The `power_target` lines in the device log give the
model setpoint with its floor and outdoor temperature, and each boost and brake
start and end with its reason.

The cooling power control still uses the earlier formula.

### Observed heat pump behaviour

These observations from the Daniel installation (Oct 27 – Nov 6 2026, HA
recorder) are the basis for the power target design. Re-check them before
changing the controller.

**Plant speed.** A water setpoint change alone is slow. A +4 K step took about
65 minutes to raise power from 960 W to about 1900 W. After an energy state
returns to ES2, the LG keeps its compressor speed for about 5–25 minutes before
it follows its own setpoint again. Any per-cycle recalculation of the setpoint
from current power therefore fights the energy state and produces a swing of
about one hour. The user rejected an integral term for the same reason.

**Energy states** (holding register 9):

| ES | Observed effect |
| ---: | --- |
| 2 | Normal operation. |
| 5 | Drives the compressor to maximum quickly (29 → 85 in about 20 minutes). |
| 6 | Moderate step up from low compressor speed (reaches about 35–47), sometimes after a delay of up to 17 minutes. No effect above about 55. |
| 7 | Gentle decrease, about one compressor step per 2–3 minutes. |
| 8 | Stopped the compressor within 70–100 s in all three tests, restarting at 30 after 3–6 minutes. Not used. |

**Ceiling.** The highest draw seen was 3.87 kW at compressor 85, so targets
above about 3.9 kW are not reachable. The controller then shows `Limited`.

**The LG's own compressor model.** Analysed on Aug–Nov 2024 (recorder database
in the `core_2024.8.1` backup) and Aug–Nov 2026. In settled operation with
silent mode off, compressor speed follows the setpoint and outdoor temperature,
not the outlet error:

    model speed ≈ 15.6 + 1.54 × (water setpoint − outdoor temperature)

This explains R² 0.66 of settled speeds (sd ±3.5); setpoint − outlet explains
none. The LG accepts a 2–4 K outlet shortfall for hours, and the compressor
stays put whatever is written: 13 Oct 2026 at 35, 6 Nov at 52–57, 7 Nov
13:22–15:16 at 52–54 through ES5, ES7 and setpoints of 28–33 °C. It is not a
write rate limit: on 13 Oct nothing was written for two hours. Silent mode caps
the compressor at about 30. Turning it off lets the compressor jump to its own
demand (37–47) within about 3 minutes, never higher.

*Upward* (378 setpoint increases, no ES active): the compressor only responds
when the setpoint is above the outlet **and** the model speed for the new
setpoint is at least about 12 steps above the current speed. That held for 63%
of 30 steps (median +8); otherwise 4–8% responded. Example: going from 52 at
10 °C outside needs a model speed of about 64, so a setpoint of about 41.5 °C.
ES5 is not covered by this rule. It worked 3 of 3 times with setpoint − outlet
≥ 5 K and 1 of 4 times at about 3 K; the 7 Nov 12:46 success and 13:46 failure
had the same model headroom.

*Downward* (setpoint decreases ≥ 40 min after start, no stop): the compressor
only comes down when the outlet is above the setpoint by 0.5 K or more. With the
setpoint at or above the outlet it practically never drops, however far the
model speed is below. When the outlet is above the setpoint, the speed of
descent depends on how far the compressor is above the model speed:

| Compressor − model speed | No ES (drop in 20 min) | ES7 |
| --- | --- | --- |
| > 20 | 42% drop ≥ 5, median −4 | 100%, median −24 |
| 6–20 | 86–97%, median −31 to −35 | few cases |
| < 6 | 39–44%, median −4 | 25–62%, median −3 to −6 |

ES7 on its own is a slow creep. It mainly helps when the compressor is far
above the model speed.

*Self-stops* with heat still enabled happen in two situations. At minimum speed
(about 20), the LG stops when the outlet reaches the setpoint. With the outlet
2.1–3.4 K above the setpoint it sometimes stops at 23–46, though many such cases
in 2024 kept running. The `outlet + water_hyst_neg + 1` floor (outlet − 3 K for
Daniel) sits right in that zone.

**Pump ΔT control.** Since 2026 the pump runs in ΔT control (kept for its COP at
low power). In the 2026 data, above the minimum flow of about 17 L/min the water
ΔT stays at 5.3–5.5 K while flow rises with the compressor (18–20 L/min at
compressor 38, 23–27 at 48, 32–40 at 54, where ΔT falls to about 3 K). In 2024
the flow was fixed at about 17.2 L/min. Flow follows the compressor with a lag of
about 2.5 minutes, so flow is an effect of the LG's choice, not a cause. It
therefore correlates strongly with compressor speed (adding it to the speed
model raises R² from 0.66 to 0.93) without predicting it.

**Silent mode release and runaways** (56 silent on → off transitions while
running, 2024 and 2026). Most releases change nothing. Three times the
compressor ran away upward while the outlet was already above the setpoint:
31 Oct 13:53 (to 75), 6 Nov 10:34 (to 77) and 8 Nov 11:47 (to 85). The jumps to
80 in 2024 and on 6 Nov 04:35 were defrosts. On 8 Nov the compressor moved first
(37 s after the release, flow still flat); the setpoint had been 4–6 K above the
outlet for half an hour before, the largest accumulated shortfall of all
releases. On 7 Nov 14:34 the same situation (long shortfall, then setpoint
33 → 28 with ES7) without a release gave a slow decrease. This fits a stored
shortfall inside the LG that steady running holds back and a release lets out,
but a release on 8 Nov 11:02 with a smaller shortfall did nothing, and the
31 Oct and 6 Nov runaways followed an hour of outlet *above* the setpoint. Those
two first held a plateau for 8–9 minutes, then the pump raised flow (ΔT above
target) and the compressor followed, so there may be a second path through the
flow. Open question; the runaway check in the brake covers both.

On 9 Nov a re-trim raised the setpoint from 29 to 30 at 11:58 with the outlet at
29.3 and power just under the 2.5 kW target. After 25 minutes below the
setpoint (and flow rising from 23 to 30 L/min from 12:17) the compressor
stepped 56 → 66 → 72 → 76 → 85 within 3 minutes, with silent mode off and no
release involved. Silent mode brought it back to 56 within a minute three
times; each release after 120 s gave the same staircase. This is the
stored-shortfall pattern without a release, hence the outlet lead limit.

On 10 Nov (2.5 kW target, 0 °C) a defrost came right at the start of `Running`.
Afterwards the calculated setpoint of 29 was written with the outlet at
23–24; the compressor went 66 → 105 within 5 minutes and on to 126 (4.7 kW)
over 20 minutes. ES7 from 04:51 did not slow it; the runaway check was still
in its post-defrost window and the brake was blocked by the freezing
evaporator. The stuck-state step lowered the setpoint to 28 only at 05:06, and
the run ended in a second defrost. With the outlet lead the setpoint would have
been 24–25 (ES2) and the outlet at most (ES7).

**Water temperature sensors.**

- LG Modbus outlet/return: 0.35 K steps. During pump-only circulation outlet −
  return averages −0.39 K (sd 0.33), constant over weather. This is the
  `Water dT offset`.
- 1-Wire probes (Daniel): they read low compared with the LG, and the error
  grows as outdoor temperature falls (dT too high by 0.2–0.5 K at 13–15 °C,
  1.4–1.6 K at 3–5 °C, mostly on the return probe). They lag 2–4 K after idle
  periods. In 35 of 128 steady windows they gave eps ≥ 1, which is physically
  impossible. Counter-evidence: COP/Carnot computed from the probes is steadier
  (0.39–0.49) than from the LG pair (falls to about 0.25 at night and low
  compressor speed). This is unresolved. The probe mounting is unknown.

**Floor UA** (LG pair + 0.39 K, 128 steady 30-minute windows): median
1690 W/K, IQR 1570–1840. It is about 1670 at 17 l/min and about 1865 above
19 l/min. It drops to about 900 W/K when the floor surface is warm, because the
floor sensor measures the surface and not the slab. Re-run
`tools/steady_windows.py` to recalibrate; use the same sensor pair the
controller uses.

### Minimum-power avoidance (MPA)

During normal heating, when no target-power request is active, a nested
controller attempts to keep compressor operation away from very low output.
It observes compressor speed and filtered readings from LG registers 18 and
20, gradually adjusts an internal water target, and has idle, initializing,
stabilizing, reinitialize, and active states. The observed speed thresholds
and freeze checks are heuristics, not manufacturer protections.

### Derived heat-pump telemetry

- **Water-side thermal power:** `(outlet temperature - inlet temperature) ×
  flow / 60 × 4168`; cooling mode reverses the sign.
- **Heatpump COP:** calculated water-side thermal power divided by measured
  electrical active power.
- **Pressure-based estimated COP:** pressure readings are interpolated through
  embedded pressure/temperature tables to estimate condensing and evaporating
  temperatures. An empirical efficiency-versus-temperature-lift table then
  yields a clamped COP estimate.
- **Superheat:** suction-temperature reading minus estimated evaporating
  temperature.

The measured-power COP and pressure-based estimated COP are separate
calculations and should not be treated as interchangeable. Some derived values
depend on valid sensors and nonzero denominators; check telemetry for
availability and plausibility before using it for automation.

## Exposed controls and observability

The ESPHome API exposes Modbus-backed LG controls and measurements, profile
inputs, and template controls for thermostat target, weather curve settings,
curve offset, error gain, minimum run time, minimum after-run time, power
target parameters, force-run end, and restart. Controller and MPA state are published as text sensors.
Wi-Fi signal and uptime are also published.

The heating controller is scheduled every 20 seconds; a separate 60-second
interval decrements the thermostat timer and re-runs the thermostat script.
Several temperature and power values are moving averages intended to smooth
noisy readings.

## Secrets and deployment

The repository expects a local `secrets.yaml` for Wi-Fi, OTA, fallback access
point, and (for encrypted profiles) API credentials. It is ignored by Git.
Never commit actual credentials. Review API encryption and OTA configuration
for the selected profile before exposing the device to an untrusted network.

Validate a selected YAML with `esphome config <profile>.yaml` and compile with
`esphome compile <profile>.yaml`. `requirements.txt` intentionally leaves the
ESPHome version unpinned, so a new install or explicit upgrade may change
ESPHome behavior; compile and review the merged configuration before updating
hardware.

## Known assumptions and verification needs

- A number of heat-pump registers are explicitly labelled unknown; do not
  infer their meaning from adjacent addresses.
- The custom control loop uses fixed thresholds and empirical approximations
  alongside Home Assistant and installation-specific sensor assumptions.
- Profile differences are significant; selecting the wrong profile can use
  incorrect GPIOs or room-sensor inputs.
- Daniel and new-PCB profiles require a Home Assistant
  `input_number.wp_target_power` entity whose value is expressed in kW.
- Confirm that any selected profile provides all IDs referenced by shared
  package lambdas before building.
- The power target COP model (`cop_*`) and the LG compressor speed estimate
  behind the boost (15.6 + 1.54 × (setpoint − outdoor)) are fitted on the Daniel
  installation; the speed estimate dates from fixed-flow operation. Check boost
  size and model setpoints against the first optimizer runs (Nov 2026).
- No automated unit or hardware-in-the-loop tests are present in the tracked
  project files. Use ESPHome configuration validation, inspect live telemetry,
  and verify control actions on the actual installation.
