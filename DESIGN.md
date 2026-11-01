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
1-Wire temperatures (where configured) ──────────────────────────┤
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

When `wp_target_power` changes during the `Running` state, the controller also
uses the LG energy-state register to help ramp electrical power toward the
new target:

| Target change | Initial energy state | Within 500 W of target |
| --- | ---: | ---: |
| Down by 1000 W or more | ES8 | ES7 |
| Down by at least 250 W but less than 1000 W | ES7 | ES7 |
| Up by at least 250 W and no more than 1000 W | ES6 | ES6 |
| Up by more than 1000 W | ES5 | ES6 |

Once measured active power crosses the new target in the requested direction
(at or below it for a reduction, at or above it for an increase), the
controller selects ES2. Changes smaller than 250 W do not change the energy
state. A further command received while running replaces any pending
transition. Target changes outside `Running` do not start a transition.

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
curve offset, error gain, minimum run time, minimum after-run time, force-run
end, and restart. Controller and MPA state are published as text sensors.
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
- No automated unit or hardware-in-the-loop tests are present in the tracked
  project files. Use ESPHome configuration validation, inspect live telemetry,
  and verify control actions on the actual installation.
