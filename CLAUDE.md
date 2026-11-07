# Agent guide

ESPHome firmware for an LG Therma V heat pump. `DESIGN.md` is the design
reference: control behaviour, the power target model, and the observed heat
pump behaviour it is based on. This file covers how to work on the code.

## Where to change what

| Task | Place |
| --- | --- |
| Heating/cooling control, energy state, setpoint | `packages/state_machine.c` |
| Settings exposed to HA (template `number`s), derived sensors, thermostat script | `packages/generic.yaml` |
| LG registers, coils, LG-based sensors | `packages/LGheatpump.yaml` |
| Power meter registers | `packages/Powermeter.yaml` |
| Per-installation inputs (HA sensors, 1-Wire), substitutions, pins | `LGtherma_*.yaml` |
| Per-installation constants and idle behaviour (`C->...`) | `customize/custom_<profile>.h` |
| Recalibrating floor UA / dT offset from recorder data | `tools/steady_windows.py` |

When behaviour changes, update the matching section of `DESIGN.md` too.

## `state_machine.c` conventions

- It is not a C file. It is the body of an ESPHome `interval` lambda
  (`!include` from `generic.yaml`) that runs every 20 s. `static` locals keep
  state between runs; everything else is recomputed on each run.
- States: `state` is the current state, and assigning `newstate` changes it at
  the end of the run. `Defrosting` falls through into `Running`, so code there
  checks `state == Running` before acting on measurements.
- During the first 60 s after boot the lambda only infers the state and
  returns early.
- `set_target_temp(x)` rounds to whole degrees and only writes when the value
  differs from the current LG target by more than 0.7 K.
- Power target values are in W. `power_target_effective` is HA's
  `input_number.wp_target_power` (kW) × 1000, forced to 0 after 10 minutes
  without the API. A target counts as active when it is ≥ 0.05. Use
  `requested_power` in the state machine, not `id(wp_target_power)`.
- Power target code uses a `pt_` prefix for its statics. `pt_status` is
  published at the end of the run.
- `Running` uses filtered power `lg_total_active_power_filtered` (30 × 5 s), not
  the raw meter value.

## Profile contract

Shared code references these IDs, so every profile must define them:
`huiskamer_lucht_hass`, `huiskamer_vloer`, `wp_target_power`, `hp_temp_out`,
`hp_temp_in`, `api_id`. It must also define the substitutions
`thermostat_step_size`, `thermostat_initial_value`, `max_wtemp_custom`,
`floor_ua_initial` and the four `GPIO_modbus_*`. Marco fakes the HA and 1-Wire
inputs with templates and has no power target (fixed 0 W). A new shared input
or substitution has to be added to all three `LGtherma_*.yaml`.

New settings follow the existing template number pattern: `mode: box`,
`restore_value: yes`, `optimistic: true`, with an initial value. Because of
`restore_value`, a changed `initial_value` only applies to a fresh device; the
value stored on the device wins.

## Decisions not to revisit without new evidence

- Heating control uses only the LG Modbus temperatures, the LG flow and the
  power meter. The 1-Wire probes are for display and cooling only (their error
  depends on outdoor temperature). HA inputs besides the power target must not
  be needed for heating control.
- No integral/PI term on power: the plant takes about an hour to settle. The
  setpoint is calculated once per target change and held. The energy state does
  the fast correction.
- Never write ES8: it stops the compressor.
- `Heatpump Water power` and `Heatpump COP` stay on the 1-Wire probes for
  history continuity (decided Nov 2026).

## Home Assistant entity map (Daniel installation)

The ESPHome device is `lgtherma`. Older entities have no prefix; entities
added in Nov 2026 have the `warmtepomp_lgtherma_` prefix.

| ESPHome id | HA entity |
| --- | --- |
| `water_temp_aanvoer` / `water_temp_retour` | `sensor.water_aanvoer` / `sensor.water_retour` |
| `hp_temp_out` / `hp_temp_in` (1-Wire) | `sensor.heatpump_temp_outflow` / `sensor.heatpump_temp_inflow` |
| `current_flow_rate` | `sensor.flow_rate` |
| `compressor_speed` | `sensor.compressor_speed` |
| `lg_total_active_power` | `sensor.lg_therma_total_active_power` |
| `water_temp_target_output` | `number.lg_water_temperature_target` |
| `energy_state` | `number.lg_energy_state` |
| `modbus_set_silent_mode` / `silent_mode` | `switch.lg_set_silent_mode` / `binary_sensor.silent_mode` |
| `compressor_running`, `defrosting`, `pump_running` | `binary_sensor.compressor_running`, `.defrosting`, `.pump_running` |
| `lg_controller_state` | `sensor.lg_controller_state` |
| `thermostat_wp_heat` | `switch.wp_heat` |
| `buiten_temp` | `sensor.buitentemperatuur` |
| `huiskamer_vloer` (HA input) | `sensor.huiskamer_vloer` |
| `wp_target_power` (HA input, kW) | `input_number.wp_target_power` |
| `power_target_state` | `sensor.warmtepomp_lgtherma_power_target_state` |
| `power_target_effective` | `sensor.warmtepomp_lgtherma_target_power_effective` |
| `water_dt_corrected` | `sensor.warmtepomp_lgtherma_water_dt_corrected` |
| `floor_eps` | `sensor.warmtepomp_lgtherma_floor_heat_exchanger_efficiency` |
| `floor_temp_est` | `sensor.warmtepomp_lgtherma_estimated_floor_temperature` |
| `floor_ua_measured` | `sensor.warmtepomp_lgtherma_floor_ua_measured` |
| `floor_ua` | `number.warmtepomp_lgtherma_power_target_floor_ua` |
| `water_dt_offset` | `number.warmtepomp_lgtherma_water_dt_offset` |
| `power_target_deadband` | `number.warmtepomp_lgtherma_power_target_deadband` |
| `power_target_max_wtemp` | `number.warmtepomp_lgtherma_power_target_maximum_watertemperatuur` |
| `power_target_silent_limit` | `number.warmtepomp_lgtherma_power_target_silent_mode_limit` |

The `power_target` debug log lines (setpoint calculation, re-trim, ES, power
every cycle) only show in the ESPHome device log, not in HA. Recorder history
lives in `/config/home-assistant_v2.db`; open it read-only
(`file:...?mode=ro`, as `tools/steady_windows.py` does).

## Validate, build, deploy

- On the Daniel Home Assistant host, build through the ESPHome add-on with
  `/config/scripts/esphome validate|compile lgtherma_daniel.yaml` (see
  `/config/CLAUDE.md`). `lgtherma_daniel.yaml` in `/config/esphome` is a wrapper
  that includes this repo's profile. Validate and compile are safe to run at
  any time and catch C++ errors in the lambdas. `upload` flashes the heat pump
  controller and needs the user's confirmation every time.
- Without the add-on: validate in a throwaway venv (`pip install esphome`),
  `esphome config <profile>.yaml`, with `secrets.yaml` temporarily copied into
  the repo root (it is gitignored). That does not compile lambdas, and the
  ESP-IDF toolchain does not run in the Alpine-based Claude Code add-on. If an
  attempt creates `/data/.cache/esphome` or `/config/esphome/.esphome`, remove
  them.
- Known existing failure: the Marco and new-PCB profiles use `api: password:`,
  which ESPHome 2026.1 removed. Strip that line in a temporary copy to validate
  the rest.
- The builder compiles whatever branch is checked out in this repo. Do work on a
  branch; commit or push only when the user asks.
