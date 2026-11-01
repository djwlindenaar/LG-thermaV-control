# LG-thermaV-control

ESPHome firmware for monitoring and controlling an LG Therma V heat pump from
an ESP32. It reads the heat pump and an optional power meter over separate
Modbus/RTU links, exposes controls and measurements to Home Assistant, and
implements a configurable heating controller.

The firmware can change heat-pump operating targets and enable/disable heating.
Check the selected profile, wiring, Modbus register mappings, and configured
limits against your installation before connecting it to live equipment.

## Device configurations

Choose the YAML configuration that matches your hardware and Home Assistant
entities:

| Configuration | Device name | Profile notes |
| --- | --- | --- |
| `LGtherma_daniel.yaml` | `lgtherma` | Daniel wiring, Home Assistant room sensors and power target, plus two 1-Wire temperature probes. |
| `LGtherma_daniel_newpcb.yaml` | `lgtherma_test` | New PCB pin assignments and 1-Wire probes. |
| `LGtherma_marco.yaml` | `esp-lgthermav` | Marco custom settings; maps room temperature and heat-pump water temperatures differently. |

These configurations include the shared files in `packages/`. Per-profile
substitutions select GPIO pins, device name, tuning values, and the matching
custom behavior in `customize/`. See [DESIGN.md](DESIGN.md) for the architecture,
control flow, and configuration details.

## Development environment

`requirements.txt` installs the latest ESPHome release. Python 3.12 is recommended.

From the repository root, create and activate a virtual environment, then
install the project dependencies:

```sh
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

To upgrade an existing environment to the latest ESPHome release, run while
the environment is active:

```sh
python -m pip install --upgrade esphome
```

In Windows PowerShell, activate it with:

```powershell
py -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

## Secrets and first build

Create a local `secrets.yaml` in the repository root; do not commit it. The
shared package expects the Wi-Fi, OTA, and fallback access-point keys below.
The API key is also needed by the Daniel and Marco profiles:

```yaml
wifi_ssid: "your-wifi-name"
wifi_password: "your-wifi-password"
wifi_domain: ".your-local-domain"
LG_therma_otapassword: "a-unique-ota-password"
LG_therma_WIFIpassword: "a-unique-fallback-ap-password"
LG_therma_apikey: "your-home-assistant-api-encryption-key"
```

The API encryption key is referenced by the Daniel and Marco profiles. The
new-PCB profile currently has API encryption commented out and an empty API
password; review and secure its API settings before deploying it. Keep real
credentials out of version control.

The Daniel and new-PCB profiles read `input_number.wp_target_power` from Home
Assistant and convert its value from kW to W. Marco's profile defines the same
sensor ID as a fixed zero-watt template, so target-power control is disabled
for that profile.

With the environment active, validate and compile the selected configuration:

```sh
esphome config LGtherma_daniel.yaml
esphome compile LGtherma_daniel.yaml
```

Replace the example filename with the profile you selected. Compilation
requires network access to download ESPHome platform/toolchain components. The
ESPHome dashboard/API can also be used for installation and operation.

ESPHome commands can be run without activating the environment by using
`.venv/bin/esphome` (Linux/macOS) or `.venv\Scripts\esphome.exe` (Windows).

To leave the environment, run `deactivate`.

## Repository layout

- `LGtherma_*.yaml` — per-device substitutions, package selection, API, and
  profile-specific sensors.
- `packages/LGheatpump.yaml` — LG heat-pump Modbus controls and measurements,
  plus derived pressure-based estimates.
- `packages/Powermeter.yaml` — Modbus power-meter readings.
- `packages/Powermeter_dummy.yaml` — zero-valued power-meter substitute for
  configurations that intentionally do not use a physical meter.
- `packages/generic.yaml` — ESP32/network setup, thermostat and weather-curve
  scripts, exposed controls, derived sensors, and controller scheduling.
- `packages/state_machine.c` — heating, cooling, defrost, after-run, and
  minimum-power-avoidance state logic.
- `customize/` — per-profile tuning, idle behavior, and lookup helper.
- `requirements.txt` — Python package requirement for ESPHome.

## Design

See [DESIGN.md](DESIGN.md) for component boundaries, data flow, Modbus
interfaces, controller behavior, profile variations, and operational caveats.
