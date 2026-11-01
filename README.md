# LG-thermaV-control

## Development environment

`requirements.txt` installs the latest ESPHome release. Python 3.12 is
recommended.

From the repository root, create and activate a virtual environment, then
install the project dependencies:

```sh
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

To upgrade an existing environment to the latest ESPHome release, run:

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

With the environment active, validate or compile a device configuration:

```sh
esphome config LGtherma_daniel.yaml
esphome compile LGtherma_daniel.yaml
```

Use the corresponding `LGtherma_*.yaml` file for a different device
configuration. ESPHome commands can also be run without activating the
environment by using `.venv/bin/esphome` (Linux/macOS) or
`.venv\Scripts\esphome.exe` (Windows).

To leave the environment, run `deactivate`.
