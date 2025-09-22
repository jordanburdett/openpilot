# Mach-E Debugging Quickstart

This document collects the practical steps we used while debugging Mach-E steering changes. It should be enough context for an assistant (human or AI) to repeat the workflow end-to-end.

## Repository Layout

- `openpilot/` — main sunnypilot sources. Most custom logic (e.g., Mach-E controller) lives under `sunnypilot/selfdrive/...`.
- `opendbc_repo/` — CAN + vehicle definitions. Treated as a submodule; we avoid editing it directly.
- `bluepilot/` — upstream reference fork used for comparison (bp-4.0 branch).
- `tools/` — host utilities (log parsing, repl, etc.).

## Branching

- All current work lives on branch `jb-new` in `github.com/jordanburdett/openpilot.git`.
- Submodule `opendbc_repo` is pinned to sunnypilot/master; no custom commits are required after the latest changes.

## Environment Setup

1. Activate the managed venv: `source .venv/bin/activate` (from repo root).
2. Build native deps (only needed on device or after clean): `scons -u`.
3. Preferred tests:
   - Steering interface fuzz: `python -m pytest selfdrive/car/tests/test_car_interfaces.py -k MACH_E -vv`
   - Ford CAN unit tests: `python -m pytest opendbc_repo/opendbc/car/ford/tests`
   - Static checks: `python -m mypy sunnypilot/selfdrive/car/interfaces.py sunnypilot/selfdrive/car/ford/mach_e_controller.py`
   - Import sanity: `python -m compileall sunnypilot/selfdrive/car/interfaces.py sunnypilot/selfdrive/car/ford/mach_e_controller.py`

## Connecting to the Comma Device

The repo assumes an SSH config entry named `comma3x`:

```bash
Host comma3x
    HostName 192.168.2.214
    User comma
    IdentityFile ~/.ssh/comma3x_key
    IdentitiesOnly yes
```

Common commands:

- File listing: `ssh comma3x "ls /data"`
- Pull latest repo state on device: `ssh comma3x "cd /data/openpilot && git fetch origin jb-new && git checkout jb-new && scons -u"`
- Sync logs: `scp comma3x:/data/log/swaglog.0000000007 ./logs/`

## Logs & CAN Analysis

### Swaglogs

- Location: `/data/log/swaglog.*`
- Grep for CAN parser faults: `ssh comma3x "grep -n 'CANParser' /data/log/swaglog.*"`

### Recorded Drives

- Stored at `/data/media/0/realdata/<route>/<segment>/`.
- Typical segment contents: `qlog.zst`, `rlog.zst`, `fcamera.hevc`, ...
- Download: `scp comma3x:/data/media/0/realdata/<route>/<segment>/qlog.zst /tmp/`
- Decompress: `zstd -d -f /tmp/qlog.zst -o /tmp/qlog`

### Python Helpers

Examples (run after `source .venv/bin/activate`):

```python
from tools.lib.logreader import LogReader
from collections import Counter

alerts = Counter()
for m in LogReader('/tmp/qlog'):
    if m.which() == 'controlsState':
        cs = m.controlsState
        alerts[cs.alertSound2DEPRECATED] += 1
print(alerts)
```

Decode Ford LCA packets (0x186 / 982):

```python
from opendbc.can.parser import CANParser

parser = CANParser('ford_lincoln_base_pt', [('LateralMotionControl2', 0)], 0)
for m in LogReader('/tmp/qlog'):
    if m.which() == 'sendcan':
        frames = [(c.address, c.dat, c.src) for c in m.sendcan]
        parser.update([(m.logMonoTime, frames)])
        lat = parser.vl['LateralMotionControl2']
        if lat['LatCtl_D2_Rq'] != 0:
            print(m.logMonoTime, lat)
            break
```

### Inspect Steering Faults

```python
fault_samples = []
for m in LogReader('/tmp/qlog'):
    if m.which() == 'carState' and m.carState.steerFaultTemporary:
        fault_samples.append((m.logMonoTime, m.carState.steeringTorque))
print('fault count', len(fault_samples))
```

## Common Failure Signatures

- `LatCtlRampType_D_Rq = 0` with non-zero curvature triggers PSCM lockout; send `2` when engaged.
- Missing CAN frames (CANParser warnings in swaglog) usually indicate Panda or harness issues; confirm `/data/log/swaglog.*` before digging into software.
- `carState.canValid == False` or `vehicleSensorsInvalid == True` points to upstream sensor data missing (e.g., if `SteeringPinion_Data` is absent).

## Code Touchpoints

- Mach-E specific controller wrapper: `sunnypilot/selfdrive/car/ford/mach_e_controller.py`
- Controller injection hook: `sunnypilot/selfdrive/car/interfaces.py`
- Reference implementation: `bluepilot/opendbc_repo/opendbc/car/ford/carcontroller.py`
- CAN helpers: `opendbc_repo/opendbc/car/ford/fordcan.py`

## Workflow Checklist

1. Reproduce and capture a short drive (`qlog.zst`).
2. Pull logs locally, inspect for `LatCtl_*` values and PSCM faults.
3. Compare behaviour against Bluepilot when in doubt.
4. Adjust controller logic, run unit + static tests, push to `jb-new`.
5. Deploy build onto comma device and verify on road.

