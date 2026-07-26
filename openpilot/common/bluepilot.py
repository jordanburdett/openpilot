"""BluePilot detection. Safe to import on any fork — returns False when not BluePilot."""
import os
from functools import cache

@cache
def is_bluepilot() -> bool:
  # BPVERSION lives at the REPO root. This file used to be common/bluepilot.py, one level down
  # from it; the upstream restructure moved it to openpilot/common/bluepilot.py, so the marker is
  # now two levels up. Getting this wrong fails silently and disables every BluePilot feature.
  return os.path.isfile(os.path.join(os.path.dirname(__file__), '..', '..', 'BPVERSION'))
