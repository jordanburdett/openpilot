"""Every BluePilot module must import cleanly.

Replaces test_backend_import.py and test_modules_only.py, which were print-based scripts
that returned True/False instead of asserting -- they could not fail a test runner, and
`bluepilot/` was not collected by tools/test_runner.py anyway, so they never ran at all.
Their hardcoded import lists also went stale silently.

This walks the package instead of naming modules, so new modules are covered automatically.
Each module is imported in a subprocess: importing them in-process would populate
sys.modules and mask a later module's broken import.
"""

import subprocess
import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
PKG_ROOT = REPO_ROOT / "bluepilot"

# The web/ tree is a TypeScript app; setup_web_routes is a one-shot installer script.
SKIP_PARTS = {"web", "node_modules", "__pycache__"}


def _modules() -> list[str]:
  mods = []
  for path in sorted(PKG_ROOT.rglob("*.py")):
    rel = path.relative_to(REPO_ROOT)
    if SKIP_PARTS & set(rel.parts):
      continue
    parts = list(rel.with_suffix("").parts)
    if parts[-1] == "__init__":
      parts.pop()
    if parts:
      mods.append(".".join(parts))
  return mods


class TestBluePilotImports(unittest.TestCase):
  def test_package_is_discoverable(self):
    self.assertTrue(PKG_ROOT.is_dir(), f"{PKG_ROOT} missing")
    self.assertGreater(len(_modules()), 50, "suspiciously few modules discovered")

  def test_every_module_imports(self):
    failures = []
    for mod in _modules():
      proc = subprocess.run([sys.executable, "-c", f"import {mod}"],
                            cwd=REPO_ROOT, capture_output=True, text=True, timeout=120)
      if proc.returncode != 0:
        last = proc.stderr.strip().splitlines()[-1] if proc.stderr.strip() else "no stderr"
        failures.append(f"{mod}: {last}")
    self.assertEqual(failures, [], "modules failed to import:\n  " + "\n  ".join(failures))


if __name__ == "__main__":
  unittest.main()
