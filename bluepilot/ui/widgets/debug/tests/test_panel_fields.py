"""Fail offline when BluePilot code reads a capnp field that no longer resolves.

Upstream retires fields by moving them into a `deprecated :group` rather than deleting them,
so `x.foo` becomes `x.deprecated.foo` and the old path raises AttributeError at runtime. That
has bitten this fork repeatedly:

  * RadarData.RadarPoint aRel/yvRel/measured  -> killed `card` mid-drive (Ford radar)
  * RadarState.LeadData  aRel/dPath/fcw       -> blank cards in the debug panel
  * LateralTorqueTuning  kp/ki/kf             -> "N/A" in the tuning card
  * LongitudinalPIDTuning kpBP/kpV/kf         -> "N/A" in the tuning card

Grepping for the names is far too noisy, because most of them are still live on some *other*
struct. So this walks the AST for locals bound to `sm['<service>']`, collects the attribute
chains rooted at them, and evaluates each chain against a real message built from the live
schema. A chain that raises here is dead code on the device.

Known blind spot: chains rooted at a function parameter (e.g. `lead_rows(lead)`) are invisible
to the AST pass, so the structs the panels read through helpers are asserted explicitly below.
"""

import ast
import unittest
from pathlib import Path

import openpilot.cereal.messaging as messaging

REPO_ROOT = Path(__file__).resolve().parents[5]
SCAN_DIRS = ("bluepilot", "openpilot/sunnypilot")
# attributes that exist on the capnp builder itself, not on the message schema
NON_SCHEMA_ATTRS = {"which", "to_dict", "as_builder", "as_reader", "copy", "schema", "total_size"}


def _message(service):
  try:
    return getattr(messaging.new_message(service), service)
  except Exception:
    return None


def _chain(node):
  attrs = []
  while isinstance(node, ast.Attribute):
    attrs.append(node.attr)
    node = node.value
  return (node.id, list(reversed(attrs))) if isinstance(node, ast.Name) else None


class _Collector(ast.NodeVisitor):
  """Track locals bound to sm['service'] and every attribute chain rooted at them."""

  def __init__(self):
    self.bound = {}
    self.uses = []

  def visit_Assign(self, node):
    if len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
      value = node.value
      if (isinstance(value, ast.Subscript) and isinstance(value.slice, ast.Constant)
          and isinstance(value.slice.value, str)):
        base = value.value
        if (isinstance(base, ast.Name) and base.id == "sm") or \
           (isinstance(base, ast.Attribute) and base.attr == "sm"):
          self.bound[node.targets[0].id] = (value.slice.value, [])
      else:
        chain = _chain(value)
        if chain and chain[0] in self.bound and chain[1]:
          service, prefix = self.bound[chain[0]]
          self.bound[node.targets[0].id] = (service, prefix + chain[1])
    self.generic_visit(node)

  def visit_Attribute(self, node):
    chain = _chain(node)
    if chain and chain[0] in self.bound:
      service, prefix = self.bound[chain[0]]
      self.uses.append((service, prefix + chain[1], node.lineno))
    self.generic_visit(node)


class TestCapnpFieldsResolve(unittest.TestCase):
  def test_no_reads_of_deprecated_fields(self):
    broken = []
    for scan in SCAN_DIRS:
      for path in sorted((REPO_ROOT / scan).rglob("*.py")):
        if "__pycache__" in str(path) or "/tests/" in str(path):
          continue
        try:
          tree = ast.parse(path.read_text(errors="ignore"))
        except SyntaxError:
          continue
        collector = _Collector()
        collector.visit(tree)
        seen = set()
        for service, attrs, line in collector.uses:
          if not attrs or attrs[-1] in NON_SCHEMA_ATTRS:
            continue
          key = (service, tuple(attrs))
          if key in seen:
            continue
          seen.add(key)
          msg = _message(service)
          if msg is None:
            continue
          obj = msg
          try:
            for attr in attrs:
              obj = getattr(obj, attr)
          except AttributeError as exc:
            if "no such member" in str(exc):
              rel = path.relative_to(REPO_ROOT)
              broken.append(f"{rel}:{line}  sm['{service}'].{'.'.join(attrs)}")
          except Exception:
            continue  # union not initialised, index errors, etc -- not schema drift
    self.assertEqual(broken, [], "capnp fields that no longer resolve:\n  " + "\n  ".join(broken))


class TestPanelStructsExplicitly(unittest.TestCase):
  """Structs the debug panels reach through helper functions, invisible to the AST pass."""

  def test_radarstate_lead_fields(self):
    lead = messaging.new_message('radarState').radarState.leadOne
    for field in ('dRel', 'yRel', 'vRel', 'vLead', 'aLeadK', 'modelProb', 'present',
                  'radarTrackId', 'radar'):
      with self.subTest(field=field):
        getattr(lead, field)

  def test_lateral_torque_tuning_fields(self):
    cp = messaging.new_message('carParams').carParams
    cp.lateralTuning.init('torque')
    for field in ('friction', 'latAccelFactor', 'latAccelOffset', 'steeringAngleDeadzoneDeg'):
      with self.subTest(field=field):
        getattr(cp.lateralTuning.torque, field)

  def test_longitudinal_tuning_fields(self):
    lt = messaging.new_message('carParams').carParams.longitudinalTuning
    for field in ('kiBP', 'kiV'):
      with self.subTest(field=field):
        getattr(lt, field)


if __name__ == '__main__':
  unittest.main()
