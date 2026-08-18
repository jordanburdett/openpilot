"""Loud, visible failure reporting for the onroad debug panels.

These panels used to wrap their whole update in
`except (KeyError, AttributeError, ValueError, IndexError): pass`. That is how six separate
capnp schema-drift bugs (RadarState lead aRel/dPath/fcw, lateralTuning torque kp/ki/kf,
longitudinalTuning kpBP/kpV/kf) sat unnoticed: the cards simply rendered blank.

Letting them propagate instead is not an option here. The panel is an *onroad* overlay and
`GuiApplication.render()` only catches KeyboardInterrupt, so a raise would kill the `ui`
process mid-drive -- no speed, no alerts on screen. Losing the display while moving is a worse
outcome than a broken debug card.

So: never silent, never fatal. Every failure is logged once per site with a full traceback
(so it lands in swaglog) and is surfaced as text in the panel itself, where you cannot miss it.
The real gate is offline -- see bluepilot/ui/widgets/debug/tests/test_panel_fields.py, which
fails on this class of bug where failing is free.
"""

import logging

logger = logging.getLogger(__name__)

# Sites that have already logged, so a failure in a 20Hz update loop doesn't flood swaglog.
_reported: set[str] = set()


def report(site: str, exc: BaseException) -> str:
  """Log a debug-panel failure once per site and return a short string to display.

  Args:
    site: stable identifier for the failing update, e.g. "other.radar"
    exc: the exception that was caught

  Returns:
    A one-line description suitable for rendering in the panel.
  """
  detail = f"{type(exc).__name__}: {exc}"
  if site not in _reported:
    _reported.add(site)
    logger.exception("debug panel %s failed to update", site)
  return detail[:120]


def reset() -> None:
  """Allow every site to log again. For tests."""
  _reported.clear()
