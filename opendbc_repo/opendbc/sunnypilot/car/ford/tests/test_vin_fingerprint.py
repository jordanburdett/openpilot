"""BluePilot: VIN fallback fingerprinting for Ford.

VINs are real ones from comma's public car segments database (tools/car_porting/README.md),
with the serial digits already redacted to X -- X is a legal VIN character, so they still parse.
"""

from opendbc.car.ford.values import CAR, match_vin_to_car

# (vin, expected platform or None)
CASES = [
  ('1FM5K8GC7LGXXXXXX', CAR.FORD_EXPLORER_MK6),         # Explorer 2020
  ('1FM5K7LC0MGXXXXXX', CAR.FORD_EXPLORER_MK6),         # Explorer 2021
  ('1FM5K8GC7NGXXXXXX', CAR.FORD_EXPLORER_MK6),         # Explorer 2022
  ('5LM5J7XC9LGXXXXXX', CAR.FORD_EXPLORER_MK6),         # Lincoln Aviator 2020
  ('5LM5J7XC8MGXXXXXX', CAR.FORD_EXPLORER_MK6),         # Lincoln Aviator 2021
  ('1FMCU9J94MUXXXXXX', CAR.FORD_ESCAPE_MK4),           # Escape 2021
  ('3FMCR9B69NRXXXXXX', CAR.FORD_BRONCO_SPORT_MK1),     # Bronco Sport 2022
  ('3FTTW8F98NRXXXXXX', CAR.FORD_MAVERICK_MK1),         # Maverick 2022
  ('3FTTW8E31PRXXXXXX', CAR.FORD_MAVERICK_MK1),         # Maverick 2023
  ('3FMTK3SU0MMXXXXXX', CAR.FORD_MUSTANG_MACH_E_MK1),   # Mach-E 2021
  ('1FTVW1EL4NWXXXXXX', CAR.FORD_F_150_LIGHTNING_MK1),  # position 8 = L -> electric
  ('1FTFW1E85MFXXXXXX', CAR.FORD_F_150_MK14),           # same body code, position 8 = E -> ICE
  ('WF0NXXGCHNJXXXXXX', None),                          # Focus, European VIN scheme
  ('00000000000000000', None),                          # VIN_UNKNOWN
  ('1FM5K8GC7LG', None),                                # malformed
  ('', None),                                           # no VIN at all
  ('1FM5K8GC7TGXXXXXX', None),                          # Explorer body code, model year outside 2020-24
]


def test_match_vin_to_car():
  for vin, expected in CASES:
    matches = match_vin_to_car(vin)
    if expected is None:
      assert matches == set(), f'{vin}: expected no match, got {matches}'
    else:
      assert matches == {str(expected)}, f'{vin}: expected {expected}, got {matches}'


def test_no_platform_matches_two_ways():
  """Any VIN the fallback accepts must resolve to exactly one platform, otherwise
  fingerprint() discards it anyway and the tables are wrong."""
  for vin, expected in CASES:
    if expected is not None:
      assert len(match_vin_to_car(vin)) == 1, vin


if __name__ == '__main__':
  test_match_vin_to_car()
  test_no_platform_matches_two_ways()
  print('ok')
