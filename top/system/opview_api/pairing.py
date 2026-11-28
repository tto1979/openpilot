"""The opview app's pairing code: shown on the device, typed into the app once (top/system/opview_api/server.py)."""
import secrets

from openpilot.common.params import Params

PAIRING_PARAM = "OpviewPairingCode"


def pairing_code(params: Params) -> str:
  """The device's 6 digit pairing code, made on first use"""
  code = params.get(PAIRING_PARAM)
  if not code:
    code = f"{secrets.randbelow(10**6):06d}"
    params.put(PAIRING_PARAM, code)
  return str(code)


def reset_pairing_code(params: Params) -> str:
  """A new code: phones paired with the old one have to pair again"""
  params.remove(PAIRING_PARAM)
  return pairing_code(params)
