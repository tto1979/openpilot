"""
Translations for the opview API: the strings are the UI's (selfdrive/ui/translations/app_<lang>.po), in the phone's
language when the app asks for one (?lang= or Accept-Language), otherwise in the device's language.
"""
from functools import lru_cache

from openpilot.common.params import Params
from openpilot.system.ui.lib.multilang import TRANSLATIONS_DIR, load_translations, multilang

# Accept-Language prefixes -> UI language codes (selfdrive/ui/translations/languages.json)
_ACCEPT_LANGUAGE = {
  "zh-tw": "zh-CHT", "zh-hk": "zh-CHT", "zh-mo": "zh-CHT", "zh-hant": "zh-CHT",
  "zh-cn": "zh-CHS", "zh-sg": "zh-CHS", "zh-hans": "zh-CHS", "zh": "zh-CHT",
  "pt-br": "pt-BR", "pt": "pt-BR",
}


@lru_cache(maxsize=16)
def _translations(lang: str) -> dict[str, str]:
  try:
    return load_translations(TRANSLATIONS_DIR.joinpath(f"app_{lang}.po"))[0]
  except FileNotFoundError:
    return {}


def resolve_language(query_lang: str | None, accept_language: str | None) -> str:
  codes = set(multilang.languages.values())
  if query_lang in codes:
    return str(query_lang)
  for part in (accept_language or "").split(","):
    tag = part.split(";")[0].strip().lower()
    if not tag:
      continue
    if tag in _ACCEPT_LANGUAGE:
      return _ACCEPT_LANGUAGE[tag]
    base = tag.split("-")[0]
    if base in _ACCEPT_LANGUAGE:
      return _ACCEPT_LANGUAGE[base]
    if base in codes:
      return base
  device = str(Params().get("LanguageSetting") or "en").removeprefix("main_")
  return device if device in codes else "en"


class Translator:
  def __init__(self, lang: str):
    self.lang = lang
    self._table = {} if lang == "en" else _translations(lang)

  def __call__(self, text: str) -> str:
    return self._table.get(text, text) or text
