#!/usr/bin/env python3

from openpilot.common.conversions import Conversions as CV
from openpilot.common.params import Params

class AlternativeDrivingPersonalityController:
    def __init__(self):
        # try except to avoid issue in longitudinal testing
        try:
            self._speed = 40 * CV.KPH_TO_MS
        except Exception:
            self._speed = 0

        self._mode = 0
        self._active = False
        self.params = Params()
        self.old_personality = None

    def update(self, v_ego):
        was_active = self._active
        self._active = self._speed > 0 and v_ego < self._speed

        if self._active and not was_active:
            # Switching to active state
            self.old_personality = self.params.get('LongitudinalPersonality')
            self.params.put_nonblocking('LongitudinalPersonality', str(self._mode))
        elif not self._active and was_active:
            # Switching back to inactive state
            if self.old_personality is not None:
                self.params.put_nonblocking('LongitudinalPersonality', self.old_personality)

    def get_personality(self, personality):
        return self._mode if self._active else personality

    def is_active(self):
        return self._active
