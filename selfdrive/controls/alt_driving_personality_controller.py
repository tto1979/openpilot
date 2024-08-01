#!/usr/bin/env python3

import time
from openpilot.common.conversions import Conversions as CV
from openpilot.common.params import Params

class AlternativeDrivingPersonalityController:
    def __init__(self, CP=None):
        # try except to avoid issue in longitudinal testing
        try:
            self._speed = 40 * CV.KPH_TO_MS
        except Exception:
            self._speed = 0

        self._mode = 0
        self._active = False
        self.CP = CP
        self.params = Params()
        self.old_personality = None
        self.last_record_time = 0

    def update(self, v_ego):
        was_active = self._active
        
        openpilot_longitudinal_control = getattr(self.CP, 'openpilotLongitudinalControl', True)
        
        self._active = self._speed > 0 and v_ego < self._speed and openpilot_longitudinal_control
        current_time = time.time()

        # Record LongitudinalPersonality every 30 seconds when speed is above 40 km/h
        if not self._active and current_time - self.last_record_time >= 30:
            self.old_personality = self.params.get('LongitudinalPersonality')
            self.last_record_time = current_time

        if self._active and not was_active:
            # Switching to active state
            self.params.put_nonblocking('LongitudinalPersonality', str(self._mode))
        elif not self._active and was_active:
            # Switching back to inactive state
            if self.old_personality is not None:
              self.params.put_nonblocking('LongitudinalPersonality', self.old_personality)

    def get_personality(self, personality):
        return self._mode if self._active else personality

    def is_active(self):
        return self._active
