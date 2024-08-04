#!/usr/bin/env python3
import threading
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
        self.v_ego = 0
        self.update_thread = threading.Thread(target=self.update_loop, daemon=True)
        self.update_thread.start()

    def update(self, v_ego):
        self.v_ego = v_ego
        was_active = self._active
        openpilot_longitudinal_control = getattr(self.CP, 'openpilotLongitudinalControl', True)
        self._active = self._speed > 0 and v_ego < self._speed and openpilot_longitudinal_control

        if self._active and not was_active:
            # Switching to active state
            self.params.put_nonblocking('LongitudinalPersonality', str(self._mode))
        elif not self._active and was_active:
            # Switching back to inactive state
            if self.old_personality is not None:
              self.params.put_nonblocking('LongitudinalPersonality', self.old_personality)

    def update_loop(self):
        while True:
          current_time = time.time()
          openpilot_longitudinal_control = getattr(self.CP, 'openpilotLongitudinalControl', True)

          if self.v_ego > (40 * CV.KPH_TO_MS) and openpilot_longitudinal_control:
            self.old_personality = self.params.get('LongitudinalPersonality')
            self.last_record_time = current_time

          time.sleep(5)

    def get_personality(self, personality):
        return self._mode if self._active else personality

    def is_active(self):
        return self._active
