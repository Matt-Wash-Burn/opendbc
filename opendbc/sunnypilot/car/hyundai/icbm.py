"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

from opendbc.car import DT_CTRL, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.hyundai import hyundaican, hyundaicanfd
from opendbc.car.hyundai.values import HyundaiFlags, Buttons, CANFD_CAR
from opendbc.sunnypilot.car.intelligent_cruise_button_management_interface_base import IntelligentCruiseButtonManagementInterfaceBase

ButtonType = structs.CarState.ButtonEvent.Type
SendButtonState = structs.IntelligentCruiseButtonManagement.SendButtonState

BUTTON_COPIES = 2
ALT_BTN_PERIOD = 0.04   # s between injected frames (~25Hz, one per stock 0x10b frame)
ALT_BTN_COPIES = 4      # copies per injection at a single counter slot (redundancy, no racing)
BUTTON_COPIES_TIME = 7
BUTTON_COPIES_TIME_IMPERIAL = [BUTTON_COPIES_TIME + 3, 70]
BUTTON_COPIES_TIME_METRIC = [BUTTON_COPIES_TIME, 40]

BUTTONS = {
  SendButtonState.increase: Buttons.RES_ACCEL,
  SendButtonState.decrease: Buttons.SET_DECEL,
}


class IntelligentCruiseButtonManagementInterface(IntelligentCruiseButtonManagementInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)

  def create_can_mock_button_messages(self, packer, CS, send_button) -> list[CanData]:
    can_sends = []
    copies_xp = BUTTON_COPIES_TIME_METRIC if CS.is_metric else BUTTON_COPIES_TIME_IMPERIAL
    copies = int(np.interp(BUTTON_COPIES_TIME, copies_xp, [1, BUTTON_COPIES]))

    # send resume at a max freq of 10Hz
    if (self.frame - self.last_button_frame) * DT_CTRL > 0.1:
      # send 25 messages at a time to increases the likelihood of resume being accepted
      can_sends.extend([hyundaican.create_clu11(packer, self.frame, CS.clu11, send_button, self.CP)] * copies)
      if (self.frame - self.last_button_frame) * DT_CTRL >= 0.15:
        self.last_button_frame = self.frame

    return can_sends

  def create_canfd_mock_button_messages(self, packer, CS, CAN, send_button) -> list[CanData]:
    can_sends = []
    if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
      copy = getattr(CS, "cruise_btns_alt_copy", None)
      if copy and (self.frame - self.last_button_frame) * DT_CTRL > ALT_BTN_PERIOD:
        # Mimic a real driver button press. A real press is a *sustained*, monotonic,
        # even-counter stream of btn!=0 frames on 0x10b for the entire press duration
        # (verified in rlog: a manual RES+ held btn=1 across ~1.25s of frames, counter
        # +2 each, and moved the camera's set speed cleanly).
        #
        # The previous implementation burst 12 "press" + 6 "release" frames every 0.2s,
        # each with its own counter (base + 2n). That failed for three compounding
        # reasons (confirmed in rlog 000000e0--22c9c6ecc0: SET- requested for 2.6s
        # straight, camera VSetDis never changed):
        #   1. it self-released inside every burst, so the "press" only existed for ~5ms
        #      of counter-time -- below the camera's button debounce;
        #   2. it raced the counter ~36 counts ahead of the live, still-forwarded stock
        #      0x10b stream, so when the real stream caught up the camera saw a counter
        #      regression and rejected it;
        #   3. the 0.2s gap meant btn!=0 was present <20% of the time.
        #
        # Instead hold the button continuously at the next counter slot of the live stock
        # stream, paced one injection per stock frame. Press *duration* is driven by the
        # ICBM state machine (it holds send_button until v_cruise matches); release is
        # implicit -- once send_button returns to none we stop injecting and the stock
        # btn=0 stream provides the falling edge.
        cnt = (CS.buttons_counter + 2) % 0x100
        for _ in range(ALT_BTN_COPIES):
          can_sends.append(hyundaicanfd.create_buttons_alt_0x10b(packer, self.CP, CAN, copy, cnt, send_button))
        self.last_button_frame = self.frame
    else:
      if (self.frame - self.last_button_frame) * DT_CTRL > 0.2:
        self.button_frame += 1
        button_counter_offset = [1, 1, 0, None][self.button_frame % 4]
        if button_counter_offset is not None:
          for _ in range(20):
            can_sends.append(hyundaicanfd.create_buttons(packer, self.CP, CAN, (CS.buttons_counter + button_counter_offset) % 0xF, send_button))
          self.last_button_frame = self.frame

    return can_sends

  def update(self, CS, CC_SP, packer, frame, last_button_frame, CAN) -> list[CanData]:
    can_sends = []
    self.CC_SP = CC_SP
    self.ICBM = CC_SP.intelligentCruiseButtonManagement
    self.frame = frame
    self.last_button_frame = last_button_frame

    if self.ICBM.sendButton != SendButtonState.none:
      send_button = BUTTONS[self.ICBM.sendButton]

      if self.CP.carFingerprint in CANFD_CAR:
        can_sends.extend(self.create_canfd_mock_button_messages(packer, CS, CAN, send_button))
      else:
        can_sends.extend(self.create_can_mock_button_messages(packer, CS, send_button))

    return can_sends
