import math
from collections import namedtuple

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.volkswagen.values import DBC, VolkswagenFlags

FRONT_RADAR_ADDR = 0x24F  # Strukturen_01
SIDE_RADAR_ADDR = 0x24D   # MEB_Side_Assist_02
NO_OBJECT = 0
RADAR_RATE_HZ = 25.0

# Slot status byte values for MEB_Side_Assist_02
SIDE_STATUS_ACTIVE = 0x80  # real track with current radar measurement
SIDE_STATUS_GHOST = 0x00   # placeholder slot, no live measurement
SIDE_STATUS_EMPTY = 0xff   # slot unused this frame

# Coordinate-frame translation for side radar.
# ID.4 length = 4.58 m. The two corner radars (SWA modules) are mounted at the
# rear corners and report:
#   - Long_Distance: positive forward from the rear-corner mount
#   - Lat_Distance:  positive to the RIGHT of ego centerline
# Front radar (Strukturen_01) reports dRel from the front bumper with +y=LEFT
# (openpilot convention). To expose side-radar tracks in the SAME frame as
# front radar so the two can be fused, we shift dRel by -EGO_LENGTH (rear ->
# front-bumper origin) and negate yRel (flip +y=RIGHT -> +y=LEFT).
EGO_LENGTH = 4.58

LANE_TYPES = ("Same_Lane", "Left_Lane", "Right_Lane")
FRONT_SIGNAL_SETS = tuple(
  (
    f"{prefix}_ObjectID",
    f"{prefix}_Long_Distance",
    f"{prefix}_Lat_Distance",
    f"{prefix}_Rel_Velo",
  )
  for lane in LANE_TYPES
  for idx in (1, 2)
  for prefix in (f"{lane}_0{idx}",)
)

SIDE_SIGNAL_SETS = tuple(
  (
    f"{prefix}_ObjectID",
    f"{prefix}_Status",
    f"{prefix}_Long_Distance",
    f"{prefix}_Lat_Distance",
    f"{prefix}_Zone",
  )
  for zone in ("Right", "Center", "Left")
  for idx in (1, 2)
  for prefix in (f"{zone}_0{idx}",)
)

# Coordinate frame is per the corner radar, not ego — caller must translate
# before mixing with front-radar tracks.
SideRadarTrack = namedtuple("SideRadarTrack", [
  "object_id",
  "zone",
  "d_rel",
  "y_rel",
  "v_rel",
])


def get_radar_can_parser(CP):
  if CP.flags & (VolkswagenFlags.MEB | VolkswagenFlags.MQB_EVO) and not (CP.flags & VolkswagenFlags.DISABLE_RADAR):
    messages = [
      ("Strukturen_01", 25),
      ("MEB_Side_Assist_02", 25),
    ]
  else:
    return None

  return CANParser(DBC[CP.carFingerprint][Bus.radar], messages, 2)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)

    self.updated_messages: set[int] = set()
    self.trigger_msg: int = FRONT_RADAR_ADDR
    self._track_id_counter: int = 0

    self.radar_off_can: bool = CP.radarUnavailable
    self.rcp: CANParser | None = get_radar_can_parser(CP)

    self._pts = self.pts

    # Side-radar state — exposed for downstream BSM / extended-radar consumers.
    # Not merged into ret.points: 0x24D coordinates are per corner-radar
    # (range -6..+12 m), distinct from Strukturen_01's ego-frame range.
    self.side_points: list[SideRadarTrack] = []
    self._side_prev_d_rel: dict[int, float] = {}

  def update(self, can_strings):
    """Entry-point called by the vehicle loop every CAN tick."""
    if self.radar_off_can or self.rcp is None:
      return super().update(None)

    vls = self.rcp.update(can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None

    radar_data = self._process_radar_frame()
    self.updated_messages.clear()
    return radar_data

  def _process_radar_frame(self):
    ret = structs.RadarData()

    if self.rcp is None:
      return ret

    if not self.rcp.can_valid:
      ret.errors.canError = True
      return ret

    msg = self.rcp.vl["Strukturen_01"]
    get = msg.__getitem__

    active_objects: dict[int, tuple[float, float, float]] = {}
    for obj_id_sig, long_sig, lat_sig, vel_sig in FRONT_SIGNAL_SETS:
      obj_id = get(obj_id_sig)
      if obj_id == NO_OBJECT:
        continue

      if obj_id in active_objects:
        ret.errors.canError = True
        return ret

      active_objects[obj_id] = (
        get(long_sig),  # dRel
        get(lat_sig),   # yRel
        get(vel_sig),   # vRel
      )

    for obj_id, (d_rel, y_rel, v_rel) in active_objects.items():
      if obj_id not in self._pts:
        pt = structs.RadarData.RadarPoint()
        pt.trackId = self._track_id_counter
        self._track_id_counter += 1
        self._pts[obj_id] = pt
      else:
        pt = self._pts[obj_id]

      pt.measured = True
      pt.dRel = d_rel
      pt.yRel = y_rel
      pt.vRel = v_rel
      pt.aRel = math.nan
      pt.yvRel = math.nan

    inactive_ids = self._pts.keys() - active_objects.keys()
    for obj_id in inactive_ids:
      self._pts.pop(obj_id, None)

    ret.points = list(self._pts.values())

    self._update_side_radar()

    return ret

  def _update_side_radar(self):
    """Parse 0x24D side-radar tracks into self.side_points.

    Velocity is estimated from per-ObjectID dDist/dt at the radar's 25 Hz rate
    because no rel-velocity field has been decoded in 0x24D yet.
    """
    side_msg = self.rcp.vl["MEB_Side_Assist_02"]
    side_get = side_msg.__getitem__

    new_points: list[SideRadarTrack] = []
    seen_ids: set[int] = set()

    for obj_id_sig, status_sig, long_sig, lat_sig, zone_sig in SIDE_SIGNAL_SETS:
      if int(side_get(status_sig)) != SIDE_STATUS_ACTIVE:
        continue

      obj_id = int(side_get(obj_id_sig))
      if obj_id in seen_ids:
        continue
      seen_ids.add(obj_id)

      # Raw values are in the corner-radar frame; translate to ego frame
      # (front-bumper origin, +y=LEFT) to match Strukturen_01 / openpilot.
      d_rel = float(side_get(long_sig)) - EGO_LENGTH
      y_rel = -float(side_get(lat_sig))
      zone = int(side_get(zone_sig))

      prev_d = self._side_prev_d_rel.get(obj_id)
      v_rel = (d_rel - prev_d) * RADAR_RATE_HZ if prev_d is not None else 0.0
      self._side_prev_d_rel[obj_id] = d_rel

      new_points.append(SideRadarTrack(
        object_id=obj_id,
        zone=zone,
        d_rel=d_rel,
        y_rel=y_rel,
        v_rel=v_rel,
      ))

    stale_ids = self._side_prev_d_rel.keys() - seen_ids
    for sid in stale_ids:
      self._side_prev_d_rel.pop(sid, None)

    self.side_points = new_points
