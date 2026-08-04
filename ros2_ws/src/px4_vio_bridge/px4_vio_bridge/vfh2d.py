"""2D Vector Field Histogram (VFH+) steering — the algorithm, and nothing else.

No ROS, no numpy, no vehicle: this module is a pure function of range data, so
it can be unit-tested on a laptop and reasoned about without flying. The ROS
plumbing lives in `vfh_obstacles.py` (sensor -> samples) and the two nodes
(`vfh_monitor.py`, `offboard_vfh.py`).

Conventions, chosen to match the rest of the project:

- A **sample** is `(range_m, bearing_rad)` in the BODY frame: bearing 0 is
  straight ahead, positive is to the vehicle's RIGHT. That is the same sign
  convention as PX4's NED yaw (heading increases turning right), so an absolute
  NED heading toward a chosen direction is simply `wrap_pi(heading + bearing)`.
- Sector 0 is centred on bearing -pi and sectors run clockwise (increasing
  bearing). All sector arithmetic is circular.

The pipeline per update is the VFH+ one:

1. **Polar density.** Each sample adds `1 - range/max_range` to its sector, so a
   close return counts for ~1 and one at the sensor's horizon counts for ~0.
   Per-sector point counts and minimum ranges are kept alongside.
2. **Smoothing** over `smoothing` sectors, because a stereo cloud is speckled and
   a single-sector spike is usually noise, not a pole.
3. **Binary histogram** with hysteresis: a free sector blocks at `tau_high`, a
   blocked sector clears only below `tau_low`. Without hysteresis a density
   hovering on the threshold makes the chosen direction chatter, and the vehicle
   weaves.
4. **Enlargement.** Every blocked sector along the path is widened by
   `asin((robot_radius + safety_margin) / range)`, which is what turns a point
   obstacle into something a vehicle of finite width cannot fly through. A return
   closer than `robot_radius + safety_margin` blocks a full +/-90 deg. When the
   goal distance is known, obstacles beyond it use finite endpoint-clearance
   geometry instead of incorrectly blocking an infinite continuation.
5. **Field-of-view mask.** Sectors whose centres lie outside `max_steer` are
   marked blocked. They are unobserved, not free space.
6. **Candidate directions** from the free openings (VFH+): a wide opening offers
   its two borders pulled inward plus the target direction itself when that
   already-enlarged sector is free; a narrow opening offers only its centre.
   Candidates outside `max_steer` are discarded.
7. **Cost** `mu_target * d(c, target) + mu_heading * d(c, 0) + mu_previous *
   d(c, previous)`, minimised. `mu_target` must dominate or the vehicle never
   commits to the goal; the other two are what keep the path smooth.

The one thing that is NOT classic VFH and matters on this airframe: a camera
sees roughly 70 deg, so sectors outside the field of view carry no information.
Unknown is not free — `max_steer` (default 35 deg) exists to keep every chosen
direction inside the region the sensor actually observed.
"""
import math
from dataclasses import dataclass, field


def wrap_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def angle_between(a, b):
    """Absolute smallest angle between two bearings, in [0, pi]."""
    return abs(wrap_pi(a - b))


def relative_bearing_enu(yaw_enu, deast, dnorth):
    """Body-frame bearing (0 ahead, + right) of an ENU displacement.

    ENU angles increase counter-clockwise while a body bearing increases to the
    right, so the two run in opposite directions and the ENU form subtracts.
    """
    return wrap_pi(yaw_enu - math.atan2(dnorth, deast))


def relative_bearing_ned(heading, dnorth, deast):
    """Body-frame bearing (0 ahead, + right) of a NED displacement.

    NED headings already increase to the right, so this one does NOT mirror the
    ENU version — mixing them up puts every obstacle on the wrong side of the
    vehicle. An absolute NED heading toward a body bearing is
    `wrap_pi(heading + bearing)`.
    """
    return wrap_pi(math.atan2(deast, dnorth) - heading)


@dataclass
class VfhConfig:
    """Tunables. The defaults are sized for a 0.3 m indoor hover on this drone."""

    sectors: int = 72                              # 5 deg each
    min_range: float = 0.25                        # below this the stereo cloud is junk
    max_range: float = 2.0                         # beyond this it is too sparse to trust
    min_points: int = 4                            # a sector needs this many returns to count
    tau_high: float = 6.0                          # free -> blocked
    tau_low: float = 3.0                           # blocked -> free (hysteresis)
    smoothing: int = 3                             # moving-average window, odd
    robot_radius: float = 0.30                     # prop tip to prop tip / 2
    safety_margin: float = 0.10
    max_steer: float = math.radians(35.0)          # camera FOV limit, NOT a comfort setting
    wide_valley: float = math.radians(40.0)        # opening wider than this is "wide"
    mu_target: float = 5.0
    mu_heading: float = 2.0
    mu_previous: float = 2.0

    def __post_init__(self):
        if self.sectors < 8:
            raise ValueError("sectors must be at least 8")
        if self.min_range < 0.0 or self.max_range <= self.min_range:
            raise ValueError("need 0 <= min_range < max_range")
        if self.tau_low > self.tau_high:
            raise ValueError("tau_low must not exceed tau_high")
        if self.smoothing < 1 or self.smoothing % 2 == 0:
            raise ValueError("smoothing must be a positive odd number of sectors")
        if not 0.0 < self.max_steer <= math.pi:
            raise ValueError("max_steer must be in (0, pi]")

    @property
    def sector_width(self):
        return 2.0 * math.pi / self.sectors

    @property
    def enlargement_radius(self):
        return self.robot_radius + self.safety_margin


@dataclass
class VfhResult:
    """One planning cycle. Everything needed to explain the decision in a bag."""

    direction: float = None            # chosen body-relative bearing, None when blocked
    blocked: bool = True
    reason: str = "no data"
    density: list = field(default_factory=list)
    # `binary` is the final traversability mask used to choose headings. It
    # includes the camera-FOV mask, so blind sectors are deliberately 1 even
    # when no obstacle was measured there.
    binary: list = field(default_factory=list)
    # Physical VFH obstruction after vehicle-radius enlargement, but before
    # the camera-FOV mask. Telemetry uses this to show remembered obstacles
    # outside the steering cone without painting every blind ray red.
    obstacle_binary: list = field(default_factory=list)
    counts: list = field(default_factory=list)
    sector_range: list = field(default_factory=list)  # min range per sector, inf when empty
    candidates: list = field(default_factory=list)
    cost: float = None
    nearest_range: float = math.inf
    nearest_bearing: float = None
    sample_count: int = 0

    def min_range_in_cone(self, half_angle):
        """Closest return within +/-half_angle of the nose.

        This is deliberately taken from the sector minima rather than the chosen
        direction: an emergency stop must not depend on the planner having
        agreed that anything is in the way.
        """
        best = math.inf
        n = len(self.sector_range)
        if n == 0:
            return best
        width = 2.0 * math.pi / n
        for i, r in enumerate(self.sector_range):
            if r < best and abs(wrap_pi(-math.pi + (i + 0.5) * width)) <= half_angle:
                best = r
        return best


def sector_center(index, sectors):
    """Bearing at the centre of a sector; sector 0 is centred on -pi."""
    width = 2.0 * math.pi / sectors
    return wrap_pi(-math.pi + (index + 0.5) * width)


def sector_index(bearing, sectors):
    width = 2.0 * math.pi / sectors
    return int((wrap_pi(bearing) + math.pi) // width) % sectors


class Vfh2D:
    """Stateful only in the hysteresis band; `update()` is otherwise pure."""

    def __init__(self, config=None):
        self.config = config or VfhConfig()
        self._binary = [0] * self.config.sectors

    def reset(self):
        self._binary = [0] * self.config.sectors

    # --- steps -------------------------------------------------------------
    def _accumulate(self, samples):
        cfg = self.config
        density = [0.0] * cfg.sectors
        counts = [0] * cfg.sectors
        ranges = [math.inf] * cfg.sectors
        nearest_range = math.inf
        nearest_bearing = None
        used = 0

        for sample in samples:
            r, bearing = float(sample[0]), float(sample[1])
            if not (math.isfinite(r) and math.isfinite(bearing)):
                continue
            if r < cfg.min_range or r > cfg.max_range:
                continue
            i = sector_index(bearing, cfg.sectors)
            density[i] += 1.0 - r / cfg.max_range
            counts[i] += 1
            if r < ranges[i]:
                ranges[i] = r
            if r < nearest_range:
                nearest_range = r
                nearest_bearing = wrap_pi(bearing)
            used += 1

        return density, counts, ranges, nearest_range, nearest_bearing, used

    def _smooth(self, density):
        """Triangular (VFH+) smoothing: weights l+1-|i|, normalised by (l+1)^2.

        Deliberately NOT a flat moving average. A flat window divides an
        isolated obstacle's density by the whole window width, and a real
        measurement — 36 stereo returns off a wall edge at 2.5 m — then reads as
        free because its empty neighbours average it away. The triangular kernel
        gives the sector itself half the weight, which still suppresses
        single-point speckle but cannot erase a solid narrow obstacle.
        """
        cfg = self.config
        if cfg.smoothing == 1:
            return list(density)
        half = cfg.smoothing // 2
        n = cfg.sectors
        norm = float((half + 1) ** 2)
        out = []
        for i in range(n):
            total = 0.0
            for k in range(-half, half + 1):
                total += (half + 1 - abs(k)) * density[(i + k) % n]
            out.append(total / norm)
        return out

    def _threshold(self, density, counts):
        """Binary histogram with hysteresis against the previous cycle."""
        cfg = self.config
        binary = []
        for i in range(cfg.sectors):
            if counts[i] < cfg.min_points:
                # Two stray VIO points are not a wall. Sparse sectors are free
                # regardless of how close those points claim to be.
                binary.append(0)
                continue
            if self._binary[i]:
                binary.append(1 if density[i] > cfg.tau_low else 0)
            else:
                binary.append(1 if density[i] > cfg.tau_high else 0)
        return binary

    def _enlarge(self, binary, ranges, target_distance=None):
        """Widen blocked sectors for collision with the path to the goal.

        Without a target distance this is the classic infinite-ray VFH
        enlargement. With one, the commanded path is a finite segment: an
        obstacle beyond its endpoint only blocks headings whose endpoint comes
        within the vehicle-plus-safety radius.
        """
        cfg = self.config
        n = cfg.sectors
        finite_path = (
            target_distance is not None
            and math.isfinite(target_distance)
            and target_distance >= 0.0
        )
        path_length = float(target_distance) if finite_path else math.inf
        out = [0] * n if finite_path else list(binary)
        for i in range(n):
            if not binary[i]:
                continue
            r = ranges[i]
            if finite_path and path_length <= 0.0:
                continue
            if finite_path and math.isfinite(r) and r > path_length:
                # The obstacle is beyond the endpoint. If its closest possible
                # distance to that endpoint exceeds the safety envelope, it
                # cannot collide with this finite path at any heading.
                if r - path_length > cfg.enlargement_radius + 1e-9:
                    continue
                cosine = (
                    r * r
                    + path_length * path_length
                    - cfg.enlargement_radius * cfg.enlargement_radius
                ) / (2.0 * r * path_length)
                gamma = math.acos(max(-1.0, min(1.0, cosine)))
            elif not math.isfinite(r) or r <= cfg.enlargement_radius:
                gamma = math.pi / 2.0
            else:
                gamma = math.asin(min(1.0, cfg.enlargement_radius / r))
            # Block a neighbour when its CENTRE falls inside the enlarged cone,
            # rather than rounding the cone up to whole sectors. Rounding up
            # inflates every obstacle by up to half a sector on each side, which
            # at typical ranges is enough to close a gap the vehicle actually
            # fits through (a 1.4 m doorway at 2.2 m, measured). The half-sector
            # of cone this leaves untested is ~0.1 m at 2 m — well inside
            # safety_margin, which is what that margin is for.
            spread = int(math.ceil(gamma / cfg.sector_width))
            center = sector_center(i, n)
            for k in range(-spread, spread + 1):
                j = (i + k) % n
                # The epsilon makes an exact tie resolve as blocked: a sector
                # centre landing precisely on the cone edge is decided by
                # floating-point noise otherwise, and "blocked" is the side to
                # be wrong on.
                if angle_between(sector_center(j, n), center) <= gamma + 1e-9:
                    out[j] = 1
        return out

    def _mask_outside_fov(self, binary):
        """Make blind sectors non-traversable before finding openings."""
        cfg = self.config
        if cfg.max_steer >= math.pi:
            return list(binary)
        return [
            int(
                blocked
                or angle_between(sector_center(i, cfg.sectors), 0.0)
                > cfg.max_steer
            )
            for i, blocked in enumerate(binary)
        ]

    def _openings(self, binary):
        """Maximal circular runs of free sectors, as (start_index, count)."""
        n = len(binary)
        if all(binary):
            return []
        if not any(binary):
            return [(0, n)]
        # Start scanning at the beginning of a blocked run so no free run is
        # split across the array boundary.
        start = next(i for i in range(n) if binary[i] and not binary[(i - 1) % n])
        openings = []
        j = 0
        while j < n:
            if binary[(start + j) % n]:
                j += 1
                continue
            length = 0
            while length < n and not binary[(start + j + length) % n]:
                length += 1
            openings.append(((start + j) % n, length))
            j += length
        return openings

    def _candidates(self, binary, target_bearing):
        cfg = self.config
        margin = cfg.wide_valley / 2.0
        candidates = []

        for start, length in self._openings(binary):
            # Bearings of the opening's outer edges (not sector centres): the
            # edge is where the neighbouring blocked sector begins.
            edge_right = sector_center(start, cfg.sectors) - cfg.sector_width / 2.0
            width = length * cfg.sector_width
            edge_left = edge_right + width

            # A camera-masked opening is a linear interval in the forward
            # cone. Clip sector edges to the exact optical FOV, since the FOV
            # does not necessarily align with sector boundaries. Unlike a
            # physical obstacle, an optical edge gets no VFH border margin;
            # an off-camera goal may therefore command the last observed
            # heading, but never a heading beyond it.
            if cfg.max_steer < math.pi:
                edge_right = max(edge_right, -cfg.max_steer)
                edge_left = min(edge_left, cfg.max_steer)
                width = edge_left - edge_right
                if width <= 0.0:
                    continue

                right_is_fov = edge_right <= -cfg.max_steer + 1e-9
                left_is_fov = edge_left >= cfg.max_steer - 1e-9
                safe_right = edge_right if right_is_fov else edge_right + margin
                safe_left = edge_left if left_is_fov else edge_left - margin

                if width >= cfg.wide_valley and safe_right <= safe_left:
                    candidates.extend((safe_right, safe_left))
                    # Enlargement already includes the vehicle and safety
                    # radius. Keep the target itself whenever its sector is
                    # free; applying the border margin a second time rejects
                    # geometrically safe headings near one side of a valley.
                    candidates.append(
                        max(edge_right, min(edge_left, target_bearing))
                    )
                else:
                    candidates.append(edge_right + width / 2.0)
                continue

            if width >= cfg.wide_valley:
                candidates.append(wrap_pi(edge_right + margin))
                candidates.append(wrap_pi(edge_left - margin))
                # Enlargement already accounts for the whole vehicle width;
                # the target only has to fall inside this free opening.
                offset = wrap_pi(target_bearing - edge_right)
                if offset < 0.0:
                    offset += 2.0 * math.pi
                if 0.0 <= offset <= width:
                    candidates.append(wrap_pi(target_bearing))
            else:
                candidates.append(wrap_pi(edge_right + width / 2.0))

        # De-duplicate candidates. The camera branch constructs every bearing
        # inside the observed cone. The clamp is retained for the full-circle
        # configuration, where max_steer == pi and it is normally a no-op.
        unique = []
        for c in candidates:
            if abs(c) > cfg.max_steer:
                c = math.copysign(cfg.max_steer, c)
            if cfg.max_steer < math.pi and abs(c) >= cfg.max_steer:
                # FOV limits can coincide with a half-open sector boundary.
                # Move one representable value inward so sector lookup cannot
                # select the first blind sector on just one side of the cone.
                c = math.nextafter(c, 0.0)
            if binary[sector_index(c, cfg.sectors)]:
                continue
            if not any(angle_between(c, u) < 1e-9 for u in unique):
                unique.append(c)
        return unique

    def _cost(self, candidate, target_bearing, previous_direction):
        cfg = self.config
        cost = cfg.mu_target * angle_between(candidate, target_bearing)
        cost += cfg.mu_heading * angle_between(candidate, 0.0)
        if previous_direction is not None:
            cost += cfg.mu_previous * angle_between(candidate, previous_direction)
        return cost

    # --- entry point -------------------------------------------------------
    def update(
        self,
        samples,
        target_bearing=0.0,
        previous_direction=None,
        target_distance=None,
    ):
        """Plan one steering direction from a batch of body-frame samples.

        `target_bearing` is where the goal lies relative to the nose; pass 0.0
        to just keep going forward. `previous_direction` is the last committed
        direction, also body-relative (the caller must re-express it after a
        heading change, which is why it is an argument and not state).
        `target_distance` makes collision checking stop at the goal; omit it
        for classic infinite-ray VFH behavior.
        """
        cfg = self.config
        target_bearing = wrap_pi(target_bearing)

        density, counts, ranges, nearest_range, nearest_bearing, used = (
            self._accumulate(samples)
        )
        smoothed = self._smooth(density)
        binary = self._threshold(smoothed, counts)
        self._binary = list(binary)
        enlarged = self._enlarge(binary, ranges, target_distance)
        traversable = self._mask_outside_fov(enlarged)
        candidates = self._candidates(traversable, target_bearing)

        result = VfhResult(
            density=smoothed,
            binary=traversable,
            obstacle_binary=enlarged,
            counts=counts,
            sector_range=ranges,
            candidates=candidates,
            nearest_range=nearest_range,
            nearest_bearing=nearest_bearing,
            sample_count=used,
        )

        if not candidates:
            result.blocked = True
            result.reason = (
                f"no free direction within +/-{math.degrees(cfg.max_steer):.0f} deg"
            )
            return result

        best = min(
            candidates,
            key=lambda c: self._cost(c, target_bearing, previous_direction),
        )
        result.direction = best
        result.cost = self._cost(best, target_bearing, previous_direction)
        result.blocked = False
        result.reason = "clear"
        return result


def histogram_bar(result, width=None):
    """One-line ASCII rendering of the binary histogram, nose in the middle.

    Purely for the terminal and the log — a blocked sector is `#`, a free one
    with returns is `-`, an empty one is `.`, and the chosen direction is `^`.
    """
    binary = result.binary
    n = len(binary)
    if n == 0:
        return ""
    # No rotation needed: sector 0 is centred on -pi, so the array already runs
    # from behind-left through the nose in the middle to behind-right.
    chars = []
    for i in range(n):
        if binary[i]:
            chars.append("#")
        elif result.counts and result.counts[i]:
            chars.append("-")
        else:
            chars.append(".")
    if result.direction is not None:
        chars[sector_index(result.direction, n)] = "^"
    text = "".join(chars)
    if width is not None and width < len(text):
        start = (len(text) - width) // 2
        text = text[start:start + width]
    return text
