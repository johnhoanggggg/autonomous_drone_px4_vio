"""Where flight bags get written.

Every launch file that records names its bag `<root>/<mode>_<UTC stamp>`, so two
runs never collide and nothing has to be hand-numbered. The root used to be
`Path.cwd() / "flight_logs"`, which put bags under `ros2_ws/flight_logs` or the
repo root depending on which directory the launch happened to be typed from —
that is how the tree ended up with two flight_logs directories. It is now pinned
to `<repo>/ros2_ws/flight_logs`, which is where the rest of the tooling looks
(`scripts/fetch_ulog.py` writes ULogs there, `scripts/analyze_flight.py` reads
bags from there). Set `PX4_VIO_FLIGHT_LOGS` to override.
"""

import os
from datetime import datetime, timezone
from pathlib import Path


def _repo_root():
    # Resolve through the symlink-install shim to the source tree, then walk up
    # to the checkout. Falls back to cwd for a non-symlink install, which is the
    # old behaviour rather than a wrong absolute path.
    for parent in Path(__file__).resolve().parents:
        if (parent / ".git").exists():
            return parent
    return Path.cwd()


def flight_log_root():
    """Directory bags are written into. Created if missing."""
    override = os.environ.get("PX4_VIO_FLIGHT_LOGS")
    root = Path(override).expanduser() if override else _repo_root() / "ros2_ws" / "flight_logs"
    root.mkdir(parents=True, exist_ok=True)
    return root


def timestamped_bag(mode):
    """Default `bag_output` for `mode`, e.g. `.../offboard_global_20260823T044325Z`."""
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return str(flight_log_root() / f"{mode}_{stamp}")
