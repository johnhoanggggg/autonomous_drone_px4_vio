"""Process-level singleton locks for fixed-topic ROS safety nodes."""

import fcntl
import os
from pathlib import Path
import re
import tempfile


class ProcessSingleton:
    """Hold a non-stale OS lock for one role in one ROS domain."""

    def __init__(self, role, *, domain_id=None, lock_directory=None):
        domain = str(
            os.environ.get("ROS_DOMAIN_ID", "0") if domain_id is None else domain_id
        )
        safe_role = re.sub(r"[^A-Za-z0-9_.-]", "_", str(role))
        safe_domain = re.sub(r"[^A-Za-z0-9_.-]", "_", domain)
        directory = Path(lock_directory or tempfile.gettempdir())
        self.path = directory / (
            f"px4_vio_bridge_{safe_role}_ros_domain_{safe_domain}.lock"
        )
        self._file = self.path.open("a+", encoding="ascii")
        try:
            fcntl.flock(self._file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            self._file.seek(0)
            holder = self._file.read().strip() or "unknown"
            self._file.close()
            raise RuntimeError(
                f"duplicate {role} for ROS_DOMAIN_ID={domain}; "
                f"existing holder PID={holder}"
            ) from exc
        self._file.seek(0)
        self._file.truncate()
        self._file.write(str(os.getpid()))
        self._file.flush()

    def close(self):
        if self._file is None:
            return
        fcntl.flock(self._file.fileno(), fcntl.LOCK_UN)
        self._file.close()
        self._file = None

    def __enter__(self):
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback):
        self.close()

