"""What demo_drive and leg_executor share about driving motivation/jetson's robot_link as a child process:
the lines it prints, and the `--stop` flush before a plan."""
import logging
import re
import subprocess

import drive_config as cfg

PROGRESS_RE = re.compile(r"primitive (\d+)/(\d+)")     # MissionRunner's per-primitive line
COMPLETE_MARK = "plan complete"
FAILED_MARK = "plan failed"
LINE_BUFFERED = ("stdbuf", "-oL")                       # robot_link's stdout is a pipe: flush per line

logger = logging.getLogger(__name__)


def flush_link(port, runlog):
    """robot_link --stop: clears a partial line in the ESP32's RX buffer before a plan (drive_config
    ROBOT_LINK_STOP_TIMEOUT_S). A failure is logged, not fatal: the plan then reports its own error."""
    cmd = [cfg.ROBOT_LINK_BIN, "--port", port, "--stop"]
    try:
        done = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True,
                              timeout=cfg.ROBOT_LINK_STOP_TIMEOUT_S)
    except (OSError, subprocess.TimeoutExpired) as exc:
        logger.warning("robot_link --stop failed before the plan: %s", exc)
        runlog.event("link_flush", ok=False, detail=str(exc))
        return False
    ok = done.returncode == 0
    if not ok:
        logger.warning("robot_link --stop exited %d before the plan: %s", done.returncode, done.stdout.strip())
    runlog.event("link_flush", ok=ok, exit_code=done.returncode)
    return ok
