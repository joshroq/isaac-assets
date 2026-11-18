# _check_single_oak.sh — refuse to start a SECOND OAK-D camera driver.
#
# SOURCE this near the top of start_oak.sh. The OAK-D is a single USB device; a
# second depthai driver can't claim it, so the launch prints a confusing bare
# "Terminated" right after "Camera ready!" (one of the two containers loses the
# device). Abort up front with a clear message instead.
#
# Matches the launch + container by name (never the "start_oak.sh" wrapper, so no
# pgrep self-match). Override for the rare intentional case with ALLOW_MULTI_OAK=1.
if [ "${ALLOW_MULTI_OAK:-0}" != "1" ]; then
  if pgrep -f "oak_d_lite.launch|oak_container|depthai_ros_driver" >/dev/null 2>&1; then
    echo "[single-oak] ABORT: an OAK-D driver is already running (it owns the USB camera)." >&2
    echo "[single-oak] A second driver can't claim the device (you'd get a bare 'Terminated')." >&2
    echo "[single-oak] Reuse the running one, or stop it first:  ~/rover_nav/stop_all.sh" >&2
    exit 1
  fi
fi