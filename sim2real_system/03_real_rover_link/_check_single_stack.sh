# _check_single_stack.sh — refuse to start a SECOND base-owning nav stack.
#
# SOURCE this near the top of any script that starts ugv_bringup (the base driver
# that opens /dev/ttyTHS1). If the serial port is already held, a second opener
# produces garbled/interleaved ESP32 feedback ("multiple access on port" ->
# JSON decode errors -> ugv_bringup KeyError 'T' -> odom/TF stop -> Nav2
# controller_server heartbeat failure -> the whole stack collapses). Rather than
# let that happen confusingly, abort with a clear message.
#
# Override for the rare intentional case with ALLOW_MULTI_STACK=1.
_ugv_serial="${UGV_SERIAL_PORT:-/dev/ttyTHS1}"
if [ "${ALLOW_MULTI_STACK:-0}" != "1" ]; then
  if fuser "${_ugv_serial}" >/dev/null 2>&1 || pgrep -f "ugv_bringup " >/dev/null 2>&1; then
    echo "[single-stack] ABORT: a nav stack is already running (something holds ${_ugv_serial})." >&2
    echo "[single-stack] Running a second base bringup double-opens the serial port and shreds" >&2
    echo "[single-stack] the ESP32 feedback. Stop the current stack first:" >&2
    echo "[single-stack]     ~/rover_nav/stop_all.sh" >&2
    echo "[single-stack] (only ONE of start_bringup / start_lidar_nav / start_depth_nav / ... at a time)." >&2
    echo "[single-stack] To view a stack that is ALREADY running, don't restart it — just open the" >&2
    echo "[single-stack] desktop viewer:  rover_comms/desktop/view_rover.sh <stack>" >&2
    exit 1
  fi
fi
unset _ugv_serial