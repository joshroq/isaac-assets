# Sourced helper: provides `rsudo` for the install scripts, which run detached
# (setsid/nohup) and so cannot prompt interactively.
#
# Unattended (detached builds): export your rover's sudo password first, e.g.
#     export ROVER_SUDO_PW='...'      # do NOT commit this
# Interactive: leave ROVER_SUDO_PW unset and either run `sudo -v` first (to cache
# credentials) or let the first rsudo call prompt you on the terminal.
#
# NOTE: the old scripts hardcoded the password inline; that has been removed so
# this bundle is safe to publish and portable to a rover with a different password.
rsudo() {
  if [ -n "${ROVER_SUDO_PW:-}" ]; then
    echo "$ROVER_SUDO_PW" | sudo -S "$@"
  else
    sudo "$@"
  fi
}
