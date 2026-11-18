#!/usr/bin/env bash
# Open a replay USD in the media-remote viewer (experiments/play_replay.py): PLAY/PAUSE plus
# FAST-FORWARD and REWIND that cycle 1x->2x->3x->4x->1x per direction, like a DVD remote.
# Needs a display (run on a machine/session with a GUI). No ROS.
#   experiments/play_replay.sh experiments/runs/.../*_stage_record_sampled.usda
#
# Controls (click the Isaac window first):
#   SPACE play 1x | P pause | ->/L fast-forward(1-4x) | <-/J rewind(1-4x) | HOME/END | ESC quit
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
USD="${1:?usage: play_replay.sh <replay.usda>}"
USD="$(realpath "$USD")"
echo "Opening $USD in the replay remote viewer. Click the window, then use SPACE / arrows."
exec isaacpy "$HERE/play_replay.py" "$USD"
