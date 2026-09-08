#!/bin/bash
# Revert amcl_dwa.yaml to its pristine (pre-depth) copy.
B=~/rover_nav/param_backups/amcl_dwa.yaml.orig
[ -f "$B" ] && cp "$B" ~/ugv_ws/src/ugv_main/ugv_nav/param/amcl_dwa.yaml && echo restored || echo "no backup"
