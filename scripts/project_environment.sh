#!/usr/bin/env bash
# Sourced only by the child build/run scripts; never edits shell startup files.
if (( EUID == 0 )); then
  echo 'Run this private project as the normal user, without sudo.' >&2
  exit 2
fi
mkdir -p "$root/runtime/ros/log" "$root/runtime/cache" "$root/runtime/tmp" \
  "$root/runtime/colcon" "$root/runtime/ccache"
export ROS_HOME="$root/runtime/ros"
export ROS_LOG_DIR="$root/runtime/ros/log"
export XDG_CACHE_HOME="$root/runtime/cache"
export TMPDIR="$root/runtime/tmp"
export COLCON_HOME="$root/runtime/colcon"
export CCACHE_DIR="$root/runtime/ccache"
export PYTHONDONTWRITEBYTECODE=1
