#!/usr/bin/env bash
# Enable the D455 depth pointcloud after the camera node starts.
#
# WHY THIS EXISTS: on the arm64 ros-jazzy-realsense2-camera build, the
# pointcloud filter params are mangled to `pointcloud__neon_.*`, and the
# rs_launch `pointcloud.enable:=true` arg does NOT reach them -- so the cloud
# never turns on from the launch file alone. We also run the camera
# DEPTH-ONLY (color off), which means the cloud needs allow_no_texture_points
# or it publishes zero points. Neither param survives a relaunch, so this
# script re-applies both each boot, once the camera node is alive.
#
# Fired automatically from sensors.launch.py (when enable_camera:=true);
# safe to run by hand too:
#   ros2 run cart_bringup enable_d455_pointcloud
set -u

NODE="/camera/camera"
TRIES=30                 # ~60 s; USB enumeration of the D455 can be slow

for i in $(seq 1 "${TRIES}"); do
  if ros2 node list 2>/dev/null | grep -qx "${NODE}"; then
    if ros2 param set "${NODE}" pointcloud__neon_.allow_no_texture_points true >/dev/null 2>&1 \
       && ros2 param set "${NODE}" pointcloud__neon_.enable true >/dev/null 2>&1; then
      echo "[enable_d455_pointcloud] pointcloud enabled on ${NODE}"
      exit 0
    fi
  fi
  sleep 2
done

echo "[enable_d455_pointcloud] ERROR: ${NODE} not ready or param set failed after ~$((TRIES * 2))s" >&2
exit 1
