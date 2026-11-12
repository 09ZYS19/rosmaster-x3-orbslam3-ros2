#!/usr/bin/env bash
# Camera-only RGB-D SLAM. Does not start drivers or publish motion commands.
set -eo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [[ ${1:-} == --help ]]; then
  echo 'Usage: bash scripts/start_real_slam.sh [ROS arguments]'
  echo 'Requires registered RGB-D and CameraInfo matching the RGB image.'
  echo 'Defaults: /camera topics, approximate sync <=20 ms, 16UC1 millimetres, no viewer.'
  echo 'Inherits the current ROS_DOMAIN_ID and network configuration.'
  echo 'Saves in results/real/1, 2, 3, ... and evaluates after shutdown completes.'
  exit 0
fi
source /opt/ros/humble/setup.bash
source "$root/ros2_ws/install-real/setup.bash"
source "$root/scripts/project_environment.sh"
export LD_LIBRARY_PATH="$root/local/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export OMP_NUM_THREADS=${OMP_NUM_THREADS:-1}
binary="$root/ros2_ws/install-real/orbslam3_x3_ros2/lib/orbslam3_x3_ros2/rgbd"
[[ -x $binary ]] || { echo 'First run: bash scripts/build_real_slam.sh' >&2; exit 1; }
for helper in run_real_slam.py evaluate_real_run.py; do
  [[ -f "$root/scripts/$helper" ]] || { echo "Missing scripts/$helper; copy the run update first." >&2; exit 1; }
done
exec python3 "$root/scripts/run_real_slam.py" "$root" "$binary" "$@"
