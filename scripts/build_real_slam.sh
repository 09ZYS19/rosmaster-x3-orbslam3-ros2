#!/usr/bin/env bash
# First build on the target computer. No package installation or sudo.
set -eo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [[ ${1:-} == --help ]]; then
  echo 'Usage: bash scripts/build_real_slam.sh [--check] (BUILD_JOBS=2 by default)'
  echo 'Builds bundled Pangolin, DBoW2, ORB-SLAM3 and the ROS package locally.'
  echo '--check only configures the dependency check; does not compile or install.'
  exit 0
fi
[[ $# -eq 0 || ( $# -eq 1 && $1 == --check ) ]] || { echo 'Unknown argument; use --help' >&2; exit 2; }
jobs=${BUILD_JOBS:-2}
[[ $jobs =~ ^[1-9][0-9]*$ ]] || { echo 'BUILD_JOBS must be a positive integer' >&2; exit 2; }
source /opt/ros/humble/setup.bash
source "$root/scripts/project_environment.sh"
export CMAKE_PREFIX_PATH="$root/local${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$root/local/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export CMAKE_BUILD_PARALLEL_LEVEL="$jobs"
# The private ROS adapter uses native conversion, not the system cv_bridge ABI.
# Prefer the existing system development config, including Jetson's non-multiarch path.
multiarch=$(g++ -print-multiarch)
opencv_dir=${ORB_REAL_OPENCV_DIR:-}
if [[ -z $opencv_dir ]]; then
  for candidate in "/usr/lib/$multiarch/cmake/opencv4" /usr/lib/cmake/opencv4 \
                   /usr/local/lib/cmake/opencv4 /usr/local/share/OpenCV; do
    if [[ -f "$candidate/OpenCVConfig.cmake" ]]; then opencv_dir=$candidate; break; fi
  done
fi
[[ -n $opencv_dir && -f "$opencv_dir/OpenCVConfig.cmake" ]] || {
  echo 'Missing OpenCV development configuration; stop without changing the shared system.' >&2
  exit 2
}
echo "Using OpenCV config: $opencv_dir/OpenCVConfig.cmake"
opencv_args=("-DOpenCV_DIR=$opencv_dir")
cmake_isolation=(-DCMAKE_EXPORT_NO_PACKAGE_REGISTRY=ON -DCMAKE_EXPORT_PACKAGE_REGISTRY=OFF)
cmake -S "$root/config/real_build_check" -B "$root/build-real/dependency-check" \
  "${opencv_args[@]}" "${cmake_isolation[@]}" -DCMAKE_BUILD_TYPE=Release
if [[ ${1:-} == --check ]]; then exit 0; fi
cmake -S "$root/third_party/Pangolin" -B "$root/build-real/pangolin" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$root/local" \
  "${cmake_isolation[@]}" \
  -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF -DBUILD_TESTS=OFF \
  -DBUILD_PANGOLIN_PYTHON=OFF -DBUILD_PANGOLIN_FFMPEG=OFF \
  -DBUILD_PANGOLIN_LIBDC1394=OFF -DBUILD_PANGOLIN_V4L=OFF \
  -DBUILD_PANGOLIN_REALSENSE=OFF -DBUILD_PANGOLIN_REALSENSE2=OFF \
  -DBUILD_PANGOLIN_OPENNI=OFF -DBUILD_PANGOLIN_OPENNI2=OFF
cmake --build "$root/build-real/pangolin" --parallel "$jobs"
cmake --install "$root/build-real/pangolin"
orb="$root/src/ORB_SLAM3"
cmake -S "$orb/Thirdparty/DBoW2" -B "$orb/Thirdparty/DBoW2/build" \
  -DCMAKE_BUILD_TYPE=Release "${opencv_args[@]}" "${cmake_isolation[@]}"
cmake --build "$orb/Thirdparty/DBoW2/build" --parallel "$jobs"
if [[ ! -f "$orb/Vocabulary/ORBvoc.txt" ]]; then
  tar -xzf "$orb/Vocabulary/ORBvoc.txt.tar.gz" -C "$orb/Vocabulary" ORBvoc.txt
fi
cmake -S "$orb" -B "$orb/build" -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_DISABLE_FIND_PACKAGE_realsense2=ON "${opencv_args[@]}" "${cmake_isolation[@]}"
cmake --build "$orb/build" --target ORB_SLAM3 --parallel "$jobs"
cd "$root/ros2_ws"
colcon --log-base log-real build --build-base build-real --install-base install-real \
  --packages-select orbslam3_x3_ros2 --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release "-DCMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH" \
  "${opencv_args[@]}" "${cmake_isolation[@]}"
echo "Build complete. Real-camera entry: $root/scripts/start_real_slam.sh"
