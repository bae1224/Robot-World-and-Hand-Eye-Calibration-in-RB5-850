# Source this file from the shell used for ROS recording (do not execute it).
_rwhe_ros_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
_rwhe_ros_prefix="$_rwhe_ros_root/.runtime/ros2_ws/install-feedback-v2"
if [[ ! -f "$_rwhe_ros_prefix/share/rbpodo_hardware/local_setup.bash" ]]; then
    echo 'ROS driver overlay is missing. Run scripts/build_rbpodo_ros2.py first.' >&2
    unset _rwhe_ros_root _rwhe_ros_prefix
    return 1
fi
export CMAKE_PREFIX_PATH="$_rwhe_ros_root/.runtime/rbpodo_cpp_install:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$_rwhe_ros_root/.runtime/rbpodo_cpp_install/lib:${LD_LIBRARY_PATH:-}"
for _rwhe_ros_pkg in rbpodo_msgs rbpodo_hardware rbpodo_description rbpodo_bringup; do
    # CMake installs per-package hooks; a colcon workspace setup file is not required.
    AMENT_CURRENT_PREFIX="$_rwhe_ros_prefix" source "$_rwhe_ros_prefix/share/$_rwhe_ros_pkg/local_setup.bash"
done
unset _rwhe_ros_root _rwhe_ros_prefix _rwhe_ros_pkg
