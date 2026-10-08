#!/usr/bin/env bash
# Can be launched from any directory; does not launch/control the robot driver.
rwhe_capture_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)" || exit 1
cd "$rwhe_capture_root" || exit 1
source scripts/ros2_env.sh || exit 1
exec /usr/bin/python3 -m hardware.rb5_gopro --config hardware/rb5.local.json capture "$@"
