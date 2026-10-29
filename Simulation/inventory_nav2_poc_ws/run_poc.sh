#!/usr/bin/env bash
# Run from the workspace after dependencies are installed. No sudo and no .bashrc edits.
set -e
# VS Code's Snap terminal can inject GTK/GIO modules linked against its older
# glibc. Let system ROS/Gazebo applications discover system modules instead.
for task_module_path in GTK_PATH GIO_MODULE_DIR; do
  if [[ "${!task_module_path:-}" == *"/snap/"* ]]; then
    unset "$task_module_path"
    printf 'Ignoring Snap-injected %s for ROS/Gazebo.\n' "$task_module_path"
  fi
done
task_distro="${ROS_DISTRO:-lyrical}"
case "$task_distro" in
  jazzy|lyrical) ;;
  *) printf 'Unsupported ROS_DISTRO=%s; use Jazzy or Lyrical.\n' "$task_distro" >&2; exit 1 ;;
esac
if [ ! -f "/opt/ros/$task_distro/setup.bash" ]; then
  printf 'Missing /opt/ros/%s/setup.bash\n' "$task_distro" >&2
  exit 1
fi
task_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "/opt/ros/$task_distro/setup.bash"
cd "$task_workspace"
# Restrict discovery to this workspace; nested repository copies may contain
# packages with the same names.
colcon build --base-paths src --symlink-install --packages-up-to inventory_inspection_poc
source install/local_setup.bash
ros2 run inventory_inspection_poc check_environment
exec ros2 launch inventory_inspection_poc inspection.launch.py "$@"
