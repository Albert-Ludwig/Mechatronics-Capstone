"""ROS-independent checks for a completed mission and measured stationary dwell."""
import math
from .geometry import angle_error


def check_action_result(status, result, waypoint_action=False):
    """ROS action SUCCEEDED=4 alone is insufficient: check missed waypoints too."""
    if status != 4:
        raise RuntimeError(f'Navigation action status={status} (expected SUCCEEDED=4)')
    if getattr(result, 'error_code', 0) != 0:
        raise RuntimeError(f'Nav2 error {result.error_code}: {getattr(result, "error_msg", "")}')
    if waypoint_action and list(getattr(result, 'missed_waypoints', [])):
        raise RuntimeError(f'Nav2 reported missed waypoints: {result.missed_waypoints}')


class DwellTracker:
    def __init__(self, poses):
        self.poses = poses
        self.start = [None] * len(poses)
        self.previous = [None] * len(poses)
        self.longest = [0.] * len(poses)

    def observe(self, stamp, x, y, yaw, linear, angular, target_index):
        # Only the active inspection target counts; passing another shelf does not.
        for index, pose in enumerate(self.poses):
            valid = (index == target_index and
                     math.hypot(x-pose['x'], y-pose['y']) <= .23 and
                     angle_error(yaw, pose['yaw']) <= .25 and
                     abs(linear) <= .02 and abs(angular) <= .04)
            previous = self.previous[index]
            # Missing TF/odom samples must not count as stationary observation.
            if not valid:
                self.start[index] = None
                self.previous[index] = None
                continue
            if previous is None or stamp < previous or stamp - previous > .6:
                self.start[index] = stamp
            self.previous[index] = stamp
            self.longest[index] = max(self.longest[index], stamp-self.start[index])

    def results(self):
        return [dict(name=pose['name'], observed_stationary_seconds=round(seconds, 3),
                     dwell_observed=seconds >= 4.5)
                for pose, seconds in zip(self.poses, self.longest)]
