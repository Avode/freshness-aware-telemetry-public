"""Explainable freshness diagnostics; no simulator truth or injected-fault flags."""
from collections import deque
import math


class HealthTracker:
    # Wall-time arrival and advancing capture time are both required.
    LIMITS = {'imu': 1., 'lidar': 1., 'position': 1., 'encoders': 1., 'camera': 2.}

    def __init__(self):
        self.samples = {name: deque(maxlen=1200) for name in self.LIMITS}
        self.last_stamp = {}
        self.sigma = None
        self.motion = deque(maxlen=300)
        self.motion_disagreement = None
        self.motion_stamp = None

    def compare_motion(self, stamp, localized, odometry):
        """Compare four-second relative motion, independent of absolute frame origins."""
        if self.motion and stamp <= self.motion[-1][0]: return
        self.motion.append((stamp, localized, odometry))
        while len(self.motion)>1 and self.motion[1][0] <= stamp-4.: self.motion.popleft()
        if stamp-self.motion[0][0] < 3.8: return
        _, old_local, old_odom = self.motion[0]
        def relative(old, new):
            dx, dy = new[0]-old[0], new[1]-old[1]
            c, s = math.cos(old[2]), math.sin(old[2])
            return c*dx+s*dy, -s*dx+c*dy
        self.motion_disagreement = math.dist(relative(old_local, localized), relative(old_odom, odometry))
        self.motion_stamp = stamp

    def observe(self, name, stamp, wall):
        if not math.isfinite(stamp) or stamp <= self.last_stamp.get(name, -1.):
            return
        self.last_stamp[name] = stamp
        self.samples[name].append((stamp, wall))

    def snapshot(self, sim, wall):
        sensors, issues = {}, []
        for name, limit in self.LIMITS.items():
            values = self.samples[name]
            capture_age = max(0., sim-values[-1][0]) if values else None
            arrival_age = max(0., wall-values[-1][1]) if values else None
            stale = not values or max(capture_age, arrival_age) > limit
            recent = [v for v in values if wall-v[1] <= 5.]
            rate = ((len(recent)-1)/(recent[-1][0]-recent[0][0])
                    if len(recent) > 1 and recent[-1][0] > recent[0][0] else 0.)
            sensors[name] = dict(status='STALE' if stale else 'OK', capture_age_s=capture_age,
                                 arrival_age_s=arrival_age, rate_hz=round(rate, 1), limit_s=limit)
            if stale:
                issues.append(dict(code=name+'_stale', severity='warning' if name == 'camera' else 'critical',
                                   message=f'{name.capitalize()} capture is missing or no longer advancing'))
        if self.sigma is not None and self.sigma > 1.5:
            issues.append(dict(code='position_uncertainty', severity='warning',
                               message='Reported position standard deviation exceeds 1.5 m; accuracy is unverified'))
        comparison = self.motion_disagreement if self.motion_stamp is not None and sim-self.motion_stamp<1. else None
        if comparison is not None and comparison > 1.:
            issues.append(dict(code='position_motion_disagreement', severity='warning',
                               message='Localized motion differs from continuous odometry by over 1 m in four seconds'))
        level = 'CRITICAL' if any(i['severity'] == 'critical' for i in issues) else 'DEGRADED' if issues else 'HEALTHY'
        return dict(stamp=sim, status=level, sensors=sensors, issues=issues,
                    reported_position_sigma_m=self.sigma,
                    motion_disagreement_m=comparison,
                    confidence_note='Estimator covariance is not a calibrated accuracy guarantee')
