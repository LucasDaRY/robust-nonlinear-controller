"""Wheel odometry, with a short IMU correction while the wheels are slipping.

Nominal motion is taken from the wheel pose. A shove that the encoders miss
shows up as specific force or yaw rate the wheels did not produce. For that
short window the pose is integrated from the IMU, then a zero-velocity update
hands the pose back to the wheels. The ground is treated as a plane: z is not
integrated.
"""

import math


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class PlanarDisturbanceOdom:
    def __init__(
        self,
        accel_threshold=0.45,
        yaw_rate_threshold=0.4,
        settle_time=0.4,
        max_event_time=4.0,
        debounce_time=0.05,
        filter_tau=0.08,
    ):
        self.accel_threshold = accel_threshold
        self.yaw_rate_threshold = yaw_rate_threshold
        self.settle_time = settle_time
        self.max_event_time = max_event_time
        self.debounce_time = debounce_time
        self.filter_tau = filter_tau

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.mode = 'tracking'
        self.forced_exit = False

        self._have_wheel = False
        self._last_wx = 0.0
        self._last_wy = 0.0
        self._last_wyaw = 0.0
        self._last_wheel_t = None
        self._wheel_v = 0.0
        self._wheel_w = 0.0
        self._wheel_a = 0.0

        self._a_forward = 0.0
        self._a_lateral = 0.0
        self._high_time = 0.0
        self._low_time = 0.0
        self._event_time = 0.0
        self._settle_time = 0.0
        self._last_imu_t = None
        self._armed = False

    @property
    def disturbance(self):
        return self.mode != 'tracking'

    def wheel(self, x, y, yaw, linear, angular, stamp):
        if not self._have_wheel:
            self.x = x
            self.y = y
            self.yaw = yaw
            self._last_wx = x
            self._last_wy = y
            self._last_wyaw = yaw
            self._last_wheel_t = stamp
            self._wheel_v = linear
            self._wheel_w = angular
            self._have_wheel = True
            return

        dt = None if self._last_wheel_t is None else stamp - self._last_wheel_t
        if dt is not None and 1e-4 < dt < 0.5:
            raw_a = (linear - self._wheel_v) / dt
            alpha = dt / (self.filter_tau + dt)
            self._wheel_a += alpha * (raw_a - self._wheel_a)

        if self.mode == 'tracking':
            self.x += x - self._last_wx
            self.y += y - self._last_wy
            self.yaw = wrap(self.yaw + wrap(yaw - self._last_wyaw))

        self._last_wx = x
        self._last_wy = y
        self._last_wyaw = yaw
        self._last_wheel_t = stamp
        self._wheel_v = linear
        self._wheel_w = angular

    def imu(self, accel_xy, yaw_rate, stamp):
        if not self._have_wheel:
            return
        if self._last_imu_t is None:
            self._last_imu_t = stamp
            return
        dt = stamp - self._last_imu_t
        self._last_imu_t = stamp
        if dt <= 1e-4 or dt > 0.5:
            return

        ax, ay, _vertical = accel_xy
        yaw = self.yaw
        forward = math.cos(yaw) * ax + math.sin(yaw) * ay
        lateral = -math.sin(yaw) * ax + math.cos(yaw) * ay
        # A diff-drive produces lateral acceleration v * yaw_rate. Anything else is slip.
        expected_lateral = self._wheel_v * self._wheel_w
        alpha = dt / (self.filter_tau + dt)
        self._a_forward += alpha * (forward - self._a_forward)
        self._a_lateral += alpha * ((lateral - expected_lateral) - self._a_lateral)

        tripped = (
            abs(self._a_forward - self._wheel_a) > self.accel_threshold
            or abs(self._a_lateral) > self.accel_threshold
            or abs(yaw_rate - self._wheel_w) > self.yaw_rate_threshold
        )
        if tripped:
            if self.mode == 'tracking' and self._high_time == 0.0:
                self._arm_shadow()
            self._high_time += dt
            self._low_time = 0.0
        else:
            self._low_time += dt
            self._high_time = 0.0
            if self.mode == 'tracking':
                self._armed = False

        if self.mode == 'tracking':
            if self._armed:
                self._step_shadow(ax, ay, yaw_rate, dt)
                if self._high_time >= self.debounce_time:
                    self._commit_shadow()
            return

        self.vx += ax * dt
        self.vy += ay * dt
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.yaw = wrap(self.yaw + yaw_rate * dt)
        self._event_time += dt

        if self._event_time >= self.max_event_time:
            self._enter_tracking(forced=True)
            return

        if self.mode == 'disturbance' and self._low_time >= self.debounce_time:
            self.mode = 'settling'
            self._settle_time = 0.0
        elif self.mode == 'settling':
            if tripped:
                self.mode = 'disturbance'
                self._settle_time = 0.0
            else:
                self._settle_time += dt
                if self._settle_time >= self.settle_time:
                    self._enter_tracking(forced=False)

    def _arm_shadow(self):
        self._armed = True
        self._sx = self.x
        self._sy = self.y
        self._syaw = self.yaw
        self._svx = self._wheel_v * math.cos(self.yaw)
        self._svy = self._wheel_v * math.sin(self.yaw)

    def _step_shadow(self, ax, ay, yaw_rate, dt):
        self._svx += ax * dt
        self._svy += ay * dt
        self._sx += self._svx * dt
        self._sy += self._svy * dt
        self._syaw = wrap(self._syaw + yaw_rate * dt)

    def _commit_shadow(self):
        self.x = self._sx
        self.y = self._sy
        self.yaw = self._syaw
        self.vx = self._svx
        self.vy = self._svy
        self.mode = 'disturbance'
        self._event_time = self._high_time
        self._settle_time = 0.0
        self._armed = False
        self.forced_exit = False

    def _enter_tracking(self, forced):
        self.mode = 'tracking'
        self.vx = 0.0
        self.vy = 0.0
        self._high_time = 0.0
        self._low_time = 0.0
        self._event_time = 0.0
        self._settle_time = 0.0
        self.forced_exit = forced
