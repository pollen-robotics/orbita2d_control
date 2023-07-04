from . import _orbita2d


class KinematicsModel:
    def __init__(self, ratio_a, ratio_b) -> None:
        self.ratio_a = ratio_a
        self.ratio_b = ratio_b

    def forward_position(self, angle_a, angle_b):
        roll = _orbita2d.ffi.new("double *")
        pitch = _orbita2d.ffi.new("double *")

        _orbita2d.lib.orbita2d_forward_position(
            self.ratio_a, self.ratio_b, angle_a, angle_b, roll, pitch
        )

        return roll[0], pitch[0]

    def forward_velocity(self, velocity_a, velocity_b):
        roll_velocity = _orbita2d.ffi.new("double *")
        pitch_velocity = _orbita2d.ffi.new("double *")

        _orbita2d.lib.orbita2d_forward_velocity(
            self.ratio_a,
            self.ratio_b,
            velocity_a,
            velocity_b,
            roll_velocity,
            pitch_velocity,
        )

        return roll_velocity[0], pitch_velocity[0]

    def forward_torque(self, torque_a, torque_b):
        roll_torque = _orbita2d.ffi.new("double *")
        pitch_torque = _orbita2d.ffi.new("double *")

        _orbita2d.lib.orbita2d_forward_torque(
            self.ratio_a,
            self.ratio_b,
            torque_a,
            torque_b,
            roll_torque,
            pitch_torque,
        )

        return roll_torque[0], pitch_torque[0]

    def inverse_position(self, roll, pitch):
        angle_a = _orbita2d.ffi.new("double *")
        angle_b = _orbita2d.ffi.new("double *")

        _orbita2d.lib.orbita2d_inverse_position(
            self.ratio_a, self.ratio_b, roll, pitch, angle_a, angle_b
        )

        return angle_a[0], angle_b[0]

    def inverse_velocity(self, roll_velocity, pitch_velocity):
        velocity_a = _orbita2d.ffi.new("double *")
        velocity_b = _orbita2d.ffi.new("double *")

        _orbita2d.lib.orbita2d_inverse_velocity(
            self.ratio_a,
            self.ratio_b,
            roll_velocity,
            pitch_velocity,
            velocity_a,
            velocity_b,
        )

        return velocity_a[0], velocity_b[0]

    def inverse_torque(self, roll_torque, pitch_torque):
        torque_a = _orbita2d.ffi.new("double *")
        torque_b = _orbita2d.ffi.new("double *")

        _orbita2d.lib.orbita2d_inverse_torque(
            self.ratio_a,
            self.ratio_b,
            roll_torque,
            pitch_torque,
            torque_a,
            torque_b,
        )

        return torque_a[0], torque_b[0]


class Orbita2dController:
    def __init__(self, uid: int) -> None:
        self.uid = uid

    @classmethod
    def with_flipsky_serial(
        cls,
        serial_ports: tuple[str, str],
        ids: tuple[int, int],
        offsets: tuple[float, float],
        ratios: tuple[float, float],
        limits: tuple[tuple[float, float], tuple[float, float]],
    ) -> "Orbita2dController":
        uid = _orbita2d.ffi.new("int *")

        _orbita2d.lib.orbita2d_controller_with_flipsky_serial(
            serial_ports[0],
            serial_ports[1],
            ids[0],
            ids[1],
            offsets[0],
            offsets[1],
            ratios[0],
            ratios[1],
            limits[0][0],
            limits[0][1],
            limits[1][0],
            limits[1][1],
            uid
        )

        return cls(uid[0])

    def is_torque_on(self) -> bool:
        on = _orbita2d.ffi.new("bool *")
        _orbita2d.lib.orbita2d_is_torque_on(self.uid, on)
        return on[0]

    def enable_torque(self, reset_target: bool = True):
        _orbita2d.lib.orbita2d_enable_torque(self.uid, reset_target)

    def disable_torque(self):
        _orbita2d.lib.orbita2d_disable_torque(self.uid)

    def get_current_orientation(self) -> tuple[float, float]:
        orientation = _orbita2d.ffi.new("double [2]")
        _orbita2d.lib.orbita2d_get_current_orientation(self.uid, orientation)
        return orientation

    def get_current_velocity(self) -> tuple[float, float]:
        velocity = _orbita2d.ffi.new("double [2]")
        _orbita2d.lib.orbita2d_get_current_velocity(self.uid, velocity)
        return velocity

    def get_current_torque(self) -> tuple[float, float]:
        torque = _orbita2d.ffi.new("double [2]")
        _orbita2d.lib.orbita2d_get_current_torque(self.uid, torque)
        return torque

    def get_target_orientation(self) -> tuple[float, float]:
        target = _orbita2d.ffi.new("double [2]")
        _orbita2d.lib.orbita2d_get_target_orientation(self.uid, target)
        return target

    def set_target_orientation(self, target: tuple[float, float]):
        _orbita2d.lib.orbita2d_set_target_orientation(self.uid, target)

    def get_velocity_limit(self) -> tuple[float, float]:
        velocity_limit = _orbita2d.ffi.new("double [2]")
        _orbita2d.lib.orbita2d_get_velocity_limit(self.uid, velocity_limit)
        return velocity_limit

    def set_velocity_limit(self, limit: tuple[float, float]):
        _orbita2d.lib.orbita2d_set_velocity_limit(self.uid, limit)

    def get_torque_limit(self) -> tuple[float, float]:
        torque_limit = _orbita2d.ffi.new("double [2]")
        _orbita2d.lib.orbita2d_get_torque_limit(self.uid, torque_limit)
        return torque_limit

    def set_torque_limit(self, limit: tuple[float, float]):
        _orbita2d.lib.orbita2d_set_torque_limit(self.uid, limit)

    def get_pid_gains(self) -> tuple[float, float, float]:
        kp = _orbita2d.ffi.new("double *")
        ki = _orbita2d.ffi.new("double *")
        kd = _orbita2d.ffi.new("double *")
        _orbita2d.lib.orbita2d_get_pid_gains(self.uid, kp, ki, kd)
        return kp[0], ki[0], kd[0]

    def set_pid_gains(self, gains: tuple[float, float, float]):
        _orbita2d.lib.orbita2d_set_pid_gains(self.uid, gains[0], gains[1], gains[2])
