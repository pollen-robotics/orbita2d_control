from ._orbita2d import ffi as ffi
from ._orbita2d import lib as lib


class KinematicsModel:
    def __init__(self, ratio_a: float, ratio_b: float) -> None:
        self.ratio_a = ratio_a
        self.ratio_b = ratio_b

    def forward_position(self, angle_a: float, angle_b: float) -> (float, float):
        ring = ffi.new("double *")
        center = ffi.new("double *")

        lib.orbita2d_forward_position(
            self.ratio_a, self.ratio_b, angle_a, angle_b, ring, center
        )

        return ring[0], center[0]

    def forward_velocity(self, velocity_a: float, velocity_b: float) -> (float, float):
        ring_velocity = ffi.new("double *")
        center_velocity = ffi.new("double *")

        lib.orbita2d_forward_velocity(
            self.ratio_a,
            self.ratio_b,
            velocity_a,
            velocity_b,
            ring_velocity,
            center_velocity,
        )

        return ring_velocity[0], center_velocity[0]

    def forward_torque(self, torque_a: float, torque_b: float) -> (float, float):
        ring_torque = ffi.new("double *")
        center_torque = ffi.new("double *")

        lib.orbita2d_forward_torque(
            self.ratio_a,
            self.ratio_b,
            torque_a,
            torque_b,
            ring_torque,
            center_torque,
        )

        return ring_torque[0], center_torque[0]

    def inverse_position(self, ring: float, center: float) -> (float, float):
        angle_a = ffi.new("double *")
        angle_b = ffi.new("double *")

        lib.orbita2d_inverse_position(
            self.ratio_a, self.ratio_b, ring, center, angle_a, angle_b
        )

        return angle_a[0], angle_b[0]

    def inverse_velocity(self, ring_velocity: float, center_velocity: float) -> (float, float):
        velocity_a = ffi.new("double *")
        velocity_b = ffi.new("double *")

        lib.orbita2d_inverse_velocity(
            self.ratio_a,
            self.ratio_b,
            ring_velocity,
            center_velocity,
            velocity_a,
            velocity_b,
        )

        return velocity_a[0], velocity_b[0]

    def inverse_torque(self, ring_torque: float, center_torque: float) -> (float, float):
        torque_a = ffi.new("double *")
        torque_b = ffi.new("double *")

        lib.orbita2d_inverse_torque(
            self.ratio_a,
            self.ratio_b,
            ring_torque,
            center_torque,
            torque_a,
            torque_b,
        )

        return torque_a[0], torque_b[0]


class Orbita2dController:
    def __init__(self, uid: int) -> None:
        self.uid = uid

    @classmethod
    def from_config(cls, configfile: str):
        uid = ffi.new("uint32_t *")
        lib.orbita2d_controller_from_config(configfile.encode(), uid)

        return cls(uid[0])

    def is_torque_on(self) -> bool:
        on = ffi.new("bool *")
        lib.orbita2d_is_torque_on(self.uid, on)
        return on[0]

    def enable_torque(self, reset_target: bool):
        lib.orbita2d_enable_torque(self.uid, reset_target)

    def disable_torque(self):
        lib.orbita2d_disable_torque(self.uid)

    def get_current_orientation(self) -> (float, float):
        angle = ffi.new("double(*)[2]")
        lib.orbita2d_get_current_orientation(self.uid, angle)
        return angle[0][0], angle[0][1]

    def get_current_velocity(self) -> (float, float):
        velocity = ffi.new("double(*)[2]")
        lib.orbita2d_get_current_velocity(self.uid, velocity)
        return velocity[0][0], velocity[0][1]

    def get_current_torque(self) -> (float, float):
        torque = ffi.new("double(*)[2]")
        lib.orbita2d_get_current_torque(self.uid, torque)
        return torque[0][0], torque[0][1]

    def get_target_orientation(self) -> (float, float):
        angle = ffi.new("double(*)[2]")
        lib.orbita2d_get_target_orientation(self.uid, angle)
        return angle[0][0], angle[0][1]

    def set_target_orientation(self, target: (float, float)):
        arr = ffi.new("double(*)[2]", target)
        arr[0][0] = target[0]
        arr[0][1] = target[1]
        lib.orbita2d_set_target_orientation(self.uid, arr)

    def get_raw_motors_velocity_limit(self) -> (float, float):
        limit = ffi.new("double(*)[2]")
        lib.orbita2d_get_raw_motors_velocity_limit(self.uid, limit)
        return limit[0][0], limit[0][1]

    def set_raw_motors_velocity_limit(self, limit: (float, float)):
        arr = ffi.new("double(*)[2]", limit)
        arr[0][0] = limit[0]
        arr[0][1] = limit[1]
        lib.orbita2d_set_raw_motors_velocity_limit(self.uid, arr)

    def get_raw_motors_torque_limit(self) -> (float, float):
        limit = ffi.new("double(*)[2]")
        lib.orbita2d_get_raw_motors_torque_limit(self.uid, limit)
        return limit[0][0], limit[0][1]

    def set_raw_motors_torque_limit(self, limit: (float, float)):
        arr = ffi.new("double(*)[2]", limit)
        arr[0][0] = limit[0]
        arr[0][1] = limit[1]
        lib.orbita2d_set_raw_motors_torque_limit(self.uid, arr)

    def get_raw_motors_pid_gains(
        self,
    ) -> ((float, float, float), (float, float, float)):
        pid = ffi.new("double(*)[6]")
        lib.orbita2d_get_raw_motors_pid_gains(self.uid, pid)
        return (pid[0][0], pid[0][1], pid[0][2]), (pid[0][3], pid[0][4], pid[0][5])

    def set_raw_motors_pid_gains(
        self, pid: ((float, float, float), (float, float, float))
    ):
        arr = ffi.new("double(*)[6]")
        arr[0][0] = pid[0][0]
        arr[0][1] = pid[0][1]
        arr[0][2] = pid[0][2]
        arr[0][3] = pid[1][0]
        arr[0][4] = pid[1][1]
        arr[0][5] = pid[1][2]
        lib.orbita2d_set_raw_motors_pid_gains(self.uid, arr)
