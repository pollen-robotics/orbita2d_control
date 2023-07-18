from ._orbita2d import ffi as ffi
from ._orbita2d import lib as lib


class KinematicsModel:
    """Kinematics model of the Orbita2D actuator."""

    def __init__(self, ratio_a: float, ratio_b: float) -> None:
        """Create a new kinematics model.

        Args:
            ratio_a (float): Ratio of the motor a.
            ratio_b (float): Ratio of the motor b.
        """
        self.ratio_a = ratio_a
        self.ratio_b = ratio_b

    def forward_position(self, angle_a: float, angle_b: float) -> (float, float):
        """Compute the position of the ring and the center given motor position.

        Args:
            angle_a (float): Angle of the motor a.
            angle_b (float): Angle of the motor b.
        Returns:
            (float, float): Position of the ring and the center.
        """
        ring = ffi.new("double *")
        center = ffi.new("double *")

        check_error(
            lib.orbita2d_forward_position(
                self.ratio_a, self.ratio_b, angle_a, angle_b, ring, center
            )
        )

        return ring[0], center[0]

    def forward_velocity(self, velocity_a: float, velocity_b: float) -> (float, float):
        """Compute the velocity of the ring and the center given motor velocity.

        Args:
            velocity_a (float): Velocity of the motor a.
            velocity_b (float): Velocity of the motor b.
        Returns:
            (float, float): Velocity of the ring and the center.
        """
        ring_velocity = ffi.new("double *")
        center_velocity = ffi.new("double *")

        check_error(
            lib.orbita2d_forward_velocity(
                self.ratio_a,
                self.ratio_b,
                velocity_a,
                velocity_b,
                ring_velocity,
                center_velocity,
            )
        )

        return ring_velocity[0], center_velocity[0]

    def forward_torque(self, torque_a: float, torque_b: float) -> (float, float):
        """Compute the torque of the ring and the center given motor torque.

        Args:
            torque_a (float): Torque of the motor a.
            torque_b (float): Torque of the motor b.
        Returns:
            (float, float): Torque of the ring and the center.
        """
        ring_torque = ffi.new("double *")
        center_torque = ffi.new("double *")

        check_error(
            lib.orbita2d_forward_torque(
                self.ratio_a,
                self.ratio_b,
                torque_a,
                torque_b,
                ring_torque,
                center_torque,
            )
        )

        return ring_torque[0], center_torque[0]

    def inverse_position(self, ring: float, center: float) -> (float, float):
        """Compute the motor position given the position of the ring and the center.

        Args:
            ring (float): Position of the ring.
            center (float): Position of the center.
        Returns:
            (float, float): Position of the motor a and the motor b.
        """
        angle_a = ffi.new("double *")
        angle_b = ffi.new("double *")

        check_error(
            lib.orbita2d_inverse_position(
                self.ratio_a, self.ratio_b, ring, center, angle_a, angle_b
            )
        )

        return angle_a[0], angle_b[0]

    def inverse_velocity(
        self, ring_velocity: float, center_velocity: float
    ) -> (float, float):
        """Compute the motor velocity given the velocity of the ring and the center.

        Args:
            ring_velocity (float): Velocity of the ring.
            center_velocity (float): Velocity of the center.
        Returns:
            (float, float): Velocity of the motor a and the motor b.
        """
        velocity_a = ffi.new("double *")
        velocity_b = ffi.new("double *")

        check_error(
            lib.orbita2d_inverse_velocity(
                self.ratio_a,
                self.ratio_b,
                ring_velocity,
                center_velocity,
                velocity_a,
                velocity_b,
            )
        )

        return velocity_a[0], velocity_b[0]

    def inverse_torque(
        self, ring_torque: float, center_torque: float
    ) -> (float, float):
        """Compute the motor torque given the torque of the ring and the center.

        Args:
            ring_torque (float): Torque of the ring.
            center_torque (float): Torque of the center.
        Returns:
            (float, float): Torque of the motor a and the motor b.
        """
        torque_a = ffi.new("double *")
        torque_b = ffi.new("double *")

        check_error(
            lib.orbita2d_inverse_torque(
                self.ratio_a,
                self.ratio_b,
                ring_torque,
                center_torque,
                torque_a,
                torque_b,
            )
        )

        return torque_a[0], torque_b[0]


class Orbita2dController:
    """Controller for the Orbita2d actuator."""

    def __init__(self, uid: int) -> None:
        """You should not call this constructor directly. Use from_config instead."""
        self.uid = uid

    @classmethod
    def from_config(cls, configfile: str):
        """Create a controller from a configuration file."""
        uid = ffi.new("uint32_t *")
        check_error(lib.orbita2d_controller_from_config(configfile.encode(), uid))
        return cls(uid[0])

    def is_torque_on(self) -> bool:
        """Return True if the torque is on, False otherwise."""
        on = ffi.new("bool *")
        check_error(lib.orbita2d_is_torque_on(self.uid, on))
        return on[0]

    def enable_torque(self, reset_target: bool):
        """Enable the torque.

        Args:
            reset_target (bool): If True, the target position is reset to the current position.
        """
        check_error(lib.orbita2d_enable_torque(self.uid, reset_target))

    def disable_torque(self):
        """Disable the torque.""""
        check_error(lib.orbita2d_disable_torque(self.uid))

    def get_current_orientation(self) -> (float, float):
        """Return the current orientation of the ring and the center (in rads)."""
        angle = ffi.new("double(*)[2]")
        check_error(lib.orbita2d_get_current_orientation(self.uid, angle))
        return angle[0][0], angle[0][1]

    def get_current_velocity(self) -> (float, float):
        """Return the current velocity of the ring and the center (in rads/s)."""
        velocity = ffi.new("double(*)[2]")
        check_error(lib.orbita2d_get_current_velocity(self.uid, velocity))
        return velocity[0][0], velocity[0][1]

    def get_current_torque(self) -> (float, float):
        """Return the current torque of the ring and the center (in N.m)."""
        torque = ffi.new("double(*)[2]")
        check_error(lib.orbita2d_get_current_torque(self.uid, torque))
        return torque[0][0], torque[0][1]

    def get_target_orientation(self) -> (float, float):
        """Return the target orientation of the ring and the center (in rads)."""
        angle = ffi.new("double(*)[2]")
        check_error(lib.orbita2d_get_target_orientation(self.uid, angle))
        return angle[0][0], angle[0][1]

    def set_target_orientation(self, target: (float, float)):
        """Set the target orientation of the ring and the center (in rads)."""
        arr = ffi.new("double(*)[2]", target)
        arr[0][0] = target[0]
        arr[0][1] = target[1]
        check_error(lib.orbita2d_set_target_orientation(self.uid, arr))

    def get_raw_motors_velocity_limit(self) -> (float, float):
        """Return the velocity limit of the motors (in rads/s).
        
        Be careful, this is not the velocity limit of the ring and the center but of the motor directly!
        """
        limit = ffi.new("double(*)[2]")
        check_error(lib.orbita2d_get_raw_motors_velocity_limit(self.uid, limit))
        return limit[0][0], limit[0][1]

    def set_raw_motors_velocity_limit(self, limit: (float, float)):
        """Set the velocity limit of the motors (in rads/s).
        
        Be careful, this is not the velocity limit of the ring and the center but of the motor directly!
        """
        arr = ffi.new("double(*)[2]", limit)
        arr[0][0] = limit[0]
        arr[0][1] = limit[1]
        check_error(lib.orbita2d_set_raw_motors_velocity_limit(self.uid, arr))

    def get_raw_motors_torque_limit(self) -> (float, float):
        """Return the torque limit of the motors (in N.m).

        Be careful, this is not the torque limit of the ring and the center but of the motor directly!
        """
        limit = ffi.new("double(*)[2]")
        check_error(lib.orbita2d_get_raw_motors_torque_limit(self.uid, limit))
        return limit[0][0], limit[0][1]

    def set_raw_motors_torque_limit(self, limit: (float, float)):
        """Set the torque limit of the motors (in N.m).

        Be careful, this is not the torque limit of the ring and the center but of the motor directly!
        """
        arr = ffi.new("double(*)[2]", limit)
        arr[0][0] = limit[0]
        arr[0][1] = limit[1]
        check_error(lib.orbita2d_set_raw_motors_torque_limit(self.uid, arr))

    def get_raw_motors_pid_gains(
        self,
    ) -> ((float, float, float), (float, float, float)):
        """Return the PID gains of the motors.
        
        Be careful, this is not the PID gains of the ring and the center but of the motor directly!
        """
        pid = ffi.new("double(*)[6]")
        check_error(lib.orbita2d_get_raw_motors_pid_gains(self.uid, pid))
        return (pid[0][0], pid[0][1], pid[0][2]), (pid[0][3], pid[0][4], pid[0][5])

    def set_raw_motors_pid_gains(
        self, pid: ((float, float, float), (float, float, float))
    ):
        """Set the PID gains of the motors.
        
        Be careful, this is not the PID gains of the ring and the center but of the motor directly!
        """
        arr = ffi.new("double(*)[6]")
        arr[0][0] = pid[0][0]
        arr[0][1] = pid[0][1]
        arr[0][2] = pid[0][2]
        arr[0][3] = pid[1][0]
        arr[0][4] = pid[1][1]
        arr[0][5] = pid[1][2]
        check_error(lib.orbita2d_set_raw_motors_pid_gains(self.uid, arr))


def check_error(error_code: int):
    if error_code != 0:
        raise Exception("Error code: {}".format(error_code))
