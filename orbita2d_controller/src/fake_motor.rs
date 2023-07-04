use std::f64::{INFINITY, NAN};

use crate::{Orbita2dController, Orbita2dMotorController, PID};

/// Fake motors implementation, only used for testing
struct FakeMotors {
    torque_on: bool,

    current_position: [f64; 2],
    current_velocity: [f64; 2],
    current_torque: [f64; 2],

    target_position: [f64; 2],

    velocity_limit: [f64; 2],
    torque_limit: [f64; 2],
    pid_gains: PID,
}

impl Default for FakeMotors {
    fn default() -> Self {
        Self {
            torque_on: false,

            current_position: [0.0, 0.0],
            current_velocity: [NAN, NAN],
            current_torque: [NAN, NAN],

            target_position: [0.0, 0.0],

            velocity_limit: [INFINITY, INFINITY],
            torque_limit: [INFINITY, INFINITY],
            pid_gains: PID {
                p: NAN,
                i: NAN,
                d: NAN,
            },
        }
    }
}

impl Orbita2dController {
    /// Create a new Orbita2dController with fake motors, only meant for testing.
    ///
    /// This implementation does not communicate with any real motor.
    /// It is not meant to mimic a real motor behavior.
    /// But instead implements the simplest model for testing purposes.
    /// More precisely, the motors current position directly teleports to the target when the torque is on.
    ///
    /// Other registers such as velocity, torque, limits, pid gains are not supported in this version.
    pub fn with_fake_motors() -> Self {
        Self::new(Box::<FakeMotors>::default(), [1.0, 1.0], [0.0, 0.0], None)
    }
}

impl Orbita2dMotorController for FakeMotors {
    fn name(&self) -> &'static str {
        "FakeMotors"
    }

    fn is_torque_on(&mut self) -> crate::Result<bool> {
        Ok(self.torque_on)
    }

    fn set_torque(&mut self, on: bool) -> crate::Result<()> {
        self.torque_on = on;

        if self.torque_on {
            self.current_position = self.target_position;
        }

        Ok(())
    }

    fn get_current_position(&mut self) -> crate::Result<[f64; 2]> {
        Ok(self.current_position)
    }

    fn get_current_velocity(&mut self) -> crate::Result<[f64; 2]> {
        Ok(self.current_velocity)
    }

    fn get_current_torque(&mut self) -> crate::Result<[f64; 2]> {
        Ok(self.current_torque)
    }

    fn get_target_position(&mut self) -> crate::Result<[f64; 2]> {
        Ok(self.target_position)
    }

    fn set_target_position(&mut self, target_position: [f64; 2]) -> crate::Result<()> {
        self.target_position = target_position;

        if self.torque_on {
            self.current_position = target_position;
        }
        Ok(())
    }

    fn get_velocity_limit(&mut self) -> crate::Result<[f64; 2]> {
        Ok(self.velocity_limit)
    }

    fn set_velocity_limit(&mut self, velocity_limit: [f64; 2]) -> crate::Result<()> {
        self.velocity_limit = velocity_limit;
        Ok(())
    }

    fn get_torque_limit(&mut self) -> crate::Result<[f64; 2]> {
        Ok(self.torque_limit)
    }

    fn set_torque_limit(&mut self, torque_limit: [f64; 2]) -> crate::Result<()> {
        self.torque_limit = torque_limit;
        Ok(())
    }

    fn get_pid_gains(&mut self) -> crate::Result<crate::PID> {
        Ok(self.pid_gains)
    }

    fn set_pid_gains(&mut self, pid_gains: crate::PID) -> crate::Result<()> {
        self.pid_gains = pid_gains;
        Ok(())
    }
}
