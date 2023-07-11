//! Orbita2d controller
//!
//! This crate provides a controller for the Orbita2d actuator.
//!
//! # Overview
//!
//! ## Control
//! - [x] Torque ON/OFF
//! - [x] Read the current 2D orientation (position/velocity/torque)
//! - [x] Set the desired 2D orientation in radians
//! - [x] Extra controller parameters (velocity limit, torque limit, PID gains)
//!
//! ## Communication
//! - [x] Flipsky Serial communication
//! - [ ] EtherCAT communication
//!
//! ## Usage
//! ```no_run
//! use orbita2d_controller::Orbita2dController;
//!
//! let mut orbita2d = Orbita2dController::with_flipsky_serial(
//!     ("/dev/ttyUSB0", "/dev/ttyUSB1"),
//!     (30, 31),
//!     [0.0, 0.0],
//!     [1.0, 1.0],
//!     None,
//!     false,
//! ).expect("Failed to initialize Orbita2d controller");
//!
//! let orientation = orbita2d.get_current_orientation().expect("Communication Error");
//! println!("Current 2d orientation: {:?}", orientation);
//!
//! ```

use std::fmt::Debug;

use orbita2d_kinematics::Orbita2dKinematicsModel;

/// Result generic wrapper using `std::error::Error` trait
pub type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

mod coherency;
use coherency::CoherentResult;
mod fake_motor;
mod flipsky_serial;

#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
/// PID gains wrapper
pub struct PID {
    /// Propotional gain
    pub p: f64,
    /// Integral gain
    pub i: f64,
    /// Derivative gain
    pub d: f64,
}

/// Orbita2d controller main interface
pub struct Orbita2dController {
    inner: Box<dyn Orbita2dMotorController>,
    kinematics: Orbita2dKinematicsModel,

    motors_offset: [f64; 2],
    orientation_limits: Option<[AngleLimit; 2]>,
}

impl Debug for Orbita2dController {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Orbita2dController")
            .field("inner", &self.inner.name())
            .field("kinematics", &self.kinematics)
            .field("motors_offset", &self.motors_offset)
            .field("orientation_limits", &self.orientation_limits)
            .finish()
    }
}

#[derive(Debug)]
/// Angle limit wrapper
pub struct AngleLimit {
    /// lower limit in radians
    pub min: f64,
    /// upper limit in radians
    pub max: f64,
}

impl Orbita2dController {
    fn new(
        motors_controller: Box<dyn Orbita2dMotorController>,
        motors_ratio: [f64; 2],
        motors_offset: [f64; 2],
        orientation_limits: Option<[AngleLimit; 2]>,
    ) -> Self {
        Self {
            inner: motors_controller,
            kinematics: Orbita2dKinematicsModel::new(motors_ratio[0], motors_ratio[1]),
            motors_offset,
            orientation_limits,
        }
    }

    /// Check if the torque is ON or OFF.
    pub fn is_torque_on(&mut self) -> Result<bool> {
        self.inner.is_torque_on().coherent()
    }
    /// Enable the torque.
    ///
    /// # Arguments
    /// * `reset_target` - If true, the target orientation is reset to the current orientation.
    pub fn enable_torque(&mut self, reset_target: bool) -> Result<()> {
        if reset_target {
            let current_pos = self.get_current_orientation()?;
            self.set_target_orientation(current_pos)?;
        }
        self.set_torque(true)?;
        Ok(())
    }
    /// Disable the torque.
    pub fn disable_torque(&mut self) -> Result<()> {
        self.set_torque(false)
    }
    fn set_torque(&mut self, on: bool) -> Result<()> {
        self.inner.set_torque([on, on])
    }

    /// Read the current orientation (in radians)
    pub fn get_current_orientation(&mut self) -> Result<[f64; 2]> {
        let pos = self.inner.get_current_position()?;

        let pos = [
            pos[0] - self.motors_offset[0],
            pos[1] - self.motors_offset[1],
        ];

        Ok(self.kinematics.compute_forward_kinematics(pos))
    }
    /// Read the current velocity (in radians/s)
    pub fn get_current_velocity(&mut self) -> Result<[f64; 2]> {
        self.inner.get_current_velocity()
    }
    /// Read the current torque (in Nm)
    pub fn get_current_torque(&mut self) -> Result<[f64; 2]> {
        self.inner.get_current_torque()
    }

    /// Get the desired orientation (in radians)
    pub fn get_target_orientation(&mut self) -> Result<[f64; 2]> {
        let pos = self.inner.get_target_position()?;

        let pos = [
            pos[0] - self.motors_offset[0],
            pos[1] - self.motors_offset[1],
        ];

        Ok(self.kinematics.compute_forward_kinematics(pos))
    }
    /// Set the desired orientation (in radians)
    pub fn set_target_orientation(&mut self, target_orientation: [f64; 2]) -> Result<()> {
        let target_orientation = match &self.orientation_limits {
            Some(limits) => [
                target_orientation[0].clamp(limits[0].min, limits[0].max),
                target_orientation[1].clamp(limits[1].min, limits[1].max),
            ],
            None => target_orientation,
        };

        let ik = self
            .kinematics
            .compute_inverse_kinematics(target_orientation);

        let pos = [ik[0] + self.motors_offset[0], ik[1] + self.motors_offset[1]];

        self.inner.set_target_position(pos)
    }

    /// Get the velocity limit (in radians/s)
    pub fn get_velocity_limit(&mut self) -> Result<[f64; 2]> {
        self.inner.get_velocity_limit()
    }
    /// Set the velocity limit (in radians/s)
    pub fn set_velocity_limit(&mut self, velocity_limit: [f64; 2]) -> Result<()> {
        self.inner.set_velocity_limit(velocity_limit)
    }
    /// Get the torque limit (in Nm)
    pub fn get_torque_limit(&mut self) -> Result<[f64; 2]> {
        self.inner.get_torque_limit()
    }
    /// Set the torque limit (in Nm)
    pub fn set_torque_limit(&mut self, torque_limit: [f64; 2]) -> Result<()> {
        self.inner.set_torque_limit(torque_limit)
    }
    /// Get the PID gains
    pub fn get_pid_gains(&mut self) -> Result<PID> {
        self.inner.get_pid_gains().coherent()
    }
    /// Set the PID gains
    pub fn set_pid_gains(&mut self, pid_gains: PID) -> Result<()> {
        self.inner.set_pid_gains([pid_gains, pid_gains])
    }
}

/// Low-level motors controller abstraction for an Orbita2d controller
pub trait Orbita2dMotorController {
    /// Get the name of the motors controller (used only for Debug)
    fn name(&self) -> &'static str;

    /// Check if the torque is ON or OFF
    fn is_torque_on(&mut self) -> Result<[bool; 2]>;
    /// Enable/Disable the torque
    ///
    /// _Caution: You should guarantee that both motors are always in the same state!_
    fn set_torque(&mut self, on: [bool; 2]) -> Result<()>;
    /// Read the current position (in radians) of each motor
    fn get_current_position(&mut self) -> Result<[f64; 2]>;

    /// Read the current velocity (in radians/s) of each motor
    fn get_current_velocity(&mut self) -> Result<[f64; 2]>;
    /// Read the current torque (in Nm) of each motor
    fn get_current_torque(&mut self) -> Result<[f64; 2]>;
    /// Read the target position (in radians) of each motor
    fn get_target_position(&mut self) -> Result<[f64; 2]>;
    /// Set the target position (in radians) for each motor
    fn set_target_position(&mut self, target_position: [f64; 2]) -> Result<()>;

    /// Get the velocity limit (in radians/s) of each motor
    fn get_velocity_limit(&mut self) -> Result<[f64; 2]>;
    /// Set the velocity limit (in radians/s) for each motor
    fn set_velocity_limit(&mut self, velocity_limit: [f64; 2]) -> Result<()>;
    /// Get the torque limit (in Nm) of each motor
    fn get_torque_limit(&mut self) -> Result<[f64; 2]>;
    /// Set the torque limit (in Nm) for each motor
    fn set_torque_limit(&mut self, torque_limit: [f64; 2]) -> Result<()>;
    /// Get the `PID` gains of each motor
    fn get_pid_gains(&mut self) -> Result<[PID; 2]>;
    /// Set the `PID` gains for each motor
    fn set_pid_gains(&mut self, pid_gains: [PID; 2]) -> Result<()>;
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use super::*;
    use crate::Orbita2dController;
    use rand::Rng;

    #[test]
    fn set_target_orientation() {
        let mut rng = rand::thread_rng();
        let orientation = [rng.gen_range(-PI..PI), rng.gen_range(-PI..PI)];

        let mut fake_orbita = Orbita2dController::with_fake_motors();

        fake_orbita.set_target_orientation(orientation).unwrap();

        let current_target = fake_orbita.get_target_orientation().unwrap();

        assert!((current_target[0] - orientation[0]).abs() < 1e-6);
        assert!((current_target[1] - orientation[1]).abs() < 1e-6);
    }

    #[test]
    fn set_target_orientation_with_motors_offset() {
        let mut rng = rand::thread_rng();
        let orientation = [rng.gen_range(-PI..PI), rng.gen_range(-PI..PI)];

        let mut fake_orbita = Orbita2dController::with_fake_motors();
        fake_orbita.motors_offset = [rng.gen_range(-PI..PI), rng.gen_range(-PI..PI)];

        fake_orbita.set_target_orientation(orientation).unwrap();

        let current_target = fake_orbita.get_target_orientation().unwrap();

        assert!((current_target[0] - orientation[0]).abs() < 1e-6);
        assert!((current_target[1] - orientation[1]).abs() < 1e-6);
    }

    #[test]
    #[should_panic]
    fn test_bad_orientation_limits() {
        let mut rng = rand::thread_rng();
        let orientation = [rng.gen_range(-PI..PI), rng.gen_range(-PI..PI)];

        let mut fake_orbita = Orbita2dController::with_fake_motors();
        fake_orbita.motors_offset = [rng.gen_range(-PI..PI), rng.gen_range(-PI..PI)];

        fake_orbita.orientation_limits = Some([
            AngleLimit {
                min: 0.1,
                max: -0.1,
            },
            AngleLimit {
                min: 0.1,
                max: -0.1,
            },
        ]);
        // limits are min>max and should panic
        fake_orbita.set_target_orientation(orientation).unwrap();
    }

    #[test]
    fn set_target_orientation_with_orientation_limits() {
        let mut rng = rand::thread_rng();
        let orientation = [rng.gen_range(-PI..PI), rng.gen_range(-PI..PI)];

        let mut fake_orbita = Orbita2dController::with_fake_motors();
        fake_orbita.motors_offset = [rng.gen_range(-PI..PI), rng.gen_range(-PI..PI)];

        fake_orbita.orientation_limits = Some([
            AngleLimit {
                min: orientation[0] - 0.1,
                max: orientation[0] + 0.1,
            },
            AngleLimit {
                min: orientation[1] - 0.1,
                max: orientation[1] + 0.1,
            },
        ]);
        // orientation is within limits
        fake_orbita.set_target_orientation(orientation).unwrap();

        let current_target = fake_orbita.get_target_orientation().unwrap();

        assert!((current_target[0] - orientation[0]).abs() < 1e-6);
        assert!((current_target[1] - orientation[1]).abs() < 1e-6);

        fake_orbita.orientation_limits = Some([
            AngleLimit {
                min: orientation[0],
                max: orientation[0],
            },
            AngleLimit {
                min: orientation[1],
                max: orientation[1],
            },
        ]);
        // orientation is exactly at the limits
        fake_orbita.set_target_orientation(orientation).unwrap();

        let current_target = fake_orbita.get_target_orientation().unwrap();

        assert!((current_target[0] - orientation[0]).abs() < 1e-6);
        assert!((current_target[1] - orientation[1]).abs() < 1e-6);

        fake_orbita.orientation_limits = Some([
            AngleLimit {
                min: orientation[0] + 0.1,
                max: orientation[0] + 0.2,
            },
            AngleLimit {
                min: orientation[1] - 0.2,
                max: orientation[1] - 0.1,
            },
        ]);
        // orientation is exactly outside the limits
        fake_orbita.set_target_orientation(orientation).unwrap();

        let current_target = fake_orbita.get_target_orientation().unwrap();
        //it shoube clamped to the limits
        assert!((current_target[0] - orientation[0] - 0.1).abs() < 1e-6);
        assert!((current_target[1] - orientation[1] + 0.1).abs() < 1e-6);
    }

    #[test]
    fn set_torque() {
        let mut fake_orbita = Orbita2dController::with_fake_motors();
        // Test each transition
        fake_orbita.set_torque(true).unwrap();
        assert!(fake_orbita.is_torque_on().unwrap());
        fake_orbita.set_torque(false).unwrap();
        assert!(!fake_orbita.is_torque_on().unwrap());
        fake_orbita.set_torque(true).unwrap();
        assert!(fake_orbita.is_torque_on().unwrap());
        fake_orbita.disable_torque().unwrap();
        assert!(!fake_orbita.is_torque_on().unwrap());
        fake_orbita.enable_torque(false).unwrap();
        assert!(fake_orbita.is_torque_on().unwrap());
    }

    #[test]
    fn test_torque_with_target_reset() {
        let mut fake_orbita = Orbita2dController::with_fake_motors();
        fake_orbita.set_torque(true).unwrap();
        let mut rng = rand::thread_rng();
        let orientation = [rng.gen_range(-PI..PI), rng.gen_range(-PI..PI)];
        fake_orbita.set_target_orientation(orientation).unwrap();
        let _ = fake_orbita.enable_torque(true);
        let current_target = fake_orbita.get_target_orientation().unwrap();
        assert!((current_target[0] - orientation[0]).abs() < 1e-6);
        assert!((current_target[1] - orientation[1]).abs() < 1e-6);

        let orientation = [rng.gen_range(-PI..PI), rng.gen_range(-PI..PI)];
        let ik = fake_orbita
            .kinematics
            .compute_inverse_kinematics(orientation);

        //change the motor position
        let _ = fake_orbita.inner.set_target_position(ik);

        let _ = fake_orbita.enable_torque(true);
        let current_target = fake_orbita.get_target_orientation().unwrap();
        assert!((current_target[0] - orientation[0]).abs() < 1e-6);
        assert!((current_target[1] - orientation[1]).abs() < 1e-6);
    }
}
