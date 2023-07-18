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

use log::{debug, info};
use std::fmt::Debug;

pub use fake_motor::FakeConfig;
pub use flipsky_serial::FlipskyConfig;
use orbita2d_kinematics::Orbita2dKinematicsModel;

/// Result generic wrapper using `std::error::Error` trait
pub type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

mod coherency;
use coherency::CoherentResult;
use serde::{Deserialize, Serialize};
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
    inner: Box<dyn Orbita2dMotorController + Send>,
    kinematics: Orbita2dKinematicsModel,

    motors_offset: [f64; 2],
    orientation_limits: Option<[AngleLimit; 2]>,
}

#[derive(Debug, Deserialize, Serialize)]
pub enum Orbita2dConfig {
    FakeMotors(FakeConfig),
    Flipsky(FlipskyConfig),
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

#[derive(Debug, Deserialize, Serialize)]
/// Angle limit wrapper
pub struct AngleLimit {
    /// lower limit in radians
    pub min: f64,
    /// upper limit in radians
    pub max: f64,
}

impl Orbita2dController {
    fn new(
        motors_controller: Box<dyn Orbita2dMotorController + Send>,
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

    /// Create a Orbita2d controller with motors implementation as defined in the config file.
    pub fn with_config(configfile: &str) -> Result<Self> {
        let f = std::fs::File::open(configfile)?;
        info!("Loading config file: {}", configfile);

        let config: Orbita2dConfig = serde_yaml::from_reader(f)?;
        info!("Config: {:?}", config);

        match config {
            Orbita2dConfig::FakeMotors(_) => Ok(Self::with_fake_motors()),
            Orbita2dConfig::Flipsky(config) => Self::with_flipsky_serial(
                (&config.serial_port[0], &config.serial_port[1]),
                (config.ids[0], config.ids[1]),
                config.motors_offset,
                config.motors_ratio,
                config.orientation_limits,
                config.use_cache,
            ),
        }
    }

    /// Check if the torque is ON or OFF.
    pub fn is_torque_on(&mut self) -> Result<bool> {
        debug!(target: &self.log_target(),
	       "is_torque_on: {:?}", self.inner.is_torque_on().coherent());
        self.inner.is_torque_on().coherent()
    }
    /// Enable the torque.
    ///
    /// # Arguments
    /// * `reset_target` - If true, the target orientation is reset to the current orientation.
    pub fn enable_torque(&mut self, reset_target: bool) -> Result<()> {
        debug!(target: &self.log_target(), "enable_torque, reset_target: {:?}", reset_target);
        if reset_target {
            let current_pos = self.get_current_orientation()?;
            self.set_target_orientation(current_pos)?;
            debug!(target: &self.log_target(),
                "enable_torque -> reset_target to current_pos: {:?}",
                current_pos
            );
        }
        self.set_torque(true)?;
        Ok(())
    }
    /// Disable the torque.
    pub fn disable_torque(&mut self) -> Result<()> {
        debug!(target: &self.log_target(), "disable_torque");
        self.set_torque(false)
    }
    fn set_torque(&mut self, on: bool) -> Result<()> {
        debug!(target: &self.log_target(), "set_torque: {:?}", on);
        self.inner.set_torque([on, on])
    }

    /// Read the current orientation [ring, center] (in radians)
    pub fn get_current_orientation(&mut self) -> Result<[f64; 2]> {
        let pos = self.inner.get_current_position()?;
        debug!(target: &self.log_target(), "get_current_orientation: {:?}", pos);

        let pos = [
            pos[0] - self.motors_offset[0],
            pos[1] - self.motors_offset[1],
        ];
        debug!(target: &self.log_target(), "get_current_orientation (with offset): {:?}", pos);

        Ok(self.kinematics.compute_forward_kinematics(pos))
    }
    /// Read the current velocity [ring, center] (in radians/s)
    pub fn get_current_velocity(&mut self) -> Result<[f64; 2]> {
        let vel = self.inner.get_current_velocity()?;
        debug!(target: &self.log_target(), "get_current_velocity: {:?}", vel);
        Ok(self.kinematics.compute_output_velocity(vel))
    }
    /// Read the current torque [ring, center] (in Nm)
    pub fn get_current_torque(&mut self) -> Result<[f64; 2]> {
        let torque = self.inner.get_current_torque()?;
        debug!(target: &self.log_target(), "get_current_torque: {:?}", torque);
        Ok(self.kinematics.compute_output_torque(torque))
    }

    /// Get the desired orientation [ring, center] (in radians)
    pub fn get_target_orientation(&mut self) -> Result<[f64; 2]> {
        let pos = self.inner.get_target_position()?;
        debug!(target: &self.log_target(), "get_target_orientation: {:?}", pos);
        let pos = [
            pos[0] - self.motors_offset[0],
            pos[1] - self.motors_offset[1],
        ];
        debug!(target: &self.log_target(), "get_target_orientation (with offset): {:?}", pos);

        Ok(self.kinematics.compute_forward_kinematics(pos))
    }
    /// Set the desired orientation [ring, center] (in radians)
    pub fn set_target_orientation(&mut self, target_orientation: [f64; 2]) -> Result<()> {
        let target_orientation = match &self.orientation_limits {
            Some(limits) => [
                target_orientation[0].clamp(limits[0].min, limits[0].max),
                target_orientation[1].clamp(limits[1].min, limits[1].max),
            ],
            None => target_orientation,
        };
        debug!(target: &self.log_target(),
            "set_target_orientation: {:?} orientation_limits {:?}",
            target_orientation, self.orientation_limits
        );

        let ik = self
            .kinematics
            .compute_inverse_kinematics(target_orientation);
        debug!(target: &self.log_target(), "set_target_orientation ik res: {:?}", ik);
        let pos = [ik[0] + self.motors_offset[0], ik[1] + self.motors_offset[1]];
        debug!(target: &self.log_target(), "set_target_orientation to motors (with offset): {:?}", pos);

        self.inner.set_target_position(pos)
    }

    /// Get the velocity limit of each raw motor [motor_a, motor_b] (in radians/s)
    /// caution: this is the raw value used by the motors used inside the actuator, not a limit in orbita2d orientation!
    pub fn get_raw_motors_velocity_limit(&mut self) -> Result<[f64; 2]> {
        debug!(target: &self.log_target(), "get_raw_motors_velocity_limit");
        self.inner.get_velocity_limit()
    }
    /// Set the velocity limit of each raw motor [motor_a, motor_b] (in radians/s)
    /// caution: this is the raw value used by the motors used inside the actuator, not a limit in orbita2d orientation!
    pub fn set_raw_motors_velocity_limit(&mut self, velocity_limit: [f64; 2]) -> Result<()> {
        debug!(target: &self.log_target(), "set_raw_motors_velocity_limit: {:?}", velocity_limit);
        self.inner.set_velocity_limit(velocity_limit)
    }
    /// Get the torque limit of each raw motor [motor_a, motor_b] (in Nm)
    /// caution: this is the raw value used by the motors used inside the actuator, not a limit in orbita2d orientation!
    pub fn get_raw_motors_torque_limit(&mut self) -> Result<[f64; 2]> {
        debug!(target: &self.log_target(), "get_raw_motors_torque_limit");
        self.inner.get_torque_limit()
    }
    /// Set the torque limit of each raw motor [motor_a, motor_b] (in Nm)
    /// caution: this is the raw value used by the motors used inside the actuator, not a limit in orbita2d orientation!
    pub fn set_raw_motors_torque_limit(&mut self, torque_limit: [f64; 2]) -> Result<()> {
        debug!(target: &self.log_target(), "set_raw_motors_torque_limit: {:?}", torque_limit);
        self.inner.set_torque_limit(torque_limit)
    }
    /// Get the PID gains of each raw motor [motor_a, motor_b]
    /// caution: this is the raw value used by the motors used inside the actuator, not a limit in orbita2d orientation!
    pub fn get_raw_motors_pid_gains(&mut self) -> Result<[PID; 2]> {
        debug!(target: &self.log_target(), "get_raw_motors_pid_gains");
        self.inner.get_pid_gains()
    }
    /// Set the PID gains of each raw motor [motor_a, motor_b]
    /// caution: this is the raw value used by the motors used inside the actuator, not a limit in orbita2d orientation!
    pub fn set_raw_motors_pid_gains(&mut self, pid_gains: [PID; 2]) -> Result<()> {
        debug!(target: &self.log_target(), "set_pid_gains: {:?}", pid_gains);
        self.inner.set_pid_gains(pid_gains)
    }

    fn log_target(&self) -> String {
        let name = self.inner.name();
        format!("Orbita2d_controller: {name}")
    }
}

/// Low-level motors controller abstraction for an Orbita2d controller
pub trait Orbita2dMotorController {
    /// Get the name of the motors controller (used only for Debug)
    fn name(&self) -> &'static str;

    /// Check if the torque is ON or OFF [motor_a, motor_b]
    fn is_torque_on(&mut self) -> Result<[bool; 2]>;
    /// Enable/Disable the torque [motor_a, motor_b]
    ///
    /// _Caution: You should guarantee that both motors are always in the same state!_
    fn set_torque(&mut self, on: [bool; 2]) -> Result<()>;
    /// Read the current position (in radians) of each motor [motor_a, motor_b]
    fn get_current_position(&mut self) -> Result<[f64; 2]>;

    /// Read the current velocity (in radians/s) of each motor [motor_a, motor_b]
    fn get_current_velocity(&mut self) -> Result<[f64; 2]>;
    /// Read the current torque (in Nm) of each motor [motor_a, motor_b]
    fn get_current_torque(&mut self) -> Result<[f64; 2]>;
    /// Read the target position (in radians) of each motor [motor_a, motor_b]
    fn get_target_position(&mut self) -> Result<[f64; 2]>;
    /// Set the target position (in radians) for each motor [motor_a, motor_b]
    fn set_target_position(&mut self, target_position: [f64; 2]) -> Result<()>;

    /// Get the velocity limit (in radians/s) of each motor [motor_a, motor_b]
    fn get_velocity_limit(&mut self) -> Result<[f64; 2]>;
    /// Set the velocity limit (in radians/s) for each motor [motor_a, motor_b]
    fn set_velocity_limit(&mut self, velocity_limit: [f64; 2]) -> Result<()>;
    /// Get the torque limit (in Nm) of each motor [motor_a, motor_b]
    fn get_torque_limit(&mut self) -> Result<[f64; 2]>;
    /// Set the torque limit (in Nm) for each motor [motor_a, motor_b]
    fn set_torque_limit(&mut self, torque_limit: [f64; 2]) -> Result<()>;
    /// Get the `PID` gains of each motor [motor_a, motor_b]
    fn get_pid_gains(&mut self) -> Result<[PID; 2]>;
    /// Set the `PID` gains for each motor [motor_a, motor_b]
    fn set_pid_gains(&mut self, pid_gains: [PID; 2]) -> Result<()>;
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use super::*;
    use crate::Orbita2dController;
    use rand::Rng;

    fn init() {
        let _ = env_logger::builder().is_test(true).try_init();
    }

    #[test]
    fn set_target_orientation() {
        init();
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
