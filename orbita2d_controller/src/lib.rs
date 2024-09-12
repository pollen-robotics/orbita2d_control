//! Orbita2d controller
//!
//! This crate provides a controller for the Orbita2d actuator.
//!
//! # Overview
//!
//! ## Setup
//! - [x] Load configuration from file
//! - [x] Different support for communication layer (Flipsky serial, Fake motors)
//! - [x] Support for inverted axes (ring and center)
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
//!     [false, false],
//!     None,
//!     false,
//! ).expect("Failed to initialize Orbita2d controller");
//!
//! let orientation = orbita2d.get_current_orientation().expect("Communication Error");
//! println!("Current 2d orientation: {:?}", orientation);
//!
//! ```

use log::{debug, error, info};
use serde::{Deserialize, Serialize};
use std::fmt::Debug;

pub use fake_motor::FakeConfig;
pub use flipsky_serial::FlipskyConfig;
use orbita2d_kinematics::Orbita2dKinematicsModel;

use motor_toolbox_rs::{Result, PID};

/// Result generic wrapper using `std::error::Error` trait
// pub type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
mod coherency;
use coherency::CoherentResult;
mod fake_motor;
mod flipsky_serial;
mod poulpe;

mod poulpe_ethercat;
use poulpe_ethercat::PoulpeEthercatConfig;

// #[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
// /// PID gains wrapper
// pub struct PID {
//     /// Propotional gain
//     pub p: f64,
//     /// Integral gain
//     pub i: f64,
//     /// Derivative gain
//     pub d: f64,
// }

#[derive(Debug, Deserialize, Serialize, Copy, Clone)]
/// Feedback struct
pub struct Orbita2dFeedback {
    pub orientation: [f64; 2],
    // pub velocity: [f64; 2],
    // pub torque: [f64; 2],
}

/// Orbita2d controller main interface
pub struct Orbita2dController {
    inner: Box<dyn Orbita2dMotorController + Send>,
    kinematics: Orbita2dKinematicsModel,

    motors_offset: [f64; 2],
    inverted_axes: [bool; 2],

    // Expressed in the "corrected" Orbita2d reference frame
    // Meaning after offset and inverted axes correction
    orientation_limits: Option<[AngleLimit; 2]>,
}

// #[derive(Debug, Deserialize, Serialize)]
// /// Zero type config
// /// This is used to configure the zero of each disk
// pub enum ZeroType {
//     /// ZeroStartup config
//     ZeroStartup(ZeroStartup),
//     /// HallZero config
//     SensorZero(SensorZero),

// }

// #[derive(Debug, Deserialize, Serialize)]
// /// ZeroStartup config
// pub struct ZeroStartup;

// #[derive(Debug, Deserialize, Serialize)]
// /// SensorZero config
// pub struct SensorZero;

#[derive(Debug, Deserialize, Serialize)]
pub enum Orbita2dConfig {
    FakeMotors(FakeConfig),
    Flipsky(FlipskyConfig),
    Poulpe(PoulpeConfig),
    PoulpeEthercat(PoulpeEthercatConfig),
}

#[derive(Debug, Deserialize, Serialize)]
/// Flipsky configuration
pub struct PoulpeConfig {
    /// Serial port name
    pub serial_port: String,
    /// Actuator id
    pub id: u8,
    /// Motors offset [motor_a, motor_b]
    pub motors_offset: [f64; 2],
    /// Motors ratio [motor_a, motor_b]
    pub motors_ratio: [f64; 2],
    /// Motors axes inverted [motor_a, motor_b]
    pub inverted_axes: [bool; 2],
    /// Orientation limits [motor_a, motor_b] (expressed in the corrected motor reference frame - after offset and inversion)
    pub orientation_limits: Option<[AngleLimit; 2]>,
    /// Use cache or not
    pub use_cache: bool,
    /// Hardware zeros already set in the firmware
    pub firmware_zero: Option<bool>,
}

impl Debug for Orbita2dController {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Orbita2dController")
            // .field("inner", &self.inner.name())
            .field("kinematics", &self.kinematics)
            .field("motors_offset", &self.motors_offset)
            .field("inverted_axes", &self.inverted_axes)
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
        inverted_axes: [bool; 2],
        orientation_limits: Option<[AngleLimit; 2]>,
    ) -> Self {
        Self {
            inner: motors_controller,
            kinematics: Orbita2dKinematicsModel::new(motors_ratio[0], motors_ratio[1]),
            motors_offset,
            inverted_axes,
            orientation_limits,
        }
    }

    /// Create a Orbita2d controller with motors implementation as defined in the config file.
    pub fn with_config(configfile: &str) -> Result<Self> {
        let f = match std::fs::File::open(configfile) {
            Ok(f) => f,
            Err(e) => {
                error!("Error opening config file: {}", configfile);
                return Err(Box::new(e));
            }
        };
        info!("Loading config file: {}", configfile);

        let config: Orbita2dConfig = serde_yaml::from_reader(f)?;
        info!("Config: {:?}", config);

        match config {
            Orbita2dConfig::FakeMotors(config) => Ok(Self::with_fake_motors(config.inverted_axes)),
            Orbita2dConfig::Flipsky(config) => Self::with_flipsky_serial(
                (&config.serial_port[0], &config.serial_port[1]),
                (config.ids[0], config.ids[1]),
                config.motors_offset,
                config.motors_ratio,
                config.inverted_axes,
                config.orientation_limits,
                config.use_cache,
            ),
            Orbita2dConfig::Poulpe(config) => Self::with_poulpe_serial(
                &config.serial_port,
                config.id,
                config.motors_offset,
                config.motors_ratio,
                config.inverted_axes,
                config.orientation_limits,
                config.use_cache,
                config.firmware_zero,
            ),
            Orbita2dConfig::PoulpeEthercat(config) => Self::with_poulpe_ethercat(
                &config.url,
                config.id as u16,
                config.motors_offset,
                config.motors_ratio,
                config.inverted_axes,
                config.orientation_limits,
                config.firmware_zero,
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
        if !self.is_torque_on()? && reset_target {
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

        let orientation = self.kinematics.compute_forward_kinematics(pos);
        debug!(target: &self.log_target(), "get_current_orientation (with kinematics): {:?}", orientation);

        let orientation = [
            if self.inverted_axes[0] {
                -orientation[0]
            } else {
                orientation[0]
            },
            if self.inverted_axes[1] {
                -orientation[1]
            } else {
                orientation[1]
            },
        ];
        debug!(target: &self.log_target(), "get_current_orientation (with inverted axes): {:?}", orientation);

        Ok(orientation)
    }
    /// Read the current velocity [ring, center] (in radians/s)
    pub fn get_current_velocity(&mut self) -> Result<[f64; 2]> {
        let vel = self.inner.get_current_velocity()?;
        debug!(target: &self.log_target(), "get_current_velocity: {:?}", vel);

        let oriented_velocity = self.kinematics.compute_forward_kinematics(vel);
        debug!(target: &self.log_target(), "get_current_velocity (with kinematics): {:?}", oriented_velocity);

        let oriented_velocity = [
            if self.inverted_axes[0] {
                -oriented_velocity[0]
            } else {
                oriented_velocity[0]
            },
            if self.inverted_axes[1] {
                -oriented_velocity[1]
            } else {
                oriented_velocity[1]
            },
        ];
        debug!(target: &self.log_target(), "get_current_velocity (with inverted axes): {:?}", oriented_velocity);

        Ok(oriented_velocity)
    }
    /// Read the current torque [ring, center] (in Nm)
    pub fn get_current_torque(&mut self) -> Result<[f64; 2]> {
        let torque = self.inner.get_current_torque()?;
        debug!(target: &self.log_target(), "get_current_torque: {:?}", torque);

        let oriented_torque = self.kinematics.compute_output_torque(torque);
        debug!(target: &self.log_target(), "get_current_torque (with kinematics): {:?}", oriented_torque);

        let oriented_torque = [
            if self.inverted_axes[0] {
                -oriented_torque[0]
            } else {
                oriented_torque[0]
            },
            if self.inverted_axes[1] {
                -oriented_torque[1]
            } else {
                oriented_torque[1]
            },
        ];
        debug!(target: &self.log_target(), "get_current_torque (with inverted axes): {:?}", oriented_torque);

        Ok(oriented_torque)
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

        let oriented_target = self.kinematics.compute_forward_kinematics(pos);
        debug!(target: &self.log_target(), "get_target_orientation (with kinematics): {:?}", oriented_target);

        let oriented_target = [
            if self.inverted_axes[0] {
                -oriented_target[0]
            } else {
                oriented_target[0]
            },
            if self.inverted_axes[1] {
                -oriented_target[1]
            } else {
                oriented_target[1]
            },
        ];
        debug!(target: &self.log_target(), "get_target_orientation (with inverted axes): {:?}", oriented_target);

        Ok(oriented_target)
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

        let target_orientation = [
            if self.inverted_axes[0] {
                -target_orientation[0]
            } else {
                target_orientation[0]
            },
            if self.inverted_axes[1] {
                -target_orientation[1]
            } else {
                target_orientation[1]
            },
        ];
        debug!(target: &self.log_target(), "set_target_orientation (with inverted axes): {:?}", target_orientation);

        let ik = self
            .kinematics
            .compute_inverse_kinematics(target_orientation);
        debug!(target: &self.log_target(), "set_target_orientation ik res: {:?}", ik);
        let pos = [ik[0] + self.motors_offset[0], ik[1] + self.motors_offset[1]];
        debug!(target: &self.log_target(), "set_target_orientation to motors (with offset): {:?}", pos);

        self.inner.set_target_position(pos)
    }

    /// Set the desired orientation [ring, center] (in radians)
    pub fn set_target_orientation_fb(
        &mut self,
        target_orientation: [f64; 2],
    ) -> Result<Orbita2dFeedback> {
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

        let target_orientation = [
            if self.inverted_axes[0] {
                -target_orientation[0]
            } else {
                target_orientation[0]
            },
            if self.inverted_axes[1] {
                -target_orientation[1]
            } else {
                target_orientation[1]
            },
        ];
        debug!(target: &self.log_target(), "set_target_orientation (with inverted axes): {:?}", target_orientation);

        let ik = self
            .kinematics
            .compute_inverse_kinematics(target_orientation);
        debug!(target: &self.log_target(), "set_target_orientation ik res: {:?}", ik);
        let pos = [ik[0] + self.motors_offset[0], ik[1] + self.motors_offset[1]];
        debug!(target: &self.log_target(), "set_target_orientation to motors (with offset): {:?}", pos);

        match self.inner.set_target_position_fb(pos) {
            Ok(fb) => {
                let pos = [
                    fb.orientation[0] - self.motors_offset[0],
                    fb.orientation[1] - self.motors_offset[1],
                ];
                let orientation = self.kinematics.compute_forward_kinematics(pos);
                let orientation = [
                    if self.inverted_axes[0] {
                        -orientation[0]
                    } else {
                        orientation[0]
                    },
                    if self.inverted_axes[1] {
                        -orientation[1]
                    } else {
                        orientation[1]
                    },
                ];
                Ok(Orbita2dFeedback {
                    orientation,
                    // velocity: fb.velocity,
                    // torque: fb.torque,
                })
            }
            Err(e) => Err(e),
        }
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

    pub fn get_axis_sensors(&mut self) -> Result<[f64; 2]> {
        debug!(target: &self.log_target(), "get_axis_sensors");
        self.inner.get_axis_sensors()
    }

    /// Get the BoardState code
    pub fn get_board_state(&mut self) -> Result<u8> {
        debug!(target: &self.log_target(), "get_board_state");
        self.inner.get_board_state()
    }

    /// Set the BoardState code (to reset error)
    pub fn set_board_state(&mut self, state: u8) -> Result<()> {
        debug!(target: &self.log_target(), "set_board_state: {:?}", state);
        self.inner.set_board_state(state)
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
    /// Read the Ring/Center sensors
    fn get_axis_sensors(&mut self) -> Result<[f64; 2]>;
    /// Read the current velocity (in radians/s) of each motor [motor_a, motor_b]
    fn get_current_velocity(&mut self) -> Result<[f64; 2]>;
    /// Read the current torque (in Nm) of each motor [motor_a, motor_b]
    fn get_current_torque(&mut self) -> Result<[f64; 2]>;
    /// Read the target position (in radians) of each motor [motor_a, motor_b]
    fn get_target_position(&mut self) -> Result<[f64; 2]>;
    /// Set the target position (in radians) for each motor [motor_a, motor_b]
    fn set_target_position(&mut self, target_position: [f64; 2]) -> Result<()>;
    /// Set the target position (in radians) for each motor [motor_a, motor_b] and returns the feedback [position, velocity, torque]
    fn set_target_position_fb(&mut self, target_position: [f64; 2]) -> Result<Orbita2dFeedback>;
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
    /// Get the BoardState code
    fn get_board_state(&mut self) -> Result<u8>;
    /// Set the BoardState
    fn set_board_state(&mut self, state: u8) -> Result<()>;
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use super::*;
    // use crate::Orbita2dController;
    use rand::Rng;

    fn init() {
        let _ = env_logger::builder().is_test(true).try_init();
    }

    #[test]
    fn set_target_orientation() {
        init();
        let mut rng = rand::thread_rng();
        let orientation = [rng.gen_range(-PI..PI), rng.gen_range(-PI..PI)];

        let mut fake_orbita = Orbita2dController::with_fake_motors([false, false]);

        fake_orbita.set_target_orientation(orientation).unwrap();

        let current_target = fake_orbita.get_target_orientation().unwrap();

        assert!((current_target[0] - orientation[0]).abs() < 1e-6);
        assert!((current_target[1] - orientation[1]).abs() < 1e-6);
    }

    #[test]
    fn set_target_orientation_with_motors_offset() {
        let mut rng = rand::thread_rng();
        let orientation = [rng.gen_range(-PI..PI), rng.gen_range(-PI..PI)];

        let mut fake_orbita = Orbita2dController::with_fake_motors([false, false]);
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

        let mut fake_orbita = Orbita2dController::with_fake_motors([false, false]);
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

        let mut fake_orbita = Orbita2dController::with_fake_motors([false, false]);
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
    fn set_target_outside_limits_and_inverted_axes() {
        let mut fake_orbita = Orbita2dController::with_fake_motors([true, true]);

        fake_orbita.orientation_limits = Some([
            AngleLimit { min: 0.0, max: 1.0 },
            AngleLimit {
                min: -1.0,
                max: 0.0,
            },
        ]);

        fake_orbita.set_target_orientation([0.5, -0.5]).unwrap();
        let current_target = fake_orbita.get_target_orientation().unwrap();

        assert_eq!(current_target[0], 0.5);
        assert_eq!(current_target[1], -0.5);
    }

    #[test]
    fn set_torque() {
        let mut fake_orbita = Orbita2dController::with_fake_motors([false, false]);
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
        let mut fake_orbita = Orbita2dController::with_fake_motors([false, false]);
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

    #[test]
    fn inverted_axes() {
        let mut rng = rand::thread_rng();
        let raw_motors_pos = [rng.gen_range(-PI..PI), rng.gen_range(-PI..PI)];

        let mut no_inversion = Orbita2dController::with_fake_motors([false, false]);
        no_inversion.enable_torque(false).unwrap();
        no_inversion
            .inner
            .set_target_position(raw_motors_pos)
            .unwrap();

        let mut ring_inverted: Orbita2dController =
            Orbita2dController::with_fake_motors([true, false]);
        ring_inverted.enable_torque(false).unwrap();
        ring_inverted
            .inner
            .set_target_position(raw_motors_pos)
            .unwrap();

        let mut center_inverted: Orbita2dController =
            Orbita2dController::with_fake_motors([false, true]);
        center_inverted.enable_torque(false).unwrap();
        center_inverted
            .inner
            .set_target_position(raw_motors_pos)
            .unwrap();

        let mut both_inverted: Orbita2dController =
            Orbita2dController::with_fake_motors([true, true]);
        both_inverted.enable_torque(false).unwrap();
        both_inverted
            .inner
            .set_target_position(raw_motors_pos)
            .unwrap();

        let orientation = no_inversion.get_current_orientation().unwrap();

        let ring_inverted_orientation = ring_inverted.get_current_orientation().unwrap();
        assert_eq!(orientation[0], -ring_inverted_orientation[0]);
        assert_eq!(orientation[1], ring_inverted_orientation[1]);

        let center_inverted_orientation = center_inverted.get_current_orientation().unwrap();
        assert_eq!(orientation[0], center_inverted_orientation[0]);
        assert_eq!(orientation[1], -center_inverted_orientation[1]);

        let both_inverted_orientation = both_inverted.get_current_orientation().unwrap();
        assert_eq!(orientation[0], -both_inverted_orientation[0]);
        assert_eq!(orientation[1], -both_inverted_orientation[1]);
    }

    #[test]
    fn set_target_inverted_axes() {
        for axes in [[false, false], [true, false], [false, true], [true, true]] {
            let mut fake_orbita = Orbita2dController::with_fake_motors(axes);
            fake_orbita.enable_torque(false).unwrap();

            let mut rng = rand::thread_rng();
            let orientation = [rng.gen_range(-PI..PI), rng.gen_range(-PI..PI)];
            fake_orbita.set_target_orientation(orientation).unwrap();

            let current_target = fake_orbita.get_target_orientation().unwrap();
            assert!((current_target[0] - orientation[0]).abs() < 1e-6);
            assert!((current_target[1] - orientation[1]).abs() < 1e-6);

            let current_orientation = fake_orbita.get_current_orientation().unwrap();
            assert!((current_orientation[0] - orientation[0]).abs() < 1e-6);
            assert!((current_orientation[1] - orientation[1]).abs() < 1e-6);
        }
    }
}
