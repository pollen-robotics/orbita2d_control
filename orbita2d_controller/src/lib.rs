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
//! ).expect("Failed to initialize Orbita2d controller");
//!
//! let orientation = orbita2d.get_current_orientation().expect("Communication Error");
//! println!("Current 2d orientation: {:?}", orientation);
//!
//! ```

use orbita2d_kinematics::Orbita2dKinematicsModel;

/// Result generic wrapper using `std::error::Error` trait
pub type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

mod flipsky_serial;

#[derive(Debug)]
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

    /// Check if the torque is ON or OFF.
    pub fn is_torque_on(&mut self) -> Result<bool> {
        self.inner.is_torque_on()
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
        self.inner.set_torque(on)
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
        todo!()
    }
    /// Read the current torque (in Nm)
    pub fn get_current_torque(&mut self) -> Result<[f64; 2]> {
        todo!()
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
        self.inner.get_pid_gains()
    }
    /// Set the PID gains
    pub fn set_pid_gains(&mut self, pid_gains: PID) -> Result<()> {
        self.inner.set_pid_gains(pid_gains)
    }
}

/// Low-level motors controller abstraction for an Orbita2d controller
pub trait Orbita2dMotorController {
    /// Check if the torque is ON or OFF
    ///
    /// _Caution: You should guarantee that both motors are always in the same state!_
    fn is_torque_on(&mut self) -> Result<bool>;
    /// Enable/Disable the torque
    ///
    /// _Caution: You should guarantee that both motors are always in the same state!_
    fn set_torque(&mut self, on: bool) -> Result<()>;
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
    fn get_pid_gains(&mut self) -> Result<PID>;
    /// Set the `PID` gains for each motor
    fn set_pid_gains(&mut self, pid_gains: PID) -> Result<()>;
}
