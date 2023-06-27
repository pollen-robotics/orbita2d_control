//! Orbita2D Controller - SimpleFOC Dynamixel device
//!
//! Library for controlling an orbita2d device.
//! It's made to work with version of Orbita2D where the firmware
//! * is based on Flipsky FSVESC ESC
//! * is based on SimpleFOC
//! * uses serial communication via Dynamixel protocol v1
//!

use std::time::Duration;

use cache_cache::Cache;
use orbita2d_kinematics::Orbita2dKinematicsModel as KinematicsModel;

use rustypot::{
    device::orbita2dof_foc::{self},
    DynamixelSerialIO,
};
use serialport::SerialPort;

#[derive(Debug)]
pub struct Limit {
    pub min: f32,
    pub max: f32,
}

/// Orbita serial controller
pub struct Orbita2dController {
    serial_ports: (Box<dyn SerialPort>, Box<dyn SerialPort>),
    io: DynamixelSerialIO,
    ids: (u8, u8),
    motors_offset: (f32, f32),
    orientation_limits: Option<(Limit, Limit)>,

    kinematics: KinematicsModel,
    target_position: Cache<(u8, u8), (f32, f32)>,
    torque_on: Cache<(u8, u8), bool>,
}

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

impl Orbita2dController {
    /// Creates an Orbita2d controller.

    pub fn new(
        serial_port_names: (&str, &str),
        ids: (u8, u8),
        motors_offset: (f32, f32),
        motors_ratio: (f32, f32),
        orientation_limits: Option<(Limit, Limit)>,
    ) -> Result<Self> {
        Ok(Orbita2dController {
            serial_ports: (
                serialport::new(serial_port_names.0, 1_000_000)
                    .timeout(Duration::from_millis(10))
                    .open()?,
                serialport::new(serial_port_names.1, 1_000_000)
                    .timeout(Duration::from_millis(10))
                    .open()?,
            ),
            io: DynamixelSerialIO::v1(),
            ids,
            motors_offset,
            orientation_limits,
            kinematics: KinematicsModel::new(motors_ratio.0.into(), motors_ratio.1.into()),
            target_position: Cache::keep_last(),
            torque_on: Cache::keep_last(),
        })
    }

    /// Checks if the torque is on.
    ///
    /// The torque is always the same on all two motors.
    pub fn is_torque_on(&mut self) -> Result<bool> {
        self.torque_on.entry(self.ids).or_try_insert_with(|_| {
            orbita2dof_foc::read_torque_enable(&self.io, self.serial_ports.0.as_mut(), self.ids.0)
                .map(|torque| torque != 0)
        })
    }
    /// Enables the torque on all two motors.
    pub fn enable_torque(&mut self) -> Result<()> {
        self.set_torque(true)
    }
    /// Disables the torque on all two motors.
    pub fn disable_torque(&mut self) -> Result<()> {
        self.set_torque(false)
    }
    fn set_torque(&mut self, torque: bool) -> Result<()> {
        let current_torque = self.is_torque_on()?;
        if torque != current_torque {
            orbita2dof_foc::write_torque_enable(
                &self.io,
                self.serial_ports.0.as_mut(),
                self.ids.0,
                torque as u8,
            )?;

            orbita2dof_foc::write_torque_enable(
                &self.io,
                self.serial_ports.1.as_mut(),
                self.ids.1,
                torque as u8,
            )?;

            self.torque_on.insert(self.ids, torque);
        }

        Ok(())
    }

    pub fn get_motors_current_position(&mut self) -> Result<(f32, f32)> {
        let pos_a = orbita2dof_foc::read_motor_a_present_position(
            &self.io,
            self.serial_ports.0.as_mut(),
            self.ids.0,
        )?;
        // In flipsky we currently only have a motor_a
        let pos_b = orbita2dof_foc::read_motor_a_present_position(
            &self.io,
            self.serial_ports.1.as_mut(),
            self.ids.1,
        )?;
        Ok((pos_a, pos_b))
    }

    pub fn get_current_orientation(&mut self) -> Result<(f32, f32)> {
        let (pos_a, pos_b) = self.get_motors_current_position()?;

        let pos_a = pos_a - self.motors_offset.0;
        let pos_b = pos_b - self.motors_offset.1;

        let ret = self
            .kinematics
            .forward_kinematics(&[f64::from(pos_a), f64::from(pos_b)]);

        Ok((ret[0] as f32, ret[1] as f32))
    }

    pub fn get_motors_target_position(&mut self) -> Result<(f32, f32)> {
        self.target_position
            .entry(self.ids)
            .or_try_insert_with(|_| {
                let pos_a = orbita2dof_foc::read_motor_a_goal_position(
                    &self.io,
                    self.serial_ports.0.as_mut(),
                    self.ids.0,
                )?;
                // In flipsky we currently only have a motor_a
                let pos_b = orbita2dof_foc::read_motor_a_goal_position(
                    &self.io,
                    self.serial_ports.1.as_mut(),
                    self.ids.1,
                )?;

                Ok((pos_a, pos_b))
            })
    }

    pub fn set_motors_goal_position(&mut self, target_position: (f32, f32)) -> Result<()> {
        let current_target = self.get_motors_target_position()?;

        if current_target != target_position {
            orbita2dof_foc::write_motor_a_goal_position(
                &self.io,
                self.serial_ports.0.as_mut(),
                self.ids.0,
                target_position.0,
            )?;
            // In flipsky we currently only have a motor_a
            orbita2dof_foc::write_motor_a_goal_position(
                &self.io,
                self.serial_ports.1.as_mut(),
                self.ids.1,
                target_position.1,
            )?;
            self.target_position.insert(self.ids, target_position);
        }

        Ok(())
    }

    pub fn get_target_orientation(&mut self) -> Result<(f32, f32)> {
        let (pos_a, pos_b) = self.get_motors_target_position()?;

        let pos_a = pos_a - self.motors_offset.0;
        let pos_b = pos_b - self.motors_offset.1;

        let ret = self
            .kinematics
            .forward_kinematics(&[f64::from(pos_a), f64::from(pos_b)]);

        Ok((ret[0] as f32, ret[1] as f32))
    }

    pub fn set_target_orientation(&mut self, target_orientation: (f32, f32)) -> Result<(f32, f32)> {
        let target_orientation = match &self.orientation_limits {
            Some(limits) => (
                target_orientation.0.clamp(limits.0.min, limits.0.max),
                target_orientation.1.clamp(limits.1.min, limits.1.max),
            ),
            None => target_orientation,
        };

        let ik = self
            .kinematics
            .inverse_kinematics(&[target_orientation.0 as f64, target_orientation.1 as f64]);

        let pos_a = ik[0] as f32 + self.motors_offset.0;
        let pos_b = ik[1] as f32 + self.motors_offset.1;

        self.set_motors_goal_position((pos_a, pos_b))?;

        Ok((pos_a, pos_b))
    }

    pub fn set_voltage_limit(&mut self, voltage_limit: f32) -> Result<()> {
        orbita2dof_foc::write_voltage_limit(
            &self.io,
            self.serial_ports.0.as_mut(),
            self.ids.0,
            voltage_limit,
        )?;
        // In flipsky we currently only have a motor_a
        orbita2dof_foc::write_voltage_limit(
            &self.io,
            self.serial_ports.1.as_mut(),
            self.ids.1,
            voltage_limit,
        )?;
        Ok(())
    }

    pub fn get_voltage_limit(&mut self) -> Result<f32> {
        // We consider that both motors have the same voltage limit

        let volt =
            orbita2dof_foc::read_voltage_limit(&self.io, self.serial_ports.0.as_mut(), self.ids.0)?;

        Ok(volt)
    }

    pub fn get_sensors_present_position(&mut self) -> Result<(f32, f32)> {
        // Ring=Motor A
        // Center=Motor B

        let ring = orbita2dof_foc::read_sensor_ring_present_position(
            &self.io,
            self.serial_ports.0.as_mut(),
            self.ids.0,
        )?;
        let center = orbita2dof_foc::read_sensor_center_present_position(
            &self.io,
            self.serial_ports.1.as_mut(),
            self.ids.1,
        )?;
        Ok((ring, center))
    }
}
