//! Orbita2D Controller - SimpleFOC Dynamixel device
//!
//! Library for controlling an orbita2d device.
//! It's made to work with version of Orbita2D where the firmware
//! * is based on Flipsky FSVESC ESC
//! * is based on SimpleFOC
//! * uses serial communication via Dynamixel protocol v1
//!

use std::time::Duration;

use crate::{AngleLimit, Orbita2dController, Orbita2dMotorController, Result, PID};
use cache_cache::Cache;

use rustypot::{
    device::orbita2dof_foc::{self},
    DynamixelSerialIO,
};
use serialport::SerialPort;

/// Orbita serial controller
pub struct Orbita2dSerialController {
    serial_ports: (Box<dyn SerialPort>, Box<dyn SerialPort>),
    io: DynamixelSerialIO,
    ids: (u8, u8),

    target_position: Cache<(u8, u8), [f64; 2]>,
    torque_on: Cache<(u8, u8), bool>,
}

impl Orbita2dController {
    pub fn with_serial(
        serial_port_names: (&str, &str),
        ids: (u8, u8),
        motors_offset: [f64; 2],
        motors_ratio: [f64; 2],
        orientation_limits: Option<[AngleLimit; 2]>,
    ) -> Result<Self> {
        let serial_controller = Orbita2dSerialController {
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
            target_position: Cache::keep_last(),
            torque_on: Cache::keep_last(),
        };

        Ok(Self::new(
            Box::new(serial_controller),
            motors_ratio,
            motors_offset,
            orientation_limits,
        ))
    }
}

impl Orbita2dMotorController for Orbita2dSerialController {
    fn is_torque_on(&mut self) -> Result<bool> {
        self.torque_on.entry(self.ids).or_try_insert_with(|_| {
            orbita2dof_foc::read_torque_enable(&self.io, self.serial_ports.0.as_mut(), self.ids.0)
                .map(|torque| torque != 0)
        })
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

    fn get_current_position(&mut self) -> Result<[f64; 2]> {
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
        Ok([pos_a as f64, pos_b as f64])
    }

    fn get_current_velocity(&mut self) -> Result<[f64; 2]> {
        todo!()
    }

    fn get_current_torque(&mut self) -> Result<[f64; 2]> {
        todo!()
    }

    fn get_target_position(&mut self) -> Result<[f64; 2]> {
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

                Ok([pos_a as f64, pos_b as f64])
            })
    }

    fn set_target_position(&mut self, target_position: [f64; 2]) -> Result<()> {
        let current_target = self.get_target_position()?;

        if current_target != target_position {
            orbita2dof_foc::write_motor_a_goal_position(
                &self.io,
                self.serial_ports.0.as_mut(),
                self.ids.0,
                target_position[0] as f32,
            )?;
            // In flipsky we currently only have a motor_a
            orbita2dof_foc::write_motor_a_goal_position(
                &self.io,
                self.serial_ports.1.as_mut(),
                self.ids.1,
                target_position[1] as f32,
            )?;
            self.target_position.insert(self.ids, target_position);
        }

        Ok(())
    }

    fn set_velocity_limit(&mut self, _velocity_limit: [f64; 2]) -> Result<()> {
        todo!()
    }

    fn set_torque_limit(&mut self, _torque_limit: [f64; 2]) -> Result<()> {
        todo!()
    }

    fn set_pid_gains(&mut self, _pid_gains: PID) -> Result<()> {
        todo!()
    }
}
