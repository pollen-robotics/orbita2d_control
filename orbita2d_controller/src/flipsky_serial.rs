use std::time::Duration;

use crate::{AngleLimit, Orbita2dController, Orbita2dMotorController, Result, PID};
use cache_cache::Cache;

use rustypot::{
    device::orbita2dof_foc::{self},
    DynamixelSerialIO,
};
use serialport::SerialPort;

/// Orbita serial controller
pub struct Orbita2dFlipskySerialController {
    serial_ports: [Box<dyn SerialPort>; 2],
    io: DynamixelSerialIO,
    ids: [u8; 2],

    target_position: Cache<u8, f64>,
    torque_on: Cache<u8, bool>,
}

impl Orbita2dController {
    /// Create a new Orbita2dController using flipsky serial communication.
    ///
    /// It's made to work with version of Orbita2D where the firmware/electronic board:
    /// * is based on Flipsky FSVESC ESC
    /// * is based on SimpleFOC
    /// * uses serial communication via Dynamixel protocol v1
    ///
    /// # Arguments
    /// * `serial_port_names` - A tuple with the name of each flipsky serial port.
    /// * `ids` - A tuple with the id of eachy motor.
    /// * `motors_ratio` - An array of the ratio for each motor.
    /// * `motors_offset` - An array of the offset for each motor.
    /// * `orientation_limits` - An option array of the `AngleLimit` for each motor.
    pub fn with_flipsky_serial(
        serial_port_names: (&str, &str),
        ids: (u8, u8),
        motors_offset: [f64; 2],
        motors_ratio: [f64; 2],
        orientation_limits: Option<[AngleLimit; 2]>,
    ) -> Result<Self> {
        let serial_controller = Orbita2dFlipskySerialController {
            serial_ports: [
                serialport::new(serial_port_names.0, 1_000_000)
                    .timeout(Duration::from_millis(10))
                    .open()?,
                serialport::new(serial_port_names.1, 1_000_000)
                    .timeout(Duration::from_millis(10))
                    .open()?,
            ],
            io: DynamixelSerialIO::v1(),
            ids: [ids.0, ids.1],
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

impl Orbita2dMotorController for Orbita2dFlipskySerialController {
    fn name(&self) -> &'static str {
        "FlipskySerialController"
    }

    fn is_torque_on(&mut self) -> Result<[bool; 2]> {
        Ok([
            self.torque_on.entry(self.ids[0]).or_try_insert_with(|_| {
                orbita2dof_foc::read_torque_enable(
                    &self.io,
                    self.serial_ports[0].as_mut(),
                    self.ids[0],
                )
                .map(|torque| torque != 0)
            })?,
            self.torque_on.entry(self.ids[1]).or_try_insert_with(|_| {
                orbita2dof_foc::read_torque_enable(
                    &self.io,
                    self.serial_ports[1].as_mut(),
                    self.ids[1],
                )
                .map(|torque| torque != 0)
            })?,
        ])
    }

    fn set_torque(&mut self, torque: [bool; 2]) -> Result<()> {
        let current_torques = self.is_torque_on()?;

        for i in 0..2 {
            if torque[i] != current_torques[i] {
                orbita2dof_foc::write_torque_enable(
                    &self.io,
                    self.serial_ports[i].as_mut(),
                    self.ids[i],
                    torque[i] as u8,
                )?;

                self.torque_on.insert(self.ids[0], torque[0]);
            }
        }

        Ok(())
    }

    fn get_current_position(&mut self) -> Result<[f64; 2]> {
        let pos_a = orbita2dof_foc::read_motor_a_present_position(
            &self.io,
            self.serial_ports[0].as_mut(),
            self.ids[0],
        )?;
        // In flipsky we currently only have a motor_a
        let pos_b = orbita2dof_foc::read_motor_a_present_position(
            &self.io,
            self.serial_ports[1].as_mut(),
            self.ids[1],
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
        Ok([
            self.target_position
                .entry(self.ids[0])
                .or_try_insert_with(|_| {
                    orbita2dof_foc::read_motor_a_goal_position(
                        &self.io,
                        self.serial_ports[0].as_mut(),
                        self.ids[0],
                    )
                    .map(|pos| pos as f64)
                })?,
            self.target_position
                .entry(self.ids[1])
                .or_try_insert_with(|_| {
                    orbita2dof_foc::read_motor_a_goal_position(
                        &self.io,
                        self.serial_ports[1].as_mut(),
                        self.ids[1],
                    )
                    .map(|pos| pos as f64)
                })?,
        ])
    }

    fn set_target_position(&mut self, target_position: [f64; 2]) -> Result<()> {
        let current_target = self.get_target_position()?;

        for i in 0..2 {
            if current_target[i] != target_position[i] {
                orbita2dof_foc::write_motor_a_goal_position(
                    &self.io,
                    self.serial_ports[i].as_mut(),
                    self.ids[i],
                    target_position[i] as f32,
                )?;
                self.target_position.insert(self.ids[i], target_position[i]);
            }
        }

        Ok(())
    }

    fn get_velocity_limit(&mut self) -> Result<[f64; 2]> {
        todo!()
    }
    fn set_velocity_limit(&mut self, _velocity_limit: [f64; 2]) -> Result<()> {
        todo!()
    }
    fn get_torque_limit(&mut self) -> Result<[f64; 2]> {
        todo!()
    }
    fn set_torque_limit(&mut self, _torque_limit: [f64; 2]) -> Result<()> {
        todo!()
    }
    fn get_pid_gains(&mut self) -> Result<[PID; 2]> {
        todo!()
    }
    fn set_pid_gains(&mut self, _pid_gains: [PID; 2]) -> Result<()> {
        todo!()
    }
}
