use std::time::Duration;

use crate::{AngleLimit, Orbita2dController, Orbita2dMotorController, Result, PID};
use cache_cache::Cache;

use rustypot::{
    device::orbita2dof_foc::{self, Pid},
    DynamixelSerialIO,
};
use serialport::SerialPort;

/// Orbita serial controller
pub struct Orbita2dFlipskySerialController {
    serial_ports: (Box<dyn SerialPort>, Box<dyn SerialPort>),
    io: DynamixelSerialIO,
    ids: (u8, u8),

    target_position: Cache<(u8, u8), [f64; 2]>,
    torque_on: Cache<(u8, u8), bool>,
    torque_limit: Cache<(u8, u8), [f64; 2]>,
    velocity_limit: Cache<(u8, u8), [f64; 2]>,
    pid: Cache<(u8, u8), PID>,
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
            torque_limit: Cache::keep_last(),
            velocity_limit: Cache::keep_last(),
            pid: Cache::keep_last(),
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
        let vel_a = orbita2dof_foc::read_motor_a_present_velocity(
            &self.io,
            self.serial_ports.0.as_mut(),
            self.ids.0,
        )?;
        // In flipsky we currently only have a motor_a
        let vel_b = orbita2dof_foc::read_motor_a_present_velocity(
            &self.io,
            self.serial_ports.1.as_mut(),
            self.ids.1,
        )?;
        Ok([vel_a as f64, vel_b as f64])
    }

    fn get_current_torque(&mut self) -> Result<[f64; 2]> {
        let load_a = orbita2dof_foc::read_motor_a_present_load(
            &self.io,
            self.serial_ports.0.as_mut(),
            self.ids.0,
        )?;
        // In flipsky we currently only have a motor_a
        let load_b = orbita2dof_foc::read_motor_a_present_load(
            &self.io,
            self.serial_ports.1.as_mut(),
            self.ids.1,
        )?;
        Ok([load_a as f64, load_b as f64])
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

    fn get_velocity_limit(&mut self) -> Result<[f64; 2]> {
        self.velocity_limit.entry(self.ids).or_try_insert_with(|_| {
            let velocity_limit_a = orbita2dof_foc::read_angle_velocity_limit(
                &self.io,
                self.serial_ports.0.as_mut(),
                self.ids.0,
            )?;

            let velocity_limit_b = orbita2dof_foc::read_angle_velocity_limit(
                &self.io,
                self.serial_ports.1.as_mut(),
                self.ids.1,
            )?;

            Ok([velocity_limit_a as f64, velocity_limit_b as f64])
        })
    }
    fn set_velocity_limit(&mut self, _velocity_limit: [f64; 2]) -> Result<()> {
        let current_velocity_limit = self.get_velocity_limit()?;

        if current_velocity_limit != _velocity_limit {
            orbita2dof_foc::write_angle_velocity_limit(
                &self.io,
                self.serial_ports.0.as_mut(),
                self.ids.0,
                _velocity_limit[0] as f32,
            )?;

            orbita2dof_foc::write_angle_velocity_limit(
                &self.io,
                self.serial_ports.1.as_mut(),
                self.ids.1,
                _velocity_limit[1] as f32,
            )?;
            self.velocity_limit.insert(self.ids, _velocity_limit);
        }

        Ok(())
    }

    /// Get the current "intensity limit" of the motors. Intensity_limit is the output of the velocity loop (input to the current loop)
    fn get_torque_limit(&mut self) -> Result<[f64; 2]> {
        self.torque_limit.entry(self.ids).or_try_insert_with(|_| {
            let torque_limit_a = orbita2dof_foc::read_intensity_limit(
                &self.io,
                self.serial_ports.0.as_mut(),
                self.ids.0,
            )?;

            let torque_limit_b = orbita2dof_foc::read_intensity_limit(
                &self.io,
                self.serial_ports.1.as_mut(),
                self.ids.1,
            )?;

            Ok([torque_limit_a as f64, torque_limit_b as f64])
        })
    }

    /// Set the current "intensity limit" of the motors. Intensity_limit is the output of the velocity loop (input to the current loop)
    fn set_torque_limit(&mut self, _torque_limit: [f64; 2]) -> Result<()> {
        let current_torque_limit = self.get_torque_limit()?;

        if current_torque_limit != _torque_limit {
            orbita2dof_foc::write_intensity_limit(
                &self.io,
                self.serial_ports.0.as_mut(),
                self.ids.0,
                _torque_limit[0] as f32,
            )?;

            orbita2dof_foc::write_intensity_limit(
                &self.io,
                self.serial_ports.1.as_mut(),
                self.ids.1,
                _torque_limit[1] as f32,
            )?;
            self.torque_limit.insert(self.ids, _torque_limit);
        }

        Ok(())
    }

    /// Get the current angle PID
    fn get_pid_gains(&mut self) -> Result<PID> {
        self.pid.entry(self.ids).or_try_insert_with(|_| {
            let pid_a =
                orbita2dof_foc::read_angle_pid(&self.io, self.serial_ports.0.as_mut(), self.ids.0)?;
            //Here we consider that the two motors have the same PID
            //The PID is set for both motors at the same time so it should be true or an error will be thrown

            Ok(PID {
                p: pid_a.p as f64,
                i: pid_a.i as f64,
                d: pid_a.d as f64,
            })
        })
    }
    /// Set the current angle PID
    fn set_pid_gains(&mut self, _pid_gains: PID) -> Result<()> {
        let current_pid = self.get_pid_gains()?;

        if current_pid != _pid_gains {
            orbita2dof_foc::write_angle_pid(
                &self.io,
                self.serial_ports.0.as_mut(),
                self.ids.0,
                Pid {
                    p: _pid_gains.p as f32,
                    i: _pid_gains.i as f32,
                    d: _pid_gains.d as f32,
                },
            )?;

            orbita2dof_foc::write_angle_pid(
                &self.io,
                self.serial_ports.1.as_mut(),
                self.ids.1,
                Pid {
                    p: _pid_gains.p as f32,
                    i: _pid_gains.i as f32,
                    d: _pid_gains.d as f32,
                },
            )?;
            self.pid.insert(self.ids, _pid_gains);
        }

        Ok(())
    }
}
