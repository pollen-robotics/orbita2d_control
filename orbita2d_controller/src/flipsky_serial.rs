use crate::{AngleLimit, Orbita2dController, Orbita2dMotorController, Result, PID};
use cache_cache::Cache;
use log::info;
use std::time::Duration;

use rustypot::{
    device::orbita2dof_foc::{self},
    DynamixelSerialIO,
};
use serde::{Deserialize, Serialize};
use serialport::SerialPort;

/// Orbita serial controller
pub struct Orbita2dFlipskySerialController {
    serial_ports: [Box<dyn SerialPort>; 2],
    io: DynamixelSerialIO,
    ids: [u8; 2],
}

/// Orbita serial cached controller
pub struct Orbita2dFlipskySerialCachedController {
    inner: Orbita2dFlipskySerialController,

    torque_on: Cache<u8, bool>,
    target_position: Cache<u8, f64>,
    torque_limit: Cache<u8, f64>,
    velocity_limit: Cache<u8, f64>,
    pid_gains: Cache<u8, PID>,
}

#[derive(Debug, Deserialize, Serialize)]
/// Flipsky configuration
pub struct FlipskyConfig {
    /// Serial port names [motor_a, motor_b]
    pub serial_port: [String; 2],
    /// Motor ids [motor_a, motor_b]
    pub ids: [u8; 2],
    /// Motors offset [motor_a, motor_b]
    pub motors_offset: [f64; 2],
    /// Motors ratio [motor_a, motor_b]
    pub motors_ratio: [f64; 2],
    /// Motors axes inverted [motor_a, motor_b]
    pub motors_axes_inverted: [bool; 2],
    /// Orientation limits [motor_a, motor_b] (expressed in the corrected motor reference frame - after offset and inversion)
    pub orientation_limits: Option<[AngleLimit; 2]>,
    /// Use cache or not
    pub use_cache: bool,
}

impl Orbita2dController {
    /// Create a new Orbita2dController using flipsky serial communication.
    ///
    /// It's made to work with version of Orbita2D where the firmware/electronic board:
    /// * is based on Flipsky FSVESC ESC
    /// * is based on SimpleFOC
    /// * uses serial communication via Dynamixel protocol v1
    ///
    /// Cache behavior is as follow:
    /// * current_position: none
    /// * current_velocity: none
    /// * current_load: none
    ///
    /// * torque_on: keep last
    /// * target_position: keep last
    /// * torque_limit: keep last
    /// * velocity_limit: keep last
    /// * pid_gains: keep last
    ///
    /// # Arguments
    /// * `serial_port_names` - A tuple with the name of each flipsky serial port (motor_a, motor_b).
    /// * `ids` - A tuple with the id of eachy motor (motor_a, motor_b).
    /// * `motors_ratio` - An array of the ratio for each motor (motor_a, motor)b.
    /// * `motors_offset` - An array of the offset for each motor (motor_a, motor_b).
    /// * `motor_axes_inverted` - An array of boolean to invert the direction of each motor (motor_a, motor_b).
    /// * `orientation_limits` - An option array of the `AngleLimit` for each motor (motor_a, motor_b). The limits is expressed in the corrected motor reference frame (after offset and inversion)
    /// * `use_cache` - A boolean to enable/disable cache.
    pub fn with_flipsky_serial(
        serial_port_names: (&str, &str),
        ids: (u8, u8),
        motors_offset: [f64; 2],
        motors_ratio: [f64; 2],
        motors_axes_inverted: [bool; 2],
        orientation_limits: Option<[AngleLimit; 2]>,
        use_cache: bool,
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
        };
        info!("FlipskySerialController:\n\t - serial_port_names: {:?}\n\t - ids: {:?}\n\t - use_cache: {:?}",
			serial_port_names, ids, use_cache);

        Ok(match use_cache {
            true => Self::new(
                Box::new(Orbita2dFlipskySerialCachedController {
                    inner: serial_controller,
                    target_position: Cache::keep_last(),
                    torque_on: Cache::keep_last(),
                    torque_limit: Cache::keep_last(),
                    velocity_limit: Cache::keep_last(),
                    pid_gains: Cache::keep_last(),
                }),
                motors_ratio,
                motors_offset,
                motors_axes_inverted,
                orientation_limits,
            ),
            false => Self::new(
                Box::new(serial_controller),
                motors_ratio,
                motors_offset,
                motors_axes_inverted,
                orientation_limits,
            ),
        })
    }
}

impl Orbita2dMotorController for Orbita2dFlipskySerialController {
    fn name(&self) -> &'static str {
        "FlipskySerialController"
    }

    fn is_torque_on(&mut self) -> Result<[bool; 2]> {
        Ok([
            orbita2dof_foc::read_torque_enable(
                &self.io,
                self.serial_ports[0].as_mut(),
                self.ids[0],
            )
            .map(|torque| torque != 0)?,
            orbita2dof_foc::read_torque_enable(
                &self.io,
                self.serial_ports[1].as_mut(),
                self.ids[1],
            )
            .map(|torque| torque != 0)?,
        ])
    }

    fn set_torque(&mut self, on: [bool; 2]) -> Result<()> {
        for (i, &on) in on.iter().enumerate() {
            orbita2dof_foc::write_torque_enable(
                &self.io,
                self.serial_ports[i].as_mut(),
                self.ids[i],
                on as u8,
            )?;
        }
        Ok(())
    }

    fn get_current_position(&mut self) -> Result<[f64; 2]> {
        Ok([
            orbita2dof_foc::read_motor_a_present_position(
                &self.io,
                self.serial_ports[0].as_mut(),
                self.ids[0],
            )
            .map(|pos| pos as f64)?,
            orbita2dof_foc::read_motor_a_present_position(
                &self.io,
                self.serial_ports[1].as_mut(),
                self.ids[1],
            )
            .map(|pos| pos as f64)?,
        ])
    }

    fn get_current_velocity(&mut self) -> Result<[f64; 2]> {
        Ok([
            orbita2dof_foc::read_motor_a_present_velocity(
                &self.io,
                self.serial_ports[0].as_mut(),
                self.ids[0],
            )
            .map(|vel| vel as f64)?,
            orbita2dof_foc::read_motor_a_present_velocity(
                &self.io,
                self.serial_ports[1].as_mut(),
                self.ids[1],
            )
            .map(|vel| vel as f64)?,
        ])
    }

    fn get_current_torque(&mut self) -> Result<[f64; 2]> {
        Ok([
            orbita2dof_foc::read_motor_a_present_load(
                &self.io,
                self.serial_ports[0].as_mut(),
                self.ids[0],
            )
            .map(|torque| torque as f64)?,
            orbita2dof_foc::read_motor_a_present_load(
                &self.io,
                self.serial_ports[1].as_mut(),
                self.ids[1],
            )
            .map(|torque| torque as f64)?,
        ])
    }

    fn get_target_position(&mut self) -> Result<[f64; 2]> {
        Ok([
            orbita2dof_foc::read_motor_a_goal_position(
                &self.io,
                self.serial_ports[0].as_mut(),
                self.ids[0],
            )
            .map(|pos| pos as f64)?,
            orbita2dof_foc::read_motor_a_goal_position(
                &self.io,
                self.serial_ports[1].as_mut(),
                self.ids[1],
            )
            .map(|pos| pos as f64)?,
        ])
    }

    fn set_target_position(&mut self, target_position: [f64; 2]) -> Result<()> {
        for (i, &target_position) in target_position.iter().enumerate() {
            orbita2dof_foc::write_motor_a_goal_position(
                &self.io,
                self.serial_ports[i].as_mut(),
                self.ids[i],
                target_position as f32,
            )?;
        }
        Ok(())
    }

    fn get_velocity_limit(&mut self) -> Result<[f64; 2]> {
        Ok([
            orbita2dof_foc::read_angle_velocity_limit(
                &self.io,
                self.serial_ports[0].as_mut(),
                self.ids[0],
            )
            .map(|vel| vel as f64)?,
            orbita2dof_foc::read_angle_velocity_limit(
                &self.io,
                self.serial_ports[1].as_mut(),
                self.ids[1],
            )
            .map(|vel| vel as f64)?,
        ])
    }

    fn set_velocity_limit(&mut self, velocity_limit: [f64; 2]) -> Result<()> {
        for (i, &velocity_limit) in velocity_limit.iter().enumerate() {
            orbita2dof_foc::write_angle_velocity_limit(
                &self.io,
                self.serial_ports[i].as_mut(),
                self.ids[i],
                velocity_limit as f32,
            )?;
        }
        Ok(())
    }

    fn get_torque_limit(&mut self) -> Result<[f64; 2]> {
        Ok([
            orbita2dof_foc::read_intensity_limit(
                &self.io,
                self.serial_ports[0].as_mut(),
                self.ids[0],
            )
            .map(|torque| torque as f64)?,
            orbita2dof_foc::read_intensity_limit(
                &self.io,
                self.serial_ports[1].as_mut(),
                self.ids[1],
            )
            .map(|torque| torque as f64)?,
        ])
    }

    fn set_torque_limit(&mut self, torque_limit: [f64; 2]) -> Result<()> {
        for (i, &torque_limit) in torque_limit.iter().enumerate() {
            orbita2dof_foc::write_intensity_limit(
                &self.io,
                self.serial_ports[i].as_mut(),
                self.ids[i],
                torque_limit as f32,
            )?;
        }
        Ok(())
    }

    fn get_pid_gains(&mut self) -> Result<[PID; 2]> {
        Ok([
            orbita2dof_foc::read_angle_pid(&self.io, self.serial_ports[0].as_mut(), self.ids[0])
                .map(|pid| PID {
                    p: pid.p as f64,
                    i: pid.i as f64,
                    d: pid.d as f64,
                })?,
            orbita2dof_foc::read_angle_pid(&self.io, self.serial_ports[1].as_mut(), self.ids[1])
                .map(|pid| PID {
                    p: pid.p as f64,
                    i: pid.i as f64,
                    d: pid.d as f64,
                })?,
        ])
    }

    fn set_pid_gains(&mut self, pid_gains: [PID; 2]) -> Result<()> {
        for (i, &pid_gains) in pid_gains.iter().enumerate() {
            orbita2dof_foc::write_angle_pid(
                &self.io,
                self.serial_ports[i].as_mut(),
                self.ids[i],
                orbita2dof_foc::Pid {
                    p: pid_gains.p as f32,
                    i: pid_gains.i as f32,
                    d: pid_gains.d as f32,
                },
            )?;
        }
        Ok(())
    }
}

impl Orbita2dMotorController for Orbita2dFlipskySerialCachedController {
    fn name(&self) -> &'static str {
        "FlipskySerialCachedController"
    }

    fn is_torque_on(&mut self) -> Result<[bool; 2]> {
        let ids = self.inner.ids;

        self.torque_on
            .entries(&ids)
            .or_try_insert_with(|_| Ok(self.inner.is_torque_on()?.to_vec()))
            .map(|vec| vec.try_into().unwrap())
    }

    fn set_torque(&mut self, torque: [bool; 2]) -> Result<()> {
        let current_torques = self.is_torque_on()?;

        if current_torques != torque {
            self.inner.set_torque(torque)?;

            for (&id, &torque) in self.inner.ids.iter().zip(torque.iter()) {
                self.torque_on.insert(id, torque);
            }
        }

        Ok(())
    }

    fn get_current_position(&mut self) -> Result<[f64; 2]> {
        self.inner.get_current_position()
    }

    fn get_current_velocity(&mut self) -> Result<[f64; 2]> {
        self.inner.get_current_velocity()
    }

    fn get_current_torque(&mut self) -> Result<[f64; 2]> {
        self.inner.get_current_torque()
    }

    fn get_target_position(&mut self) -> Result<[f64; 2]> {
        let ids = self.inner.ids;

        self.target_position
            .entries(&ids)
            .or_try_insert_with(|_| Ok(self.inner.get_target_position()?.to_vec()))
            .map(|vec| vec.try_into().unwrap())
    }

    fn set_target_position(&mut self, target_position: [f64; 2]) -> Result<()> {
        let current_target = self.get_target_position()?;

        if current_target != target_position {
            self.inner.set_target_position(target_position)?;

            for (&id, &pos) in self.inner.ids.iter().zip(target_position.iter()) {
                self.target_position.insert(id, pos);
            }
        }

        Ok(())
    }

    fn get_velocity_limit(&mut self) -> Result<[f64; 2]> {
        let ids = self.inner.ids;

        self.velocity_limit
            .entries(&ids)
            .or_try_insert_with(|_| Ok(self.inner.get_velocity_limit()?.to_vec()))
            .map(|vec| vec.try_into().unwrap())
    }
    /// Set the current velocity limit of the motors
    ///
    /// # Arguments
    /// * `velocity_limit` - The velocity limit for each motor (motor_a, motor_b) in rad/s. Max values are typically around 400-600 rads/s.
    fn set_velocity_limit(&mut self, velocity_limit: [f64; 2]) -> Result<()> {
        let current_velocity_limit = self.get_velocity_limit()?;

        if current_velocity_limit != velocity_limit {
            self.inner.set_velocity_limit(velocity_limit)?;

            for (&id, &vel) in self.inner.ids.iter().zip(velocity_limit.iter()) {
                self.velocity_limit.insert(id, vel);
            }
        }

        Ok(())
    }

    /// Get the current "intensity limit" of the motors. Intensity_limit is the output of the velocity loop (input to the current loop)
    fn get_torque_limit(&mut self) -> Result<[f64; 2]> {
        let ids = self.inner.ids;

        self.torque_limit
            .entries(&ids)
            .or_try_insert_with(|_| Ok(self.inner.get_torque_limit()?.to_vec()))
            .map(|vec| vec.try_into().unwrap())
    }

    /// Set the current "intensity limit" of the motors. Intensity_limit is the output of the velocity loop (input to the current loop)
    fn set_torque_limit(&mut self, torque_limit: [f64; 2]) -> Result<()> {
        let current_torque_limit = self.get_torque_limit()?;

        if current_torque_limit != torque_limit {
            self.inner.set_torque_limit(torque_limit)?;

            for (&id, &torque) in self.inner.ids.iter().zip(torque_limit.iter()) {
                self.torque_limit.insert(id, torque);
            }
        }

        Ok(())
    }

    /// Get the current angle PID
    fn get_pid_gains(&mut self) -> Result<[PID; 2]> {
        let ids = self.inner.ids;

        self.pid_gains
            .entries(&ids)
            .or_try_insert_with(|_| {
                Ok(self
                    .inner
                    .get_pid_gains()?
                    .iter()
                    .map(|&pid| PID {
                        p: pid.p,
                        i: pid.i,
                        d: pid.d,
                    })
                    .collect())
            })
            .map(|vec| vec.try_into().unwrap())
    }
    /// Set the current angle PID
    fn set_pid_gains(&mut self, pid_gains: [PID; 2]) -> Result<()> {
        let current_pid = self.get_pid_gains()?;

        if current_pid != pid_gains {
            self.inner.set_pid_gains(pid_gains)?;

            for (&id, &pid) in self.inner.ids.iter().zip(pid_gains.iter()) {
                self.pid_gains.insert(id, pid);
            }
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use crate::Orbita2dConfig;

    #[test]
    fn parse_config() {
        let s = "!Flipsky
        serial_port:
        - /dev/ttyACM0
        - /dev/ttyACM1
        ids:
        - 0
        - 1
        motors_offset:
        - 0.0
        - 0.0
        motors_ratio:
        - 1.0
        - 1.0
        motors_axes_inverted:
        - false
        - false
        orientation_limits: null
        use_cache: true
        ";

        let config: Result<Orbita2dConfig, _> = serde_yaml::from_str(s);
        assert!(config.is_ok());

        let config = config.unwrap();

        if let Orbita2dConfig::Flipsky(config) = config {
            assert_eq!(config.serial_port, ["/dev/ttyACM0", "/dev/ttyACM1"]);
            assert_eq!(config.ids, [0, 1]);
            assert_eq!(config.motors_offset, [0.0, 0.0]);
            assert_eq!(config.motors_ratio, [1.0, 1.0]);
            assert!(config.orientation_limits.is_none());
            assert!(config.use_cache);
        } else {
            assert!(false, "Wrong config type");
        }
    }
    #[test]
    fn parse_config_file() {
        let f = std::fs::File::open("./config/left_shoulder_flipsky.yaml").unwrap();
        let config: Orbita2dConfig = serde_yaml::from_reader(f).unwrap();

        if let Orbita2dConfig::Flipsky(config) = config {
            assert_eq!(
                config.serial_port,
                ["/dev/left_shoulder_A", "/dev/left_shoulder_B"]
            );
            assert_eq!(config.ids, [83, 84]);
            assert_eq!(config.motors_offset, [0.0, 0.0]);
            assert_eq!(config.motors_ratio, [45.018, 45.018]);
            assert!(config.orientation_limits.is_none());
            assert!(!config.use_cache);
        } else {
            assert!(false, "Wrong config type");
        }
    }
}
