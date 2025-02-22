use std::{thread, time::Duration};

use std::f64::consts::PI;

use cache_cache::Cache;
use log::{debug, error, info, warn};
use rustypot::{
    device::orbita2d_poulpe::{self, MotorValue},
    DynamixelSerialIO,
};
use serialport::TTYPort;

use crate::{Orbita2dController, Orbita2dFeedback, Orbita2dMotorController};
use motor_toolbox_rs::{Limit, Result, PID};

/// Orbita serial controller
struct Orbita2dPoulpeSerialController {
    serial_port: Box<TTYPort>,
    io: DynamixelSerialIO,
    id: u8,
    // offset: [Option<f64>; 2],
    // reduction: [Option<f64>; 2],
    // limits: [Option<Limit>; 2],
    // inverted_axes: [Option<bool>; 2],
    // raw_motor_offsets: [Option<f64>; 2],
}
struct Orbita2dPoulpeSerialCachedController {
    inner: Orbita2dPoulpeSerialController,

    torque_on: Cache<u8, [bool; 2]>,
    target_position: Cache<u8, [f64; 2]>,
    velocity_limit: Cache<u8, [f64; 2]>,
    torque_limit: Cache<u8, [f64; 2]>,
}

impl Orbita2dController {
    #[allow(clippy::too_many_arguments)]
    pub fn with_poulpe_serial(
        serial_port: &str,
        id: u8,
        motors_offset: [f64; 2],
        motors_ratio: [f64; 2],
        inverted_axes: [bool; 2],
        orientation_limits: Option<[Limit; 2]>,
        use_cache: bool,
        firmware_zero: Option<bool>,
    ) -> Result<Self> {
        let mut poulpe_controller = Orbita2dPoulpeSerialController {
            serial_port: Box::new(
                serialport::new(serial_port, 2_000_000)
                    .timeout(Duration::from_millis(10))
                    .open_native()?,
            ),
            io: DynamixelSerialIO::v1().with_post_delay(Duration::from_micros(1000)),
            id,
            // offset: [Some(motors_offset[0]), Some(motors_offset[1])],
            // reduction: [Some(motors_ratio[0]), Some(motors_ratio[1])],
            // limits: [None,None], //Orientation limits
            // inverted_axes: [Some(inverted_axes[0]), Some(inverted_axes[1])],
            // raw_motor_offsets: [None,None],
        };

        poulpe_controller.serial_port.set_exclusive(false)?;

        info!(
            "Orbita2d PoulpeSerialController:\n\t - serial_port: {:?}\n\t - id: {:?}\n\t - use_cache: {:?}",
            serial_port, id, use_cache
        );

        let mut controller = match use_cache {
            true => Self::new(
                Box::new(Orbita2dPoulpeSerialCachedController {
                    inner: poulpe_controller,
                    target_position: Cache::keep_last(),
                    velocity_limit: Cache::keep_last(),
                    torque_limit: Cache::keep_last(),
                    torque_on: Cache::keep_last(),
                }),
                motors_ratio,
                motors_offset,
                inverted_axes,
                orientation_limits,
            ),
            false => Self::new(
                Box::new(poulpe_controller),
                motors_ratio,
                motors_offset,
                inverted_axes,
                orientation_limits,
            ),
        };

        // Compute zero based on given offset and inverted axes

        // 	controller.disable_torque()?; //FIXME: It seems that the axis sensors do not work if the torque is enabled
        // 	thread::sleep(Duration::from_millis(100));

        // 	let current_position = controller.inner.get_current_position()?;
        // 	let current_axis_sensors = controller.inner.get_axis_sensors()?;

        // 	let mut current_axis_position= controller.kinematics.compute_inverse_kinematics(current_axis_sensors);
        // 	warn!("AXIS SENSOR: {:?}, AXIS POS: {:?}",current_axis_sensors, current_axis_position); //TODO Handle NaN
        // 	current_axis_position.iter_mut().enumerate().for_each(|(i, val)| {
        // 	    *val+=current_position[i];
        // 	});
        // 	let raw_zero=controller.kinematics.compute_inverse_kinematics(motors_offset);
        // 	warn!("Current absolute: {:?}", current_axis_position);
        // current_axis_position.iter_mut().enumerate().for_each(|(i, val)| {
        // 	    *val-=raw_zero[i];
        // 	});
        // 	warn!("OFFSETS: {:?}", current_axis_position);

        // controller.motors_offset=current_axis_position;
        //TODO change the name in the config: motors_offset -> axis_offset (angle offset on the axis, measured with axis_sensors). motors_offset is used for raw motors offset

        let mut trials = 0;
        controller.motors_offset = [0.0, 0.0];
        match firmware_zero {
            Some(true) => {
                // if the firmware_zero is set to true in the yaml file
                // fix for the moment
                controller.motors_offset = loop {
                    match find_additional_motor_offsets(
                        &mut controller,
                        motors_offset,
                        inverted_axes,
                        motors_ratio,
                    ) {
                        Ok(offsets) => break offsets,
                        Err(e) => {
                            warn!("Error while finding additional motor offsets: {:?}", e);
                            thread::sleep(Duration::from_millis(100));
                        }
                    }
                    trials += 1;
                    if trials > 10 {
                        return Err("Error while finding additional motor offsets".into());
                    }
                };
            }
            _ => {
                // if firmware_zero is not set or set to false in the yaml
                controller.motors_offset = loop {
                    match find_raw_motor_offsets(
                        &mut controller,
                        motors_offset,
                        inverted_axes,
                        motors_ratio,
                    ) {
                        Ok(offsets) => break offsets,
                        Err(e) => {
                            warn!("Error while finding raw motor offsets: {:?}", e);
                            thread::sleep(Duration::from_millis(100));
                        }
                    }
                    trials += 1;
                    if trials > 10 {
                        return Err("Error while finding raw motor offsets".into());
                    }
                }
            }
        };

        Ok(controller)
    }
}

// impl MotorsController<2> for Orbita2dPoulpeSerialController {
//     fn io(&mut self) -> &mut dyn RawMotorsIO<2> {
//         self
//     }

//     fn offsets(&self) -> [Option<f64>; 2] {
//         self.offsets

//     }

//     fn reduction(&self) -> [Option<f64>; 2] {
// 	let mut reduction = [None; 2];
//         reduction.iter_mut().enumerate().for_each(|(i, r)| {
// 			*r = Some(self.reduction[i].unwrap());
// 		});
// 	reduction

//     }

//     fn limits(&self) -> [Option<motor_toolbox_rs::Limit>; 2] {
//         self.limits
//     }
// }

impl Orbita2dMotorController for Orbita2dPoulpeSerialController {
    fn name(&self) -> String {
        "PoulpeSerialController".to_string()
    }

    fn is_torque_on(&mut self) -> Result<[bool; 2]> {
        orbita2d_poulpe::read_torque_enable(&self.io, self.serial_port.as_mut(), self.id)
            .map(|val| [val.motor_a, val.motor_b])
    }

    fn set_torque(&mut self, on: [bool; 2]) -> Result<()> {
        orbita2d_poulpe::write_torque_enable(
            &self.io,
            self.serial_port.as_mut(),
            self.id,
            MotorValue {
                motor_a: on[0],
                motor_b: on[1],
            },
        )
    }

    fn get_current_position(&mut self) -> Result<[f64; 2]> {
        orbita2d_poulpe::read_current_position(&self.io, self.serial_port.as_mut(), self.id)
            .map(|val| [val.motor_a as f64, val.motor_b as f64])
    }

    fn get_current_velocity(&mut self) -> Result<[f64; 2]> {
        orbita2d_poulpe::read_current_velocity(&self.io, self.serial_port.as_mut(), self.id)
            .map(|val| [val.motor_a as f64, val.motor_b as f64])
    }

    fn get_current_torque(&mut self) -> Result<[f64; 2]> {
        orbita2d_poulpe::read_current_torque(&self.io, self.serial_port.as_mut(), self.id)
            .map(|val| [val.motor_a as f64, val.motor_b as f64])
    }

    fn get_target_position(&mut self) -> Result<[f64; 2]> {
        orbita2d_poulpe::read_target_position(&self.io, self.serial_port.as_mut(), self.id)
            .map(|val| [val.motor_a as f64, val.motor_b as f64])
    }

    fn set_target_position(&mut self, target_position: [f64; 2]) -> Result<()> {
        match orbita2d_poulpe::write_target_position(
            &self.io,
            self.serial_port.as_mut(),
            self.id,
            MotorValue {
                motor_a: target_position[0] as f32,
                motor_b: target_position[1] as f32,
            },
        ) {
            Ok(_) => Ok(()),
            Err(e) => {
                error!("Error while setting target position: {:?}", e);
                Err(e)
            }
        }
    }

    fn set_target_position_fb(&mut self, target_position: [f64; 2]) -> Result<Orbita2dFeedback> {
        match orbita2d_poulpe::write_target_position(
            &self.io,
            self.serial_port.as_mut(),
            self.id,
            MotorValue {
                motor_a: target_position[0] as f32,
                motor_b: target_position[1] as f32,
            },
        ) {
            Ok(fb) => Ok(Orbita2dFeedback {
                orientation: [fb.position.motor_a as f64, fb.position.motor_b as f64],
                // velocity: [fb.speed.motor_a as f64, fb.speed.motor_b as f64],
                // torque: [fb.load.motor_a as f64, fb.load.motor_b as f64],
            }),
            Err(e) => {
                error!("Error while setting target position: {:?}", e);
                Err(e)
            }
        }
    }

    fn get_axis_sensors(&mut self) -> Result<[f64; 2]> {
        orbita2d_poulpe::read_axis_sensor(&self.io, self.serial_port.as_mut(), self.id)
            .map(|val| [val.motor_a as f64, val.motor_b as f64])
    }

    fn get_velocity_limit(&mut self) -> Result<[f64; 2]> {
        orbita2d_poulpe::read_velocity_limit(&self.io, self.serial_port.as_mut(), self.id)
            .map(|val| [val.motor_a as f64, val.motor_b as f64])
    }

    fn set_velocity_limit(&mut self, _velocity_limit: [f64; 2]) -> Result<()> {
        orbita2d_poulpe::write_velocity_limit(
            &self.io,
            self.serial_port.as_mut(),
            self.id,
            MotorValue {
                motor_a: _velocity_limit[0] as f32,
                motor_b: _velocity_limit[1] as f32,
            },
        )
    }

    fn get_torque_limit(&mut self) -> Result<[f64; 2]> {
        orbita2d_poulpe::read_torque_flux_limit(&self.io, self.serial_port.as_mut(), self.id)
            .map(|val| [val.motor_a as f64, val.motor_b as f64])
    }

    fn set_torque_limit(&mut self, _torque_limit: [f64; 2]) -> Result<()> {
        orbita2d_poulpe::write_torque_flux_limit(
            &self.io,
            self.serial_port.as_mut(),
            self.id,
            MotorValue {
                motor_a: _torque_limit[0] as f32,
                motor_b: _torque_limit[1] as f32,
            },
        )
    }

    fn get_pid_gains(&mut self) -> Result<[PID; 2]> {
        orbita2d_poulpe::read_position_pid(&self.io, self.serial_port.as_mut(), self.id).map(
            |thetas| {
                [
                    PID {
                        p: thetas.motor_a.p as f64,
                        i: thetas.motor_a.i as f64,
                        d: 0.0,
                    },
                    PID {
                        p: thetas.motor_b.p as f64,
                        i: thetas.motor_b.i as f64,
                        d: 0.0,
                    },
                ]
            },
        )
    }

    fn set_pid_gains(&mut self, _pid_gains: [PID; 2]) -> Result<()> {
        orbita2d_poulpe::write_position_pid(
            &self.io,
            self.serial_port.as_mut(),
            self.id,
            MotorValue {
                motor_a: orbita2d_poulpe::Pid {
                    p: _pid_gains[0].p as i16,
                    i: _pid_gains[0].i as i16,
                },
                motor_b: orbita2d_poulpe::Pid {
                    p: _pid_gains[1].p as i16,
                    i: _pid_gains[1].i as i16,
                },
            },
        )
    }

    fn get_board_state(&mut self) -> Result<u8> {
        orbita2d_poulpe::read_board_state(&self.io, self.serial_port.as_mut(), self.id)
    }

    fn set_board_state(&mut self, state: u8) -> Result<()> {
        orbita2d_poulpe::write_board_state(&self.io, self.serial_port.as_mut(), self.id, state)
    }
}

impl Orbita2dMotorController for Orbita2dPoulpeSerialCachedController {
    fn name(&self) -> String {
        "PoulpeSerialCachedController".to_string()
    }

    fn is_torque_on(&mut self) -> Result<[bool; 2]> {
        self.torque_on
            .entry(self.inner.id)
            .or_try_insert_with(|_| self.inner.is_torque_on())
    }

    fn set_torque(&mut self, torque: [bool; 2]) -> Result<()> {
        let current_torques = self.is_torque_on()?;

        if current_torques != torque {
            self.inner.set_torque(torque)?;
            self.torque_on.insert(self.inner.id, torque);
        }

        Ok(())
    }

    fn get_axis_sensors(&mut self) -> Result<[f64; 2]> {
        self.inner.get_axis_sensors()
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
        self.target_position
            .entry(self.inner.id)
            .or_try_insert_with(|_| self.inner.get_target_position())
    }

    fn set_target_position(&mut self, target_position: [f64; 2]) -> Result<()> {
        let current_target = self.get_target_position()?;

        if current_target != target_position {
            self.inner.set_target_position(target_position)?;
            self.target_position.insert(self.inner.id, target_position);
        }

        Ok(())
    }

    fn set_target_position_fb(&mut self, target_position: [f64; 2]) -> Result<Orbita2dFeedback> {
        // let current_target = self.get_target_position()?;

        // let mut fb = None;
        // if current_target != target_position {
        match self.inner.set_target_position_fb(target_position) {
            Ok(f) => {
                self.target_position.insert(self.inner.id, target_position);
                Ok(f)
            }
            Err(e) => Err(e),
        }

        // }
    }

    fn get_velocity_limit(&mut self) -> Result<[f64; 2]> {
        self.velocity_limit
            .entry(self.inner.id)
            .or_try_insert_with(|_| self.inner.get_velocity_limit())
    }
    /// Set the current velocity limit of the motors
    ///
    /// # Arguments
    /// * `velocity_limit` - The velocity limit for each motor (motor_a, motor_b) in rad/s. Max values are typically around 400-600 rads/s.
    fn set_velocity_limit(&mut self, velocity_limit: [f64; 2]) -> Result<()> {
        let current_vel_limit = self.get_velocity_limit()?;

        if current_vel_limit != velocity_limit {
            self.inner.set_velocity_limit(velocity_limit)?;
            self.velocity_limit.insert(self.inner.id, velocity_limit);
        }
        Ok(())
    }

    /// Get the current "intensity limit" of the motors. Intensity_limit is the output of the velocity loop (input to the current loop)
    fn get_torque_limit(&mut self) -> Result<[f64; 2]> {
        self.torque_limit
            .entry(self.inner.id)
            .or_try_insert_with(|_| self.inner.get_torque_limit())
    }

    /// Set the current "intensity limit" of the motors. Intensity_limit is the output of the velocity loop (input to the current loop)
    fn set_torque_limit(&mut self, torque_limit: [f64; 2]) -> Result<()> {
        let current_torque_limit = self.get_torque_limit()?;

        if current_torque_limit != torque_limit {
            self.inner.set_torque_limit(torque_limit)?;
            self.torque_limit.insert(self.inner.id, torque_limit);
        }
        Ok(())
    }

    /// Get the current angle PID
    fn get_pid_gains(&mut self) -> Result<[PID; 2]> {
        self.inner.get_pid_gains()
    }
    /// Set the current angle PID
    fn set_pid_gains(&mut self, pid_gains: [PID; 2]) -> Result<()> {
        self.inner.set_pid_gains(pid_gains)
    }
    /// Get the BoardState register
    fn get_board_state(&mut self) -> Result<u8> {
        self.inner.get_board_state()
    }
    /// Set the BoardState register
    fn set_board_state(&mut self, state: u8) -> Result<()> {
        self.inner.set_board_state(state)
    }
}

fn find_raw_motor_offsets(
    controller: &mut Orbita2dController,
    motors_offset: [f64; 2],
    inverted_axis: [bool; 2],
    motors_ratio: [f64; 2],
) -> Result<[f64; 2]> {
    info!("Finding raw motor offsets");

    // controller.disable_torque()?; //FIXME: It seems that the axis sensors do not work if the torque is enabled
    thread::sleep(Duration::from_millis(100));

    let current_position = controller.inner.get_current_position()?;
    let mut current_axis_sensors = controller.inner.get_axis_sensors()?;

    // remove motor offsets
    current_axis_sensors
        .iter_mut()
        .enumerate()
        .for_each(|(i, val)| {
            *val = wrap_to_pi(*val - motors_offset[i]);
        });

    let mut current_axis_position = controller
        .kinematics
        .compute_inverse_kinematics(current_axis_sensors);

    debug!("Current positions: {:?} current_axis_position: {:?} inverted_axis: {:?} motors_ratio: {:?} axis_offsets: {:?}", current_position, current_axis_position, inverted_axis, motors_ratio, motors_offset);
    debug!(
        "AXIS SENSOR: {:?}, AXIS POS: {:?}",
        current_axis_sensors, current_axis_position
    ); //TODO Handle NaN

    current_axis_position
        .iter_mut()
        .enumerate()
        .for_each(|(i, val)| {
            *val += current_position[i];
        });

    info!("OFFSETS: {:?}", current_axis_position);

    Ok(current_axis_position)
}

// orbita2d already includes the motor offsets necessary to reach the zero position of the obrbita2d
// this function calculates the remaining offsets necessary to reach the zero position related to the robot's arm kinematics
fn find_additional_motor_offsets(
    controller: &mut Orbita2dController,
    motors_offset: [f64; 2],
    _inverted_axis: [bool; 2],
    _motors_ratio: [f64; 2],
) -> Result<[f64; 2]> {
    info!("Finding additional motor offsets");

    // controller.disable_torque()?; //FIXME: It seems that the axis sensors do not work if the torque is enabled
    thread::sleep(Duration::from_millis(100));

    // the simplest way of finding these offsets would be to
    // let motor_position_offset = ik(motors_offset)
    // but this solution does not behave well as it can lead to certain joints
    // making a full turn at the init

    // instead of this simple solution we will do it in multiple steps:
    // first calcualte the current position of the orbita2d's axis (with zeros offsets removed in firmeware)
    let current_position = controller.inner.get_current_position()?;
    let current_axis_position = controller
        .kinematics
        .compute_forward_kinematics(current_position);

    // then substract the motor offsets (from the config file) from the current position
    // and wrap the values to [-pi, pi]
    let mut axis_sensor_offsets = motors_offset;
    axis_sensor_offsets
        .iter_mut()
        .enumerate()
        .for_each(|(i, val)| {
            *val = wrap_to_pi(-*val - current_axis_position[i]);
        });
    // calculate the inverse kinematics of this new axis position with new zeros
    let mut motor_position_offset = controller
        .kinematics
        .compute_inverse_kinematics(axis_sensor_offsets);
    // and calcualte the new offsets by adding the current position
    motor_position_offset
        .iter_mut()
        .enumerate()
        .for_each(|(i, val)| {
            *val += current_position[i];
        });

    info!("OFFSETS: {:?}", motor_position_offset);

    // IMPORTANT:
    // this solution will lose the absolute position of the motors that was reached at the init
    Ok(motor_position_offset)
}

// function wrapping an angle in radians to
// the range [-pi, pi]
fn wrap_to_pi(angle: f64) -> f64 {
    (((angle + PI) % (2.0 * PI)) + (2.0 * PI)) % (2.0 * PI) - PI
}

#[cfg(test)]
mod tests {
    use crate::poulpe::wrap_to_pi;
    use crate::Orbita2dConfig;

    // Custom assert_approx_eq macro to compare floating point values within a tolerance
    macro_rules! assert_approx_eq {
        ($left:expr, $right:expr, $tol:expr) => {
            assert!(
                ($left - $right).abs() < $tol,
                "{} is not approximately equal to {}, within a tolerance of {}",
                $left,
                $right,
                $tol
            );
        };
    }

    #[test]
    fn test_wrap_to_pi_positive() {
        assert_approx_eq!(
            wrap_to_pi(3.0 * std::f64::consts::PI),
            -std::f64::consts::PI,
            1e-5
        );
        assert_approx_eq!(wrap_to_pi(4.0 * std::f64::consts::PI), 0.0, 1e-5);
    }

    #[test]
    fn test_wrap_to_pi_negative() {
        assert_approx_eq!(
            wrap_to_pi(-3.0 * std::f64::consts::PI),
            -std::f64::consts::PI,
            1e-5
        );
        assert_approx_eq!(wrap_to_pi(-4.0 * std::f64::consts::PI), 0.0, 1e-5);
    }

    #[test]
    fn test_wrap_to_pi_within_range() {
        assert_approx_eq!(
            wrap_to_pi(std::f64::consts::PI - 0.001),
            std::f64::consts::PI - 0.001,
            1e-5
        );
        assert_approx_eq!(
            wrap_to_pi(-std::f64::consts::PI + 0.1),
            -std::f64::consts::PI + 0.1,
            1e-5
        );
        assert_approx_eq!(wrap_to_pi(0.0), 0.0, 1e-5);
    }

    #[test]
    fn test_wrap_to_pi_small_angles() {
        assert_approx_eq!(
            wrap_to_pi(std::f64::consts::FRAC_PI_4),
            std::f64::consts::FRAC_PI_4,
            1e-5
        );
        assert_approx_eq!(
            wrap_to_pi(-std::f64::consts::FRAC_PI_4),
            -std::f64::consts::FRAC_PI_4,
            1e-5
        );
    }

    #[test]
    fn test_wrap_to_pi_large_angles() {
        assert_approx_eq!(wrap_to_pi(100.0 * std::f64::consts::PI), 0.0, 1e-5);
        assert_approx_eq!(wrap_to_pi(-100.0 * std::f64::consts::PI), 0.0, 1e-5);
    }

    #[test]
    fn test_wrap_to_pi_random_values() {
        assert_approx_eq!(wrap_to_pi(1.2), 1.2, 1e-5);
        assert_approx_eq!(wrap_to_pi(-5.7), 0.583185307179586, 1e-5);
    }

    #[test]
    fn parse_config_no_firmware_zero() {
        let s = "!Poulpe
        serial_port: /dev/ttyACM0
        id: 42
        motors_offset:
        - 0.0
        - 0.0
        motors_ratio:
        - 1.0
        - 1.0
        inverted_axes:
        - false
        - false
        orientation_limits: null
        use_cache: true
        ";

        let config: Result<Orbita2dConfig, _> = serde_yaml::from_str(s);
        assert!(config.is_ok());

        let config = config.unwrap();

        if let Orbita2dConfig::Poulpe(config) = config {
            assert_eq!(config.serial_port, "/dev/ttyACM0");
            assert_eq!(config.id, 42);
            assert_eq!(config.motors_offset, [0.0, 0.0]);
            assert_eq!(config.motors_ratio, [1.0, 1.0]);
            assert!(config.firmware_zero.is_none());
            assert!(config.orientation_limits.is_none());
            assert!(config.use_cache);
        } else {
            panic!("Wrong config type");
        }
    }

    #[test]
    fn parse_config() {
        let s = "!Poulpe
        serial_port: /dev/ttyACM0
        id: 42
        firmware_zero: false
        motors_offset:
        - 0.0
        - 0.0
        motors_ratio:
        - 1.0
        - 1.0
        inverted_axes:
        - false
        - false
        orientation_limits: null
        use_cache: true
        ";

        let config: Result<Orbita2dConfig, _> = serde_yaml::from_str(s);
        assert!(config.is_ok());

        let config = config.unwrap();

        if let Orbita2dConfig::Poulpe(config) = config {
            assert_eq!(config.serial_port, "/dev/ttyACM0");
            assert_eq!(config.id, 42);
            assert_eq!(config.motors_offset, [0.0, 0.0]);
            assert_eq!(config.motors_ratio, [1.0, 1.0]);
            assert_eq!(config.firmware_zero, Some(false));
            assert!(config.orientation_limits.is_none());
            assert!(config.use_cache);
        } else {
            panic!("Wrong config type");
        }
    }
    #[test]
    fn parse_config_file() {
        let f = std::fs::File::open("./config/left_shoulder_poulpe.yaml").unwrap();
        let config: Orbita2dConfig = serde_yaml::from_reader(f).unwrap();

        if let Orbita2dConfig::Poulpe(config) = config {
            assert_eq!(config.serial_port, "/dev/left_shoulder");
            assert_eq!(config.id, 83);
            assert_eq!(config.motors_offset, [0.0, 0.0]);
            assert_eq!(config.motors_ratio, [45.018, 45.018]);
            assert!(config.orientation_limits.is_none());
            assert!(config.use_cache);
        } else {
            panic!("Wrong config type");
        }
    }
}
