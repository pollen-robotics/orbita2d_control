use std::{thread, time::Duration};

use std::f64::consts::PI;

use log::{debug, error, info, warn};

use crate::{Orbita2dController, Orbita2dFeedback, Orbita2dMotorController};
use motor_toolbox_rs::{Limit, Result, PID};

use poulpe_ethercat_grpc::client::PoulpeRemoteClient;

use serde::{Deserialize, Serialize};

#[derive(Debug, Deserialize, Serialize)]
/// Flipsky configuration
pub struct PoulpeEthercatConfig {
    /// url of the ethercat master grpc serverS
    pub url: String,
    /// Actuator id
    pub id: u8,
    /// Motors offset [motor_a, motor_b]
    pub motors_offset: [f64; 2],
    /// Motors ratio [motor_a, motor_b]
    pub motors_ratio: [f64; 2],
    /// Motors axes inverted [motor_a, motor_b]
    pub inverted_axes: [bool; 2],
    /// Orientation limits [motor_a, motor_b] (expressed in the corrected motor reference frame - after offset and inversion)
    pub orientation_limits: Option<[Limit; 2]>,
    /// Hardware zeros already set in the firmware
    pub firmware_zero: Option<bool>,
}

/// Orbita ethercat controller
struct Orbita2dPoulpeEthercatController {
    io: PoulpeRemoteClient,
    id: u16,
    axis_sensor_zeros: [Option<f64>; 2],
    raw_motor_offsets: [Option<f64>; 2],
}

impl Orbita2dController {
    pub fn with_poulpe_ethercat(
        url: &str,
        id: u16,
        motors_offset: [f64; 2],
        motors_ratio: [f64; 2],
        inverted_axes: [bool; 2],
        orientation_limits: Option<[Limit; 2]>,
        firmware_zero: Option<bool>,
    ) -> Result<Self> {
        let mut io = match PoulpeRemoteClient::connect(
            url.parse()?,
            vec![id],
            Duration::from_secs_f32(0.002),
        ) {
            Ok(io) => io,
            Err(e) => {
                error!("Error while connecting to the PoulpeRemoteClient: {:?}", e);
                return Err("Error while connecting to the PoulpeRemoteClient".into());
            }
        };
        
        // wait for the connection to be established
        let mut trials = 20; // 2s
        while io.get_state(id).is_err() {
            thread::sleep(Duration::from_millis(100));
            if trials == 0 {
                log::error!("Error: Timeout while connecting to the Orbita2d PoulpeRemoteClient with id {}", id);
                return Err("Error: Timeout while connecting to the Orbita2d  PoulpeRemoteClient".into());
            }
            trials -= 1;
        }
        log::info!("Connected to Orbita2d  PoulpeRemoteClient with id {}", id);


        // set the initial velocity and torque limit to 100%
        io.set_velocity_limit(id, [1.0; 2].to_vec());
        io.set_torque_limit(id, [1.0; 2].to_vec());

        //We can only change the mode if torque=off, then we ensure we are ProfilePositionMode
        io.set_mode_of_operation(id as u16, 1); //0=NoMode, 1=ProfilePositionMode, 3=ProfileVelocityMode, 4=ProfileTorqueMode

        let mut poulpe_controller = Orbita2dPoulpeEthercatController {
            io,
            id,
            axis_sensor_zeros: [None; 2],
            raw_motor_offsets: [None; 2],
        };

        //backup the raw motors offset from the file
        poulpe_controller.raw_motor_offsets = [Some(motors_offset[0]), Some(motors_offset[1])];

        // Read the axis sensor zeros once and save them
        let zeros = poulpe_controller.get_axis_sensor_zeros()?;
        poulpe_controller.axis_sensor_zeros = [Some(zeros[0]), Some(zeros[1])];

        info!(
            "Orbita2d PoulpeEthercatController:\n\t - url: {:?}\n\t - id: {:?} - zeros: {:?} - raw_motor_offsets: {:?}
",
            url, id, zeros, motors_offset
        );

        let mut controller = Self::new(
            Box::new(poulpe_controller),
            motors_ratio,
            motors_offset,
            inverted_axes,
            orientation_limits,
        );

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

impl Orbita2dMotorController for Orbita2dPoulpeEthercatController {
    // impl MotorsController<2> for Orbita2dPoulpeEthercatController {
    fn name(&self) -> String {
        "PoulpeEthercatController (Orbita2d)".to_string()
    }

    fn is_torque_on(&mut self) -> Result<[bool; 2]> {
        match self.io.get_torque_state(self.id) {
            Ok(state) => Ok([state, state]),
            Err(_) => Err("Error while getting torque state".into()),
        }
    }

    fn set_torque(&mut self, on: [bool; 2]) -> Result<()> {
        if on.iter().any(|&x| x) {
            self.io.turn_on(self.id);
        } else {
            self.io.turn_off(self.id);
        }
        Ok(())
    }

    fn get_current_position(&mut self) -> Result<[f64; 2]> {
        match self.io.get_position_actual_value(self.id) {
            Ok(position) => Ok([position[0] as f64, position[1] as f64]),
            Err(_) => Err("Error while getting position".into()),
        }
    }

    fn get_current_velocity(&mut self) -> Result<[f64; 2]> {
        match self.io.get_velocity_actual_value(self.id) {
            Ok(velocity) => Ok([velocity[0] as f64, velocity[1] as f64]),
            Err(_) => Err("Error while getting velocity".into()),
        }
    }

    fn get_current_torque(&mut self) -> Result<[f64; 2]> {
        match self.io.get_torque_actual_value(self.id) {
            Ok(torque) => Ok([torque[0] as f64, torque[1] as f64]),
            Err(_) => Err("Error while getting torque".into()),
        }
    }

    fn get_target_position(&mut self) -> Result<[f64; 2]> {
        match self.io.get_target_position(self.id) {
            Ok(position) => Ok([position[0] as f64, position[1] as f64]),
            Err(_) => Err("Error while getting target position".into()),
        }
    }

    fn set_target_position(&mut self, target_position: [f64; 2]) -> Result<()> {
        let target_position = target_position
            .iter()
            .map(|&x| x as f32)
            .collect::<Vec<f32>>();
        self.io.set_target_position(self.id, target_position);
        Ok(())
    }

    // fn get_target_velocity(&mut self) -> Result<[f64; 2]> {
    //     match self.io.get_target_velocity(self.id) {
    //         Ok(vel) => Ok([vel[0] as f64, vel[1] as f64]),
    //         Err(_) => Err("Error while getting target velocity".into()),
    //     }
    // }

    fn set_target_velocity(&mut self, vel: [f64; 2]) -> Result<()> {
        let target_velocity = vel.iter().map(|&x| x as f32).collect::<Vec<f32>>();
        self.io.set_target_velocity(self.id, target_velocity);
        Ok(())
    }

    // fn get_target_torque(&mut self) -> Result<[f64; 2]> {
    //     match self.io.get_target_torque(self.id) {
    //         Ok(vel) => Ok([vel[0] as f64, vel[1] as f64]),
    //         Err(_) => Err("Error while getting target torque".into()),
    //     }
    // }

    fn set_target_torque(&mut self, vel: [f64; 2]) -> Result<()> {
        let target_torque = vel.iter().map(|&x| x as f32).collect::<Vec<f32>>();
        self.io.set_target_torque(self.id, target_torque);
        Ok(())
    }

    fn set_target_position_fb(&mut self, target_position: [f64; 2]) -> Result<Orbita2dFeedback> {
        let target_position = target_position
            .iter()
            .map(|&x| x as f32)
            .collect::<Vec<f32>>();
        self.io.set_target_position(self.id, target_position);

        match self.io.get_position_actual_value(self.id) {
            Ok(position) => Ok(Orbita2dFeedback {
                orientation: [position[0] as f64, position[1] as f64],
                // velocity: self.io.get_velocity_actual_value(self.id),
                // torque: self.io.get_torque_actual_value(self.id),
            }),
            Err(_) => Err("Error while getting position".into()),
        }
    }

    fn get_axis_sensors(&mut self) -> Result<[f64; 2]> {
        match self.io.get_axis_sensors(self.id) {
            Ok(mut sensor) => {
                // substract the sensor zero and the axis offset
                // FIXME: inverted axis?
                for (i, s) in sensor.iter_mut().enumerate() {
                    if !self.axis_sensor_zeros[i].is_none() {
                        *s -= self.axis_sensor_zeros[i].unwrap() as f32;
                    }
                    if !self.raw_motor_offsets[i].is_none() {
                        *s -= self.raw_motor_offsets[i].unwrap() as f32;
                    }
                    *s = wrap_to_pi(*s as f64) as f32;
                }
                Ok([sensor[0] as f64, sensor[1] as f64])
            }
            Err(_) => Err("Error while getting axis sensors".into()),
        }
    }

    fn get_axis_sensor_zeros(&mut self) -> Result<[f64; 2]> {
        match self.io.get_axis_sensor_zeros(self.id) {
            Ok(sensor) => Ok([sensor[0] as f64, sensor[1] as f64]),
            Err(_) => Err("Error while getting axis sensor zeros".into()),
        }
    }

    fn get_velocity_limit(&mut self) -> Result<[f64; 2]> {
        match self.io.get_velocity_limit(self.id) {
            Ok(limit) => Ok([limit[0] as f64, limit[1] as f64]),
            Err(_) => Err("Error while getting velocity limit".into()),
        }
    }

    fn set_velocity_limit(&mut self, velocity_limit: [f64; 2]) -> Result<()> {
        let velocity_limit = velocity_limit
            .iter()
            .map(|&x| x as f32)
            .collect::<Vec<f32>>();
        self.io.set_velocity_limit(self.id, velocity_limit);
        Ok(())
    }

    fn get_torque_limit(&mut self) -> Result<[f64; 2]> {
        match self.io.get_torque_limit(self.id) {
            Ok(limit) => Ok([limit[0] as f64, limit[1] as f64]),
            Err(_) => Err("Error while getting torque limit".into()),
        }
    }

    fn set_torque_limit(&mut self, torque_limit: [f64; 2]) -> Result<()> {
        let torque_limit = torque_limit.iter().map(|&x| x as f32).collect::<Vec<f32>>();
        self.io.set_torque_limit(self.id, torque_limit);
        Ok(())
    }

    fn get_pid_gains(&mut self) -> Result<[PID; 2]> {
        Ok([PID {
            p: 0.0,
            i: 0.0,
            d: 0.0,
        }; 2])
    }

    fn set_pid_gains(&mut self, _pid_gains: [PID; 2]) -> Result<()> {
        Ok(())
    }

    fn get_board_state(&mut self) -> Result<u8> {
        match self.io.get_state(self.id) {
            Ok(state) => Ok(state as u8),
            Err(_) => Err("Error while getting board state".into()),
        }
    }

    fn set_board_state(&mut self, _state: u8) -> Result<()> {
        Ok(())
    }

    fn set_control_mode(&mut self, mode: [u8; 2]) -> Result<()> {
        if mode[0] != mode[1] {
            return Err("Error, invalid control mode".into());
        }
        self.io.set_mode_of_operation(self.id, mode[0] as u32);
        Ok(())
    }

    fn get_control_mode(&mut self) -> Result<[u8; 2]> {
        match self.io.get_mode_of_operation(self.id) {
            Ok(mode) => Ok([mode as u8, mode as u8]), //It is in fact the same for each axis (TODO make it board level?)
            Err(_) => Err("Error while getting mode of operation".into()),
        }
    }

    fn get_error_codes(&mut self) -> Result<[i32; 2]> {
        match self.io.get_error_codes(self.id) {
            Ok(codes) => Ok([codes[0], codes[1]]),
            Err(_) => Err("Error while getting error codes".into()),
        }
    }

    fn get_motor_temperatures(&mut self) -> Result<[f64; 2]> {
        match self.io.get_motor_temperatures(self.id) {
            Ok(temp) => Ok([temp[0] as f64, temp[1] as f64]),
            Err(_) => Err("Error while getting motor temperatures".into()),
        }
    }

    fn get_board_temperatures(&mut self) -> Result<[f64; 2]> {
        match self.io.get_board_temperatures(self.id) {
            Ok(temp) => Ok([temp[0] as f64, temp[1] as f64]),
            Err(_) => Err("Error while getting board temperatures".into()),
        }
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

    if current_axis_sensors.iter().any(|&x| x.is_nan()) {
        error!("Axis sensors are NaN");
        return Err("Axis sensors are NaN".into());
    }

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
    // first calcualte the current position of the orbita2d's axis (with zeros offsets removed in firmware)
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
    use crate::poulpe_ethercat::wrap_to_pi;
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
        let s = "!PoulpeEthercat
        url: http://url.com
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
        ";

        let config: Result<Orbita2dConfig, _> = serde_yaml::from_str(s);
        assert!(config.is_ok());

        let config = config.unwrap();

        if let Orbita2dConfig::PoulpeEthercat(config) = config {
            assert_eq!(config.url, "http://url.com");
            assert_eq!(config.id, 42);
            assert_eq!(config.motors_offset, [0.0, 0.0]);
            assert_eq!(config.motors_ratio, [1.0, 1.0]);
            assert!(config.firmware_zero.is_none());
            assert!(config.orientation_limits.is_none());
        } else {
            panic!("Wrong config type");
        }
    }

    #[test]
    fn parse_config() {
        let s = "!PoulpeEthercat
        url: http://url.com
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
        ";

        let config: Result<Orbita2dConfig, _> = serde_yaml::from_str(s);
        assert!(config.is_ok());

        let config = config.unwrap();

        if let Orbita2dConfig::PoulpeEthercat(config) = config {
            assert_eq!(config.url, "http://url.com");
            assert_eq!(config.id, 42);
            assert_eq!(config.motors_offset, [0.0, 0.0]);
            assert_eq!(config.motors_ratio, [1.0, 1.0]);
            assert_eq!(config.firmware_zero, Some(false));
            assert!(config.orientation_limits.is_none());
        } else {
            panic!("Wrong config type");
        }
    }
    #[test]
    fn parse_config_file() {
        let f = std::fs::File::open("./config/ethercat_poulpe.yaml").unwrap();
        let config: Orbita2dConfig = serde_yaml::from_reader(f).unwrap();

        if let Orbita2dConfig::PoulpeEthercat(config) = config {
            assert_eq!(config.url, "http://127.0.0.1:50098");
            assert_eq!(config.id, 1);
            assert_eq!(config.motors_offset, [0.0, 0.0]);
            assert_eq!(config.motors_ratio, [1.0, 1.0]);
            assert!(config.orientation_limits.is_none());
        } else {
            panic!("Wrong config type");
        }
    }
}
