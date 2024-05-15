use serde::{Deserialize, Serialize};

use crate::{Orbita2dController, Orbita2dFeedback, Orbita2dMotorController, PID};

/// Fake motors implementation, only used for testing
struct FakeMotors {
    torque_on: [bool; 2],

    current_position: [f64; 2],
    current_velocity: [f64; 2],
    current_torque: [f64; 2],

    target_position: [f64; 2],

    velocity_limit: [f64; 2],
    torque_limit: [f64; 2],
    pid_gains: [PID; 2],
}

#[derive(Debug, Deserialize, Serialize)]
pub struct FakeConfig {
    /// Inverted axes [ring, center]
    pub inverted_axes: [bool; 2],
}

impl Default for FakeMotors {
    fn default() -> Self {
        Self {
            torque_on: [false, false],

            current_position: [0.0, 0.0],
            current_velocity: [f64::NAN, f64::NAN],
            current_torque: [f64::NAN, f64::NAN],

            target_position: [0.0, 0.0],

            velocity_limit: [f64::INFINITY, f64::INFINITY],
            torque_limit: [f64::INFINITY, f64::INFINITY],
            pid_gains: [
                PID {
                    p: f64::NAN,
                    i: f64::NAN,
                    d: f64::NAN,
                },
                PID {
                    p: f64::NAN,
                    i: f64::NAN,
                    d: f64::NAN,
                },
            ],
        }
    }
}

impl Orbita2dController {
    /// Create a new Orbita2dController with fake motors, only meant for testing.
    ///
    /// This implementation does not communicate with any real motor.
    /// It is not meant to mimic a real motor behavior.
    /// But instead implements the simplest model for testing purposes.
    /// More precisely, the motors current position directly teleports to the target when the torque is on.
    ///
    /// Other registers such as velocity, torque, limits, pid gains are not supported in this version.
    pub fn with_fake_motors(motors_axes_inverted: [bool; 2]) -> Self {
        Self::new(
            Box::<FakeMotors>::default(),
            [1.0, 1.0],
            [0.0, 0.0],
            motors_axes_inverted,
            None,
        )
    }
}

impl Orbita2dMotorController for FakeMotors {
    fn name(&self) -> &'static str {
        "FakeMotors"
    }

    fn is_torque_on(&mut self) -> crate::Result<[bool; 2]> {
        Ok(self.torque_on)
    }

    fn set_torque(&mut self, on: [bool; 2]) -> crate::Result<()> {
        for (i, &on) in on.iter().enumerate() {
            if on != self.torque_on[i] {
                self.current_position[i] = self.target_position[i];
            }

            self.torque_on[i] = on;
        }

        Ok(())
    }

    fn get_current_position(&mut self) -> crate::Result<[f64; 2]> {
        Ok(self.current_position)
    }

    fn get_axis_sensors(&mut self) -> crate::Result<[f64; 2]> {
        Ok([0.0, 0.0]) //TODO
    }

    fn get_current_velocity(&mut self) -> crate::Result<[f64; 2]> {
        Ok(self.current_velocity)
    }

    fn get_current_torque(&mut self) -> crate::Result<[f64; 2]> {
        Ok(self.current_torque)
    }

    fn get_target_position(&mut self) -> crate::Result<[f64; 2]> {
        Ok(self.target_position)
    }

    fn set_target_position(&mut self, target_position: [f64; 2]) -> crate::Result<()> {
        self.target_position = target_position;

        for (i, &target_position) in target_position.iter().enumerate() {
            if self.torque_on[i] {
                self.current_position[i] = target_position;
            }
        }
        Ok(())
    }

    fn set_target_position_fb(
        &mut self,
        target_position: [f64; 2],
    ) -> crate::Result<Orbita2dFeedback> {
        self.target_position = target_position;

        for (i, &target_position) in target_position.iter().enumerate() {
            if self.torque_on[i] {
                self.current_position[i] = target_position;
            }
        }
        Ok(Orbita2dFeedback {
            orientation: self.current_position,
            // velocity: self.current_velocity,
            // torque: self.current_torque,
        })
    }

    fn get_velocity_limit(&mut self) -> crate::Result<[f64; 2]> {
        Ok(self.velocity_limit)
    }

    fn set_velocity_limit(&mut self, velocity_limit: [f64; 2]) -> crate::Result<()> {
        self.velocity_limit = velocity_limit;
        Ok(())
    }

    fn get_torque_limit(&mut self) -> crate::Result<[f64; 2]> {
        Ok(self.torque_limit)
    }

    fn set_torque_limit(&mut self, torque_limit: [f64; 2]) -> crate::Result<()> {
        self.torque_limit = torque_limit;
        Ok(())
    }

    fn get_pid_gains(&mut self) -> crate::Result<[crate::PID; 2]> {
        Ok(self.pid_gains)
    }

    fn set_pid_gains(&mut self, pid_gains: [crate::PID; 2]) -> crate::Result<()> {
        self.pid_gains = pid_gains;
        Ok(())
    }

    fn get_board_state(&mut self) -> crate::Result<u8> {
        Ok(0)
    }
    fn set_board_state(&mut self, _state: u8) -> crate::Result<()> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use crate::Orbita2dConfig;

    #[test]
    fn parse_config() {
        let s = "!FakeMotors
        inverted_axes:
            - false
            - false
        ";

        let config: Result<Orbita2dConfig, _> = serde_yaml::from_str(s);
        assert!(config.is_ok());

        let config = config.unwrap();

        if let Orbita2dConfig::FakeMotors(config) = config {
            assert_eq!(config.inverted_axes, [false, false]);
        } else {
            panic!("Wrong config type");
        }
    }
    #[test]
    fn parse_config_file() {
        let f = std::fs::File::open("./config/fake.yaml").unwrap();

        let config: Result<Orbita2dConfig, _> = serde_yaml::from_reader(f);
        assert!(config.is_ok());

        let config = config.unwrap();

        if let Orbita2dConfig::FakeMotors(config) = config {
            assert_eq!(config.inverted_axes, [false, false]);
        } else {
            panic!("Wrong config type");
        }
    }
}
