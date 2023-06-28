use orbita2d_kinematics::Orbita2dKinematicsModel;

pub type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

mod serial;

#[derive(Debug)]
pub struct PID {
    pub p: f64,
    pub i: f64,
    pub d: f64,
}

pub struct Orbita2dController {
    inner: Box<dyn Orbita2dMotorController>,
    kinematics: Orbita2dKinematicsModel,

    motors_offset: [f64; 2],
    orientation_limits: Option<[AngleLimit; 2]>,
}

#[derive(Debug)]
pub struct AngleLimit {
    min: f64,
    max: f64,
}

impl Orbita2dController {
    fn new(
        motors_controller: Box<dyn Orbita2dMotorController>,
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

    /// Torque ON/OFF
    pub fn is_torque_on(&mut self) -> Result<bool> {
        self.inner.is_torque_on()
    }
    pub fn enable_torque(&mut self, reset_target: bool) -> Result<()> {
        if reset_target {
            let current_pos = self.get_current_orientation()?;
            self.set_target_orientation(current_pos)?;
        }
        self.set_torque(true)?;
        Ok(())
    }
    pub fn disable_torque(&mut self) -> Result<()> {
        self.set_torque(false)
    }
    fn set_torque(&mut self, on: bool) -> Result<()> {
        self.inner.set_torque(on)
    }

    /// Read the current orientation (position/velocity/torque)
    /// In resp. rad, rad/s, Nm
    pub fn get_current_orientation(&mut self) -> Result<[f64; 2]> {
        let pos = self.inner.get_current_position()?;

        let pos = [
            pos[0] - self.motors_offset[0],
            pos[1] - self.motors_offset[1],
        ];

        Ok(self.kinematics.forward_kinematics(pos))
    }
    pub fn get_current_velocity(&mut self) -> Result<[f64; 2]> {
        todo!()
    }
    pub fn get_current_torque(&mut self) -> Result<[f64; 2]> {
        todo!()
    }

    /// Get/Set the desired orientation in radians
    pub fn get_target_orientation(&mut self) -> Result<[f64; 2]> {
        let pos = self.inner.get_target_position()?;

        let pos = [
            pos[0] - self.motors_offset[0],
            pos[1] - self.motors_offset[1],
        ];

        Ok(self.kinematics.forward_kinematics(pos))
    }
    pub fn set_target_orientation(&mut self, target_orientation: [f64; 2]) -> Result<()> {
        let target_orientation = match &self.orientation_limits {
            Some(limits) => [
                target_orientation[0].clamp(limits[0].min, limits[0].max),
                target_orientation[1].clamp(limits[1].min, limits[1].max),
            ],
            None => target_orientation,
        };

        let ik = self.kinematics.inverse_kinematics(target_orientation);

        let pos = [ik[0] + self.motors_offset[0], ik[1] + self.motors_offset[1]];

        self.inner.set_target_position(pos)
    }

    /// Controller parameters
    pub fn set_velocity_limit(&mut self, velocity_limit: [f64; 2]) -> Result<()> {
        self.inner.set_velocity_limit(velocity_limit)
    }
    pub fn set_torque_limit(&mut self, torque_limit: [f64; 2]) -> Result<()> {
        self.inner.set_torque_limit(torque_limit)
    }
    pub fn set_pid_gains(&mut self, pid_gains: PID) -> Result<()> {
        self.inner.set_pid_gains(pid_gains)
    }
}

pub trait Orbita2dMotorController {
    fn is_torque_on(&mut self) -> Result<bool>;
    fn set_torque(&mut self, on: bool) -> Result<()>;

    fn get_current_position(&mut self) -> Result<[f64; 2]>;
    fn get_current_velocity(&mut self) -> Result<[f64; 2]>;
    fn get_current_torque(&mut self) -> Result<[f64; 2]>;

    fn get_target_position(&mut self) -> Result<[f64; 2]>;
    fn set_target_position(&mut self, target_position: [f64; 2]) -> Result<()>;

    fn set_velocity_limit(&mut self, velocity_limit: [f64; 2]) -> Result<()>;
    fn set_torque_limit(&mut self, torque_limit: [f64; 2]) -> Result<()>;
    fn set_pid_gains(&mut self, pid_gains: PID) -> Result<()>;
}
