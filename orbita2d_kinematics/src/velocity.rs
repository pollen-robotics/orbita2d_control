//! Velocity kinematics (forward/inverse) computation

use crate::{Orbita2dKinematicsModel, Vector2f64};

impl Orbita2dKinematicsModel {
    /// Compute the forward velocity
    ///
    /// # Arguments
    /// * input_velocity - Input velocity on each motor (motor a, motor b)
    /// # Returns
    /// * Output oriented velocity (ring, center)
    pub fn compute_output_velocity(&self, input_velocity: [f64; 2]) -> [f64; 2] {
        let input_velocity = Vector2f64::from_row_slice(&input_velocity);
        let res = self.inv_mat * input_velocity;
        [res[0], res[1]]
    }
    /// Compute the inverse velocity
    ///
    /// # Arguments
    /// * output_velocity - target output oriented velocity (ring, center)
    /// # Returns
    /// * Necessary input velocity on each motor (motor a, motor b)
    pub fn compute_input_velocity(&self, output_velocity: [f64; 2]) -> [f64; 2] {
        let output_velocity = Vector2f64::from_row_slice(&output_velocity);
        let res = self.mat * output_velocity;
        [res[0], res[1]]
    }
}
