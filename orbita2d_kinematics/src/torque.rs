//! Torque kinematics (forward/inverse) computation

use crate::{Orbita2dKinematicsModel, Vector2f64};

impl Orbita2dKinematicsModel {
    /// Compute the forward torque
    ///
    /// # Arguments
    /// * input_torque - Input torque on each motor
    /// # Returns
    /// * Output oriented torque
    pub fn compute_output_torque(&self, input_torque: [f64; 2]) -> [f64; 2] {
        let input_torque = Vector2f64::from_row_slice(&input_torque);
        let res = self.mat * input_torque;
        [res[0], res[1]]
    }
    /// Compute the inverse torque
    ///
    /// # Arguments
    /// * output_torque - target output oriented torque
    /// # Returns
    /// * Necessary input torque on each motor
    pub fn compute_input_torque(&self, output_torque: [f64; 2]) -> [f64; 2] {
        let output_torque = Vector2f64::from_row_slice(&output_torque);
        let res = self.inv_mat * output_torque;
        [res[0], res[1]]
    }
}
