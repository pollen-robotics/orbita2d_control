//! Kinematics model for Orbita2d
//!
//! TODO: Add the maths behind the kinematics model here!
//!
//! We should explain how we find the mat matrix.

extern crate nalgebra as na;

use na::{Matrix2, Vector2};

type Matrix2x2f64 = Matrix2<f64>;
type Vector2f64 = Vector2<f64>;

mod torque;
mod velocity;

#[derive(Debug)]
/// Kinematics model for Orbita2d
pub struct Orbita2dKinematicsModel {
    pub ratio_a: f64,
    pub ratio_b: f64,

    mat: Matrix2x2f64,
    inv_mat: Matrix2x2f64,
}

impl Orbita2dKinematicsModel {
    /// Create a new kinematics model based on the reduction ratio for motor a and b
    ///
    /// # Arguments
    /// * ratio_a - Reduction ratio for motor a
    /// * ratio_b - Reduction ratio for motor b
    pub fn new(ratio_a: f64, ratio_b: f64) -> Self {
        let mat = Matrix2x2f64::new(ratio_a, ratio_a, ratio_b, -ratio_b);
        let inv_mat = mat.try_inverse().unwrap();

        Self {
            ratio_a,
            ratio_b,
            mat,
            inv_mat,
        }
    }
    /// Compute the forward kinematics
    ///
    /// Compute the output 2d orientation give the motor angles (in radians)
    ///
    /// # Arguments
    /// * motor_angles - Motor angles in radians
    /// # Returns
    /// * Output 2d orientation in radians
    pub fn compute_forward_kinematics(&self, motor_angles: [f64; 2]) -> [f64; 2] {
        let a = Vector2f64::from_row_slice(&motor_angles);
        let ret = self.inv_mat * a;
        [ret[0], ret[1]]
    }
    /// Compute the inverse kinematics
    ///
    /// Compute the motor angles (in radians) given the target 2d orientation
    ///
    /// # Arguments
    /// * target - Target 2d orientation in radians
    /// # Returns
    /// * Motor angles in radians
    pub fn compute_inverse_kinematics(&self, target: [f64; 2]) -> [f64; 2] {
        let t = Vector2f64::from_row_slice(&target);
        let ret = self.mat * t;
        [ret[0], ret[1]]
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use rand::Rng;

    use crate::{Orbita2dKinematicsModel, Vector2f64};

    #[test]
    fn roll_only() {
        let mut rng = rand::thread_rng();

        let kin = Orbita2dKinematicsModel::new(1.0, 1.0);

        let alpha: f64 = rng.gen();

        let res = kin.compute_forward_kinematics([alpha, -alpha]);
        assert!(res[0] == 0.0);

        let res = kin.compute_forward_kinematics([-alpha, alpha]);
        assert!(res[0] == 0.0);

        let pitch: f64 = rng.gen();
        let res = kin.compute_inverse_kinematics([0.0, pitch]);
        assert_eq!(res[0], -res[1]);
    }

    #[test]
    fn pitch_only() {
        let mut rng = rand::thread_rng();

        let kin = Orbita2dKinematicsModel::new(1.0, 1.0);

        let alpha: f64 = rng.gen();

        let res = kin.compute_forward_kinematics([alpha, alpha]);
        assert!(res[1] == 0.0);

        let roll: f64 = rng.gen();
        let res = kin.compute_inverse_kinematics([roll, 0.0]);
        assert_eq!(res[0], res[1]);
    }

    #[test]
    fn forward_kinematics() {
        let kin = Orbita2dKinematicsModel::new(1.0, 1.0);

        let res = kin.compute_forward_kinematics([0.0, 0.0]);
        assert_eq!(res, [0.0, 0.0]);

        let res = kin.compute_forward_kinematics([1.0, 0.0]);
        assert_eq!(res, [0.5, 0.5]);

        let res = kin.compute_forward_kinematics([0.0, 1.0]);
        assert_eq!(res, [0.5, -0.5]);
    }

    #[test]
    fn inverse_kinematics() {
        let kin = Orbita2dKinematicsModel::new(1.0, 1.0);

        let res = kin.compute_inverse_kinematics([0.0, 0.0]);
        assert_eq!(res, [0.0, 0.0]);

        let res = kin.compute_inverse_kinematics([0.5, 0.5]);
        assert_eq!(res, [1.0, 0.0]);

        let res = kin.compute_inverse_kinematics([0.5, -0.5]);
        assert_eq!(res, [0.0, 1.0]);
    }

    #[test]
    fn inverse_forward_reciprocity() {
        let mut rng = rand::thread_rng();

        let ratio_a: f64 = rng.gen();
        let ratio_b: f64 = rng.gen();

        let kin = Orbita2dKinematicsModel::new(ratio_a, ratio_b);

        let angles = [
            rng.gen::<f64>() * 2.0 * PI - PI,
            rng.gen::<f64>() * 2.0 * PI - PI,
        ];

        let target = kin.compute_forward_kinematics(angles);
        let reconstructed_angles = kin.compute_inverse_kinematics(target);

        let error = (Vector2f64::from_row_slice(&angles)
            - Vector2f64::from_row_slice(&reconstructed_angles))
        .norm();
        assert!(error < 1e-6);
    }
}
