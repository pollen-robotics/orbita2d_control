extern crate nalgebra as na;

use na::{Matrix2, Vector2};

pub type Matrix2x2f64 = Matrix2<f64>;
pub type Vector2f64 = Vector2<f64>;

pub mod torque;

pub struct Orbita2dKinematicsModel {
    pub ratio_a: f64,
    pub ratio_b: f64,
    pub mat: Matrix2x2f64,
    pub inv_mat: Matrix2x2f64,
}

impl Orbita2dKinematicsModel {
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

    pub fn forward_kinematics(&self, angles: [f64; 2]) -> [f64; 2] {
        let a = Vector2f64::from_row_slice(&angles);
        let ret = self.inv_mat * a;
        [ret[0], ret[1]]
    }

    pub fn inverse_kinematics(&self, target: [f64; 2]) -> [f64; 2] {
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
    fn forward_kinematics() {
        let kin = Orbita2dKinematicsModel::new(1.0, 1.0);

        let res = kin.forward_kinematics([0.0, 0.0]);
        assert_eq!(res, [0.0, 0.0]);

        let res = kin.forward_kinematics([1.0, 0.0]);
        assert_eq!(res, [0.5, 0.5]);

        let res = kin.forward_kinematics([0.0, 1.0]);
        assert_eq!(res, [0.5, -0.5]);
    }

    #[test]
    fn inverse_kinematics() {
        let kin = Orbita2dKinematicsModel::new(1.0, 1.0);

        let res = kin.inverse_kinematics([0.0, 0.0]);
        assert_eq!(res, [0.0, 0.0]);

        let res = kin.inverse_kinematics([0.5, 0.5]);
        assert_eq!(res, [1.0, 0.0]);

        let res = kin.inverse_kinematics([0.5, -0.5]);
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

        let target = kin.forward_kinematics(angles);
        let reconstructed_angles = kin.inverse_kinematics(target);

        let error = (Vector2f64::from_row_slice(&angles)
            - Vector2f64::from_row_slice(&reconstructed_angles))
        .norm();
        assert!(error < 1e-6);
    }
}
