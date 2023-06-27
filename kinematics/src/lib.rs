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

    pub fn forward_kinematics(&self, angles: &[f64; 2]) -> [f64; 2] {
        let a = Vector2f64::from_row_slice(angles);
        let ret = self.inv_mat * a;
        [ret[0], ret[1]]
    }

    pub fn inverse_kinematics(&self, target: &[f64; 2]) -> [f64; 2] {
        let t = Vector2f64::from_row_slice(target);
        let ret = self.mat * t;
        [ret[0], ret[1]]
    }
}
