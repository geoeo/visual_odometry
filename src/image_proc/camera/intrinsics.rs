extern crate nalgebra as na;

use na::Matrix3;

#[allow(non_snake_case)]
pub struct Intrinsics {
    K : Matrix3,
    K_inv : Matrix3
}

impl Intrinsics {
    pub fn new(fx)
}

