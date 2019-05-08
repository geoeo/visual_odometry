extern crate nalgebra as na;

use na::{Matrix3, Matrix3x4};
use crate::Float;

#[allow(non_snake_case)]
#[derive(Copy, Clone)]
pub struct Intrinsics {
    pub K: Matrix3x4<Float>,
    pub K_inv: Matrix3<Float>,
}

#[allow(non_snake_case)]
impl Intrinsics {
    pub fn new(fx: Float, fy: Float, ox: Float, oy: Float) -> Intrinsics {
        let K = Matrix3x4::<Float>::new(fx, 0.0, ox, 0.0,
                                        0.0, fy, oy, 0.0,
                                        0.0, 0.0, 1.0, 0.0);
        let K_inv = Intrinsics::invert(K);
        Intrinsics { K, K_inv }
    }

    pub fn invert(K: Matrix3x4<Float>) -> Matrix3<Float> {
        let fx = *K.index((0, 0));
        let fy = *K.index((1, 1));
        let ox = *K.index((0, 2));
        let oy = *K.index((1, 2));
        let fx_inv = 1.0 / fx;
        let fy_inv = 1.0 / fy;
        let ox_inv = -ox / fx;
        let oy_inv = -oy / fy;

        Matrix3::<Float>::new(fx_inv, 0.0, ox_inv,
                              0.0, fy_inv, oy_inv,
                              0.0, 0.0, 1.0)
    }

    pub fn fx(&self) -> Float { return *self.K.index((0, 0)); }
    pub fn fy(&self) -> Float { return *self.K.index((1, 1)); }
    pub fn ox(&self) -> Float { return *self.K.index((0, 2)); }
    pub fn oy(&self) -> Float { return *self.K.index((1, 2)); }
}

