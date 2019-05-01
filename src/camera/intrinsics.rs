extern crate nalgebra as na;

use na::{Matrix3, Matrix3x4};
use crate::MatrixData;

#[allow(non_snake_case)]
pub struct Intrinsics {
    K: Matrix3x4<MatrixData>,
    K_inv: Matrix3<MatrixData>,
}

#[allow(non_snake_case)]
impl Intrinsics {
    pub fn new(fx: MatrixData, fy: MatrixData, ox: MatrixData, oy: MatrixData) -> Intrinsics {
        let K = Matrix3x4::<MatrixData>::new(fx, 0.0, ox, 0.0,
                                             0.0, fy, oy, 0.0,
                                             0.0, 0.0, 1.0, 0.0);
        let K_inv = Intrinsics::invert(&K);
        Intrinsics { K, K_inv }
    }

    pub fn invert(K: &Matrix3x4<MatrixData>) -> Matrix3<MatrixData> {
        let fx = *K.index((0, 0));
        let fy = *K.index((1, 1));
        let ox = *K.index((0, 2));
        let oy = *K.index((1, 2));
        let fx_inv = 1.0 / fx;
        let fy_inv = 1.0 / fy;
        let ox_inv = -ox / fx;
        let oy_inv = -oy / fy;

        return Matrix3::<MatrixData>::new(fx_inv, 0.0, ox_inv,
                                          0.0, fy_inv, oy_inv,
                                          0.0, 0.0, 1.0);
    }

    pub fn fx(&self) -> MatrixData { return *self.K.index((0, 0)); }
    pub fn fy(&self) -> MatrixData { return *self.K.index((1, 1)); }
    pub fn ox(&self) -> MatrixData { return *self.K.index((0, 2)); }
    pub fn oy(&self) -> MatrixData { return *self.K.index((1, 2)); }
}

