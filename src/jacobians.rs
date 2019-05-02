extern crate nalgebra as na;

use na::{Vector2,Matrix2x3,DMatrix, Matrix3x4};
use crate::MatrixData;
use crate::image::Image;
use std::boxed::Box;
use crate::camera::intrinsics::Intrinsics;

pub fn image_jacobian(gradient_x : DMatrix<MatrixData>, gradient_y : DMatrix<MatrixData>, px : usize, py : usize) -> Vector2<MatrixData> {
    let index = (py, px);
    let gx = *gradient_x.index(index);
    let gy = *gradient_y.index(index);
    return Vector2::<MatrixData>::new(gx,gy);
}

#[allow(non_snake_case)]
pub fn perspective_jacobian(K : &Intrinsics, world_points: &DMatrix<MatrixData>) -> Box<Vec<Matrix2x3<MatrixData>>> {
    let N = world_points.ncols();
    let fx = K.fx();
    let fy = K.fy();
    let mut persp_jacobians : Vec<Matrix2x3<MatrixData>> = Vec::with_capacity(N);
    for P in world_points.column_iter() {
        let X = *P.index((0,0));
        let Y = *P.index((1,0));
        let Z = *P.index((2,0));
        let Z_sqrd = Z*Z;

        let (v00, v11, v02, v12) =
            if Z != 0.0 && Z_sqrd.is_finite() {
                (fx / Z, fy / Z, (-fx * X) / Z_sqrd, (-fy * Y) / Z_sqrd)
            } else {
                (0.0, 0.0, 0.0, 0.0)
            };

        let persp_jacobian = Matrix2x3::<MatrixData>::new(v00,0.0,v02,
                                                          0.0, v11, v12);
        persp_jacobians.push(persp_jacobian);

    }
    return Box::new(persp_jacobians);
}