extern crate nalgebra as na;

pub mod intrinsics;

use na::DMatrix;
use intrinsics::Intrinsics;
use crate::{MatrixData,NormalizedImageCoordinates};
use self::na::Vector3;

//TODO: @Invest -> refactor intrinsics into Camera

#[allow(non_snake_case)]
pub struct Camera {
    pub intrinsics : Intrinsics
    //SE3 : Matrix4<MatrixData>
}

#[allow(non_snake_case)]
impl Camera {

    pub fn new(intrinsics : Intrinsics) -> Camera {
        Camera {intrinsics}
    }

    pub fn apply_perspective_projection(&self, world_points: &DMatrix<MatrixData>)
        -> NormalizedImageCoordinates {
            let mut persp_points = self.intrinsics.K * world_points;
            let N = persp_points.ncols();
            for i in 0..N {
                let z = *persp_points.index((2, i));
                *persp_points.index_mut((0, i)) = *persp_points.index((0, i)) / z;
                *persp_points.index_mut((1, i)) = *persp_points.index((1, i)) / z;
                *persp_points.index_mut((2, i)) = 1.0;
        }
        persp_points
    }

    pub fn back_project_pixel(&self, x : MatrixData, y : MatrixData, Z : MatrixData) -> Vector3<MatrixData> {
        let nic = Vector3::<MatrixData>::new(x,y,1.0);
        let X = self.intrinsics.K_inv*nic;
        Z*X
    }

}