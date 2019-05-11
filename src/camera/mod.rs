extern crate nalgebra as na;

pub mod intrinsics;

use intrinsics::Intrinsics;
use crate::{Float, NormalizedImageCoordinates, HomogeneousBackProjections};
use self::na::Vector3;

//TODO: @Invest -> refactor intrinsics into Camera

#[allow(non_snake_case)]
#[derive(Copy, Clone)]
pub struct Camera {
    pub intrinsics : Intrinsics
    //SE3 : Matrix4<MatrixData>
}

#[allow(non_snake_case)]
impl Camera {

    pub fn new(intrinsics : Intrinsics) -> Camera {
        Camera {intrinsics}
    }

    pub fn apply_perspective_projection(&self, world_points: &HomogeneousBackProjections)
        -> NormalizedImageCoordinates {
            let mut persp_points = self.intrinsics.K * world_points;
            let N = persp_points.ncols();
            for i in 0..N {
                let x = *persp_points.index((0, i));
                let y = *persp_points.index((1, i));
                let z = *persp_points.index((2, i));
                *persp_points.index_mut((0, i)) = x / z;
                *persp_points.index_mut((1, i)) = y / z;
                *persp_points.index_mut((2, i)) = 1.0;
        }
        persp_points
    }

    pub fn back_project_pixel(&self, x : Float, y : Float, Z : Float) -> Vector3<Float> {
        let nic = Vector3::<Float>::new(x, y, 1.0);
        let X = self.intrinsics.K_inv*nic;
        Z*X
    }

}