extern crate nalgebra as na;

use na::{DVector,Matrix,U4,VecStorage,Dynamic};
use crate::MatrixData;
use crate::camera::Camera;
use crate::image::Image;
use crate::numerics::column_major_index;

#[allow(non_snake_case)]
pub fn back_project(residuals: &DVector<MatrixData>,
                    valid_measurements_reference: &mut DVector<bool>,
                    valid_measurements_target: &mut DVector<bool>,
                    camera_reference: Camera,
                    depth_image_reference: Image,
                    depth_image_target: Image,
                    number_of_valid_measurements: usize,
                    image_width: usize,
                    image_height: usize,
                    max_depth: MatrixData)
    -> Matrix<MatrixData, U4, Dynamic, VecStorage<MatrixData, U4, Dynamic>> {
    let depth_direction =
        match camera_reference.intrinsics.fx().is_sign_positive() {
            true => 1.0,
            false => -1.0
        };
    let mut P_vec: Vec<MatrixData> = Vec::with_capacity(4*image_width*image_height);
    for y in 0..image_height {
        for x in 0..image_width {
            let flat_idx = column_major_index(y,x,image_width);
            let idx = (y,x);
            let mut depth_reference = *depth_image_reference.buffer.index(idx);
            let depth_target = *depth_image_target.buffer.index(idx);
            *valid_measurements_reference.index_mut(flat_idx) = true;
            *valid_measurements_target.index_mut(flat_idx) = true;

            if depth_reference == 0.0 {
                depth_reference = depth_direction*max_depth;
                *valid_measurements_reference.index_mut(flat_idx) = false;
            }
            if depth_target == 0.0 {
                *valid_measurements_target.index_mut(flat_idx) = false;
            }

            let Z = depth_reference;
            //TODO: @Investigate -> Post in forum if there is a better way
            let P = camera_reference.back_project_pixel(x as MatrixData,y as MatrixData,Z);
            P_vec.push(*P.index(0));
            P_vec.push(*P.index(1));
            P_vec.push(*P.index(2));
            P_vec.push(1.0);
        }
    }

    let back_projections = Matrix::<MatrixData, U4, Dynamic, VecStorage<MatrixData, U4, Dynamic>>::from_vec(P_vec);
    return back_projections;
}

//TODO gauss_newton_step

//TODO residual
