extern crate nalgebra as na;

use na::{DVector,DMatrix,Vector6,Matrix6,Matrix3x6,Matrix2x3};
use crate::{MatrixData,NormalizedImageCoordinates,HomogeneousBackProjections};
use crate::camera::Camera;
use crate::numerics::column_major_index;
use crate::jacobians::*;

// This is not used in the Gauss-Newton estimation loop
// As such it is allocating.
#[allow(non_snake_case)]
pub fn back_project(valid_measurements_reference: &mut DVector<bool>,
                    valid_measurements_target: &mut DVector<bool>,
                    camera_reference: Camera,
                    depth_image_reference: &DMatrix<MatrixData>,
                    depth_image_target: &DMatrix<MatrixData>,
                    image_width: usize,
                    image_height: usize,
                    max_depth: MatrixData)
    -> HomogeneousBackProjections {
    let depth_direction =
        match camera_reference.intrinsics.fx().is_sign_positive() {
            true => 1.0,
            false => -1.0
        };
    let mut P_vec: Vec<MatrixData> = Vec::with_capacity(4*image_width*image_height);
    for x in 0..image_width {
        for y in 0..image_height {
            let flat_idx = column_major_index(y,x,image_width);
            let idx = (y,x);
            let mut depth_reference = *depth_image_reference.index(idx);
            let depth_target = *depth_image_target.index(idx);
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

            let P = camera_reference.back_project_pixel(x as MatrixData,y as MatrixData,Z);
            P_vec.push(*P.index(0));
            P_vec.push(*P.index(1));
            P_vec.push(*P.index(2));
            P_vec.push(1.0);
        }
    }

    HomogeneousBackProjections::from_vec(P_vec)
}

//TODO @Investigate -> Trying stack allocated g and H
#[allow(non_snake_case)]
pub fn gauss_newton_step(residuals: Box<Vec<MatrixData>>,
                         valid_measurements_reference: &DVector<bool>,
                         valid_measurements_target: &DVector<bool>,
                         image_gradient_x_target: &DMatrix<MatrixData>,
                         image_gradient_y_target: &DMatrix<MatrixData>,
                         J_lie_vec: Box<Vec<Matrix3x6<MatrixData>>>,
                         J_pi_vec: Box<Vec<Matrix2x3<MatrixData>>>,
                         weights: &DVector<MatrixData>,
                         image_width: usize,
                         image_height: usize,
                         image_range_offset: usize)
    -> (Vector6<MatrixData>,Matrix6<MatrixData>) {
    let mut g = Vector6::<MatrixData>::zeros();
    let mut H = Matrix6::<MatrixData>::zeros();

    for x in image_range_offset..(image_width-image_range_offset) {
        for y in image_range_offset..(image_height-image_range_offset) {
            let flat_idx = column_major_index(y,x,image_width);
            if !*valid_measurements_reference.index(flat_idx) || !*valid_measurements_target.index(flat_idx) {
                continue;
            }
            let J_image = image_jacobian(image_gradient_x_target,image_gradient_y_target,x,y);
            let J_pi =J_pi_vec[flat_idx];
            let J_lie = J_lie_vec[flat_idx];

            let J = J_image*J_pi*J_lie;
            let J_t = J.transpose();
            let w_i = *weights.index(flat_idx);
            let residual = residuals[flat_idx];

            g += w_i*(-J_t*residual);
            H += w_i*(J_t*J);
        }
    }
    (g,H)
}

pub fn compute_residuals(mut residuals: Box<Vec<MatrixData>>,
                         valid_measurements_reference: &mut DVector<bool>,
                         valid_measurements_target: &mut DVector<bool>,
                         image_reference:  &DMatrix<MatrixData>,
                         image_target:  &DMatrix<MatrixData>,
                         projection_onto_target: NormalizedImageCoordinates,
                         image_width: usize,
                         image_height: usize,
                         image_range_offset: usize )
    -> Box<Vec<MatrixData>> {

    for x in image_range_offset..(image_width-image_range_offset) {
        for y in image_range_offset..(image_height-image_range_offset) {
            let flat_index = column_major_index(y,x,image_width);
            let idx_reference = (y,x);
            residuals[flat_index] = 0.0;
            if !*valid_measurements_reference.index(flat_index) || !*valid_measurements_target.index(flat_index) {
                continue;
            }
            let x_idx_target = (*projection_onto_target.index((0,flat_index))).floor() as usize;
            let y_idx_target = (*projection_onto_target.index((1,flat_index))).floor() as usize;
            let idx_target = (y_idx_target, x_idx_target);
            if !((image_range_offset < y) && (y < image_height - image_range_offset) &&
                (image_range_offset < x) && (x < image_width - image_range_offset)) {
                *valid_measurements_reference.index_mut(idx_reference) = false;
                continue;
            }
            valid_measurements_reference[flat_index] = true;
            residuals[flat_index] = *image_reference.index(idx_reference) - *image_target.index(idx_target);
        }
    }

    residuals
}