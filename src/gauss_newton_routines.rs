extern crate nalgebra as na;

use na::{DMatrix,Vector6,Matrix6,Matrix3x6,Matrix2x3};
use crate::{Float, NormalizedImageCoordinates, HomogeneousBackProjections};
use crate::camera::Camera;
use crate::numerics::column_major_index;
use crate::jacobians::*;
use crate::image_pyramid::Layer;

// @GPU
// This is not used in the Gauss-Newton estimation loop
// As such it is allocating.
#[allow(non_snake_case)]
pub fn back_project(camera_reference: Camera,
                    depth_image_reference: &DMatrix<Float>,
                    depth_image_target: &DMatrix<Float>,
                    image_width: usize,
                    image_height: usize,
                    max_depth: Float,
                    layer_index: u32)
                    -> (HomogeneousBackProjections, Vec<bool>, Vec<bool>) {

    let N = image_width*image_height;
    let mut P_vec: Vec<Float> = Vec::with_capacity(4 * N);
    let mut valid_measurements_reference: Vec<bool> = Vec::with_capacity(N);
    let mut valid_measurements_target: Vec<bool> = Vec::with_capacity(N);

    for x in 0..image_width {
        for y in 0..image_height {
            let (x_high, y_high) = Layer::generate_depth_coordiantes(layer_index, x, y);
            let idx = (y_high, x_high);
            let mut depth_reference = *depth_image_reference.index(idx);
            let depth_target = *depth_image_target.index(idx);

            if depth_reference == 0.0 {
                depth_reference = max_depth;
                valid_measurements_reference.push(false);
            } else {
                valid_measurements_reference.push(true);
            }

            if depth_target == 0.0 {
                valid_measurements_target.push(false);
            } else {
                valid_measurements_target.push(true);
            }

            let Z = depth_reference;

            let P = camera_reference.back_project_pixel(x as Float, y as Float, Z);
            P_vec.push(*P.index(0));
            P_vec.push(*P.index(1));
            P_vec.push(*P.index(2));
            P_vec.push(1.0);
        }
    }

    let back_projections = HomogeneousBackProjections::from_vec(P_vec);
    (back_projections, valid_measurements_reference, valid_measurements_target)
}

// @GPU
//TODO @Investigate -> Trying stack allocated g and H
#[allow(non_snake_case)]
pub fn gauss_newton_step(residuals: &Vec<Float>,
                         valid_measurements_reference: &Vec<bool>,
                         valid_measurements_target: &Vec<bool>,
                         image_gradient_x_target: &DMatrix<Float>,
                         image_gradient_y_target: &DMatrix<Float>,
                         J_lie_vec: &Vec<Matrix3x6<Float>>,
                         J_pi_vec: &Vec<Matrix2x3<Float>>,
                         weights: &Vec<Float>,
                         image_width: usize,
                         image_height: usize,
                         image_range_offset: usize)
                         -> (Vector6<Float>, Matrix6<Float>) {
    let mut g = Vector6::<Float>::zeros();
    let mut H = Matrix6::<Float>::zeros();

    for x in image_range_offset..(image_width-image_range_offset) {
        for y in image_range_offset..(image_height-image_range_offset) {
            let flat_idx = column_major_index(y,x,image_height);
            if !(valid_measurements_reference[flat_idx] && valid_measurements_target[flat_idx]){
                continue;
            }
            let J_image = image_jacobian(image_gradient_x_target,image_gradient_y_target,x,y);
            let J_pi = J_pi_vec[flat_idx];
            let J_lie = J_lie_vec[flat_idx];

            let J = J_image*J_pi*J_lie;
            let J_t = J.transpose();
            let w_i = weights[flat_idx];
            let residual = residuals[flat_idx];

            g += w_i*(-J_t*residual);
            H += w_i*(J_t*J);
        }
    }
    (g,H)
}

// @GPU
pub fn compute_residuals(residuals: &mut Vec<Float>,
                         valid_measurements_reference: &Vec<bool>,
                         valid_measurements_target: &Vec<bool>,
                         image_reference:  &DMatrix<Float>,
                         image_target:  &DMatrix<Float>,
                         projection_onto_target: NormalizedImageCoordinates,
                         image_width: usize,
                         image_height: usize)
                         -> () {
    for x in 0..image_width {
        for y in 0..image_height {
            let flat_index = column_major_index(y, x, image_height);
            let idx_reference = (y, x);
            residuals[flat_index] = 10.0;
            if !valid_measurements_reference[flat_index] || !valid_measurements_target[flat_index] {
                continue;
            }
            let x_idx_target = (*projection_onto_target.index((0, flat_index))).floor() as usize;
            let y_idx_target = (*projection_onto_target.index((1, flat_index))).floor() as usize;
            let idx_target = (y_idx_target, x_idx_target);
            if !((0 < y) && (y < image_height) && (0 < x) && (x < image_width)) ||
                !((0 < y_idx_target) && (y_idx_target < image_height) &&
                    (0 < x_idx_target) && (x_idx_target < image_width)) {
                continue;
            }
            let r = *image_reference.index(idx_reference) - *image_target.index(idx_target);
            residuals[flat_index] = r;
        }
    }
}

#[allow(non_snake_case)]
pub fn approximate_hessian(
                         valid_measurements_reference: &Vec<bool>,
                         valid_measurements_target: &Vec<bool>,
                         image_gradient_x_target: &DMatrix<Float>,
                         image_gradient_y_target: &DMatrix<Float>,
                         J_lie_vec: &Vec<Matrix3x6<Float>>,
                         J_pi_vec: &Vec<Matrix2x3<Float>>,
                         weights: &Vec<Float>,
                         image_width: usize,
                         image_height: usize,
                         image_range_offset: usize)
                         -> Matrix6<Float> {


    let mut H = Matrix6::<Float>::zeros();

    for x in image_range_offset..(image_width-image_range_offset) {
        for y in image_range_offset..(image_height-image_range_offset) {
            let flat_idx = column_major_index(y,x,image_height);
            if !(valid_measurements_reference[flat_idx] && valid_measurements_target[flat_idx]){
                continue;
            }
            let J_image = image_jacobian(image_gradient_x_target,image_gradient_y_target,x,y);
            let J_pi = J_pi_vec[flat_idx];
            let J_lie = J_lie_vec[flat_idx];

            let J = J_image*J_pi*J_lie;
            let J_t = J.transpose();
            let w_i = weights[flat_idx];


            H += w_i*(J_t*J);
        }
    }
    H
}

