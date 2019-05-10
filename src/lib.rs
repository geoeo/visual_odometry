extern crate nalgebra as na;
extern crate nalgebra_lapack as na_lapack;

use na::{Matrix, Matrix4, U3, U4, Dynamic, Vector6, VecStorage};
use nalgebra::LU;
use numerics::lie::*;
use crate::image::Image;
use crate::gauss_newton_routines::{back_project, compute_residuals, gauss_newton_step};
use crate::camera::Camera;
use crate::jacobians::{perspective_jacobians, lie_jacobians};
use crate::numerics::{isometry_from_parts, parts_from_isometry};
use crate::numerics::weighting::{t_dist_variance,generate_weights};
use crate::frame::Frame;

pub mod image;
pub mod numerics;
pub mod camera;
pub mod jacobians;
pub mod gauss_newton_routines;
pub mod io;
pub mod frame;

pub type Float = f64; //Rename this to Float?
// Image coordinates with a "depth" fixed at 1.0 i.e. (u, v, 1.0)
pub type NormalizedImageCoordinates = Matrix<Float, U3, Dynamic, VecStorage<Float, U3, Dynamic>>;
// Homogeneous 3D coordinates i.e. (X, Y, Z, 1.0)
pub type HomogeneousBackProjections = Matrix<Float, U4, Dynamic, VecStorage<Float, U4, Dynamic>>;

#[allow(non_snake_case)]
pub fn solve(reference: &Frame,
             target: &Frame,
             camera: Camera,
             max_its: usize,
             eps: Float,
             alpha_step: Float,
             max_depth: Float,
             var_eps: Float,
             var_min: Float,
             max_its_var: usize,
             image_range_offset: usize,
             print_runtime_info: bool)
             -> (Matrix4<Float>, Vector6<Float>) {
    let mut lie = Vector6::<Float>::zeros();
    let mut SE3 = Matrix4::<Float>::identity();

    let image_width = reference.intensity.buffer.ncols();
    let image_height = reference.intensity.buffer.nrows();
    let N = image_width*image_height;

    fn gen_vec_zeros(size: usize) -> Vec<Float> { vec![0.0 as Float; size] };
    fn gen_vec_ones(size: usize) -> Vec<Float> { vec![1.0 as Float; size] };

    let mut residuals = gen_vec_zeros(N);
    let mut weights = gen_vec_ones(N);
    let mut res_squared_mean = std::f32::MAX as Float;
    let degrees_of_freedom = 5; // empirically derived: see paper Robust VO for RGBD

    // We want RHS coordiante system. As such, we invert Z and Pitch
    let generator_x = generator_x();
    let generator_y = generator_y();
    //let generator_y = generator_y_neg();
    let generator_z = generator_z_neg();
    let generator_roll = generator_roll();
    //let generator_roll = generator_roll_neg();
    let generator_pitch = generator_pitch_neg();
    let generator_yaw = generator_yaw();

    let (back_projections,
        mut valid_measurements_reference,
        valid_measurements_target)
        = back_project(camera,
                       &reference.depth.buffer,
                       &target.depth.buffer,
                       image_width,
                       image_height,
                       max_depth);

    let J_pi_vec = perspective_jacobians(&camera.intrinsics, &back_projections);
    let J_lie_vec = lie_jacobians(generator_x, generator_y, generator_z, generator_roll, generator_pitch, generator_yaw, &back_projections);
    let reference_projections = camera.apply_perspective_projection(&back_projections);

    compute_residuals(&mut residuals,
                      &mut valid_measurements_reference,
                      &valid_measurements_target,
                      &reference.intensity.buffer,
                      &target.intensity.buffer,
                      reference_projections,
                      image_width, image_height,
                      image_range_offset);


    for it in 0..max_its {

        let (g,H)
            = gauss_newton_step(&residuals,
                                &valid_measurements_reference,
                                &valid_measurements_target,
                                &target.gradient_x.buffer,
                                &target.gradient_y.buffer,
                                &J_lie_vec,
                                &J_pi_vec,
                                &weights,
                                image_width,
                                image_height,
                                image_range_offset);


        let lie_new = alpha_step*LU::new(H).solve(&g).unwrap_or_else(|| panic!("System not solvable!"));
        //let (R_current, t_current) = parts_from_isometry(SE3);
        let (R_current, t_current) = exp(lie);
        let (R_new, t_new) = exp(lie_new);

        let R_est = R_current*R_new;
        let t_est = R_current*t_new + t_current;
        lie = ln(R_est,t_est);
        SE3 = isometry_from_parts(R_est,t_est);

        let Y_est: HomogeneousBackProjections = SE3*(&back_projections);
        let target_projections = camera.apply_perspective_projection(&Y_est);

        compute_residuals(&mut residuals,
                          &mut valid_measurements_reference,
                          &valid_measurements_target,
                          &reference.intensity.buffer,
                          &target.intensity.buffer,
                          target_projections,
                          image_width, image_height,
                          image_range_offset);

        let number_of_valid_measurements = valid_measurements_reference.iter().fold(0, |acc,x| acc + *x as usize);
        let valid_pixel_ratio =  number_of_valid_measurements as Float / N as Float;
        if valid_pixel_ratio < 0.0 {
            panic!("All pixels are invalid!")
        }

        let variance
            = t_dist_variance(&residuals,
                              &valid_measurements_reference,
                              &valid_measurements_target,
                              number_of_valid_measurements,
                              degrees_of_freedom,
                              var_min,
                              var_eps,
                              max_its_var);

        //TODO: Maybe redundant
        // clear weights
        for i in 0..N {
            weights[i] = 1.0;
        }

        if variance > 0.0 {
            generate_weights(&residuals,&mut weights,variance,degrees_of_freedom);
        }

        let mut res_sum_squared = 0.0;
        for i in 0..N {
            let res = residuals[i];
            let weight = weights[i];
            res_sum_squared += weight*res*res;
        }

        let res_squared_mean_prev = res_squared_mean;
        res_squared_mean = res_sum_squared/number_of_valid_measurements as Float;
        let residual_delta = (res_squared_mean_prev-res_squared_mean).abs();


        if residual_delta <= eps {
            println!("done, squared mean error: {}, delta: {}, pixel ratio: {}",
                     res_squared_mean,
                     residual_delta,
                     valid_pixel_ratio);
            break;
        }
        if print_runtime_info {
            //TODO: think of a way to buffer this. As it impacts performance
            println!("squared mean error: {}, delta: {}, iterator:  {}, valid_pixel_ratio: {}, variance: {}", res_squared_mean, residual_delta, it, valid_pixel_ratio, variance);
        }

    }


    (SE3, lie)
}