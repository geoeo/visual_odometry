extern crate nalgebra as na;
extern crate nalgebra_lapack as na_lapack;

use na::{Matrix, Matrix4, Matrix6, U3, U4, Dynamic, Vector6, VecStorage};
use nalgebra::LU;
use numerics::lie::*;
use crate::image::Image;
use crate::gauss_newton_routines::*;
use crate::camera::Camera;
use crate::jacobians::{perspective_jacobians, lie_jacobians};
use crate::numerics::{isometry_from_parts, parts_from_isometry, lm_gamma};
use crate::numerics::weighting::{t_dist_variance,generate_weights};
use crate::frame::Frame;

pub mod image;
pub mod numerics;
pub mod camera;
pub mod jacobians;
pub mod gauss_newton_routines;
pub mod io;
pub mod frame;
pub mod image_pyramid;

pub type Float = f64;
pub type Integer = i64;
// Image coordinates with a "depth" fixed at 1.0 i.e. (u, v, 1.0)
pub type NormalizedImageCoordinates = Matrix<Float, U3, Dynamic, VecStorage<Float, U3, Dynamic>>;
// Homogeneous 3D coordinates i.e. (X, Y, Z, 1.0)
pub type HomogeneousBackProjections = Matrix<Float, U4, Dynamic, VecStorage<Float, U4, Dynamic>>;


#[derive(Debug,Copy,Clone)]
pub struct SolverOptions {
    pub lm: bool, // if false, will use gradient line search
    pub weighting: bool,
    pub print_runtime_info: bool
}

#[allow(non_snake_case)]
#[derive(Debug,Copy,Clone)]
pub struct SolverParameters {
    pub lie_prior: Vector6<Float>, // if false, will use gradient line search
    pub SE3_prior: Matrix4<Float>,
    pub max_its: usize,
    pub eps: Float,
    pub alpha_step: Float,
    pub max_depth: Float,
    pub var_eps: Float,
    pub var_min: Float,
    pub max_its_var: usize,
    pub image_range_offset: usize
}

#[allow(non_snake_case)]
pub fn solve(reference: &Frame,
             target: &Frame,
             camera: Camera,
             parameters: SolverParameters,
             runtime_options: SolverOptions)
             -> (Matrix4<Float>, Vector6<Float>) {

    let max_its = parameters.max_its;
    let eps = parameters.eps;
    let alpha_step = parameters.alpha_step;
    let max_depth = parameters.max_depth;
    let var_eps = parameters.var_eps;
    let var_min = parameters.var_min;
    let max_its_var = parameters.max_its_var;
    let image_range_offset = parameters.image_range_offset;

    let lm = runtime_options.lm;
    let weighting = runtime_options.weighting;
    let print_runtime_info = runtime_options.print_runtime_info;

    let mut lie = parameters.lie_prior;
    let mut SE3 = parameters.SE3_prior;

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
    let generator_z = generator_z_neg();
    let generator_roll = generator_roll();
    let generator_pitch = generator_pitch_neg();
    let generator_yaw = generator_yaw();

    // LM
    let mut mu = -1.0;
    let mut nu = -1.0;
    let I_6 = Matrix6::<Float>::identity();

    let (back_projections,
        mut valid_measurements_reference,
        valid_measurements_target)
        = back_project(camera,
                       &reference.depth.buffer,
                       &target.depth.buffer,
                       image_width,
                       image_height,
                       max_depth);

    let Y_est_init: HomogeneousBackProjections = SE3*(&back_projections);
    let reference_projections = camera.apply_perspective_projection(&Y_est_init);

    let J_pi_vec = perspective_jacobians(&camera.intrinsics, &Y_est_init);
    let J_lie_vec = lie_jacobians(generator_x, generator_y, generator_z, generator_roll, generator_pitch, generator_yaw, &Y_est_init);

    compute_residuals(&mut residuals,
                      &mut valid_measurements_reference,
                      &valid_measurements_target,
                      &reference.intensity.buffer,
                      &target.intensity.buffer,
                      reference_projections,
                      image_width, image_height,
                      image_range_offset);

    // Leuvenberg-Marquart Specific
    if lm {
        //TODO: @Investigate -> This might change when using image pyramids
        // If the inital approximation to SE3 is good, tau can be bigger (0.001 - 1.0)
        let tau = 0.000001 as Float;
        nu = 2.0 as Float;
        let H_initial
            = approximate_hessian(&valid_measurements_reference,
                                  &valid_measurements_target,
                                  &target.gradient_x.buffer,
                                  &target.gradient_y.buffer,
                                  &J_lie_vec,
                                  &J_pi_vec,
                                  &weights,
                                  image_width,
                                  image_height,
                                  image_range_offset);
        let h_max = (H_initial.diagonal().max()) / N as Float;
        mu = tau*h_max;
    }

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


        let lie_new
            = if lm {
            LU::new(H+mu*I_6).solve(&g).unwrap_or_else(|| panic!("System not solvable!"))
        } else {
            alpha_step*LU::new(H).solve(&g).unwrap_or_else(|| panic!("System not solvable!"))
        };
        let (R_current, t_current) = parts_from_isometry(SE3);
        //let (R_current, t_current) = exp(lie);
        let (R_new, t_new) = exp(lie_new);

        let R_est = R_current*R_new;
        let t_est = R_current*t_new + t_current;
        let lie_est = ln(R_est,t_est);
        let SE3_est = isometry_from_parts(R_est,t_est);

        let Y_est: HomogeneousBackProjections = SE3_est*(&back_projections);
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
                              degrees_of_freedom,
                              var_min,
                              var_eps,
                              max_its_var);


        if weighting && variance > 0.0 {
            generate_weights(&residuals,&mut weights,variance,degrees_of_freedom);
        }


        let mut res_sum_squared = 0.0;
        for i in 0..N {
            let res = residuals[i];
            let weight = weights[i];
            res_sum_squared += weight*res*res;
        }

        let res_squared_mean_prev = res_squared_mean;
        //res_squared_mean = res_sum_squared/number_of_valid_measurements as Float;
        res_squared_mean = res_sum_squared/N as Float;
        let residual_delta = (res_squared_mean_prev-res_squared_mean).abs();

        if residual_delta <= eps {
            lie = lie_est;
            SE3 = SE3_est;
            println!("done, squared mean error: {}, delta: {}, pixel ratio: {}",
                     res_squared_mean,
                     residual_delta,
                     valid_pixel_ratio);
            break;
        }

        //Leuvenberg-Marquardt
        // http://www2.imm.dtu.dk/pubdb/views/publication_details.php?id=3215
        if lm {
            let gamma = lm_gamma(res_squared_mean_prev, res_squared_mean, lie_new, g, mu);

            if gamma > 0.0 {
                lie = lie_est;
                SE3 = SE3_est;
                let v1 = 1.0/3.0 as Float;
                let v2 = 1.0-(2.0*gamma-1.0).powf(3.0) as Float;
                let value = if v1 > v2 { v1 } else {v2};
                mu *= value;
                nu = 2.0;

            } else {
                mu *= nu;
                nu *= 2.0;
            }

            mu /= N as Float;
        } else {
            lie = lie_est;
            SE3 = SE3_est;
        }

        if print_runtime_info {
            //TODO: think of a way to buffer this. As it impacts performance
            if lm {
                println!("squared mean error: {}, delta: {}, iterator:  {}, mu: {}", res_squared_mean, residual_delta, it, mu);
            } else {
                println!("squared mean error: {}, delta: {}, iterator:  {}, valid_pixel_ratio: {}, variance: {}", res_squared_mean, residual_delta, it, valid_pixel_ratio, variance);
            }
        }

    }


    (SE3, lie)
}