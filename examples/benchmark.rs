extern crate nalgebra as na;
extern crate visual_odometry;

use na::{Vector6, Matrix4};
use std::time::Instant;
use visual_odometry::frame::load_frames;
use visual_odometry::{solve, Float, SolverOptions, SolverParameters};
use visual_odometry::camera::intrinsics::Intrinsics;
use visual_odometry::camera::Camera;
use visual_odometry::io::{generate_folder_path,generate_runtime_intensity_depth_lists, generate_runtime_paths};
use visual_odometry::image::types::ImageFilter;

#[allow(non_snake_case)]
fn main() {

    let runs = 1;
    let root = std::env::current_dir().unwrap_or_else(|_| panic!("No current dir"));
    let reference_start_file_name = "1311868174.699578";
    let target_start_file_name = "1311868174.731625";
    let intensity_folder = "images/color";
    let depth_folder = "images/depth";
    let extension = "png";
    let frame_count = 1;
    let step_count = 1;
    let max_diff_milliseconds= 0.05;
    let pyramid_levels = 1;
    let sigma: f32 = 1.0;

    let runtime_options = SolverOptions{
        lm: true,
        weighting: true,
        print_runtime_info: false
    };

    let intensity_folder_path = generate_folder_path(root.clone(),intensity_folder);
    let depth_folder_path = generate_folder_path(root.clone(),depth_folder);

    let (reference_intensity_files, reference_depth_files)
        = generate_runtime_intensity_depth_lists(&intensity_folder_path,&depth_folder_path,reference_start_file_name,extension,step_count,frame_count,max_diff_milliseconds);
    let (target_intensity_files, target_depth_files)
        = generate_runtime_intensity_depth_lists(&intensity_folder_path,&depth_folder_path,target_start_file_name,extension,step_count,frame_count,max_diff_milliseconds);

    let (reference_intensity_paths,reference_depth_paths,target_intensity_paths,target_depth_paths)
        = generate_runtime_paths(intensity_folder_path, depth_folder_path,reference_intensity_files, reference_depth_files, target_intensity_files,target_depth_files);

    let depth_factor = 5000.0;

    let (reference_frames, target_frames, max_depths)
        = load_frames(&reference_intensity_paths,
                      &reference_depth_paths,
                      &target_intensity_paths,
                      &target_depth_paths,
                      depth_factor,
                      ImageFilter::SobelX,
                      ImageFilter::SobelY,
                      pyramid_levels,
                      sigma);

    let number_of_frames = reference_frames.len();

    let fx = 520.9;
    let fy = 521.0;
    let ox = 321.5;
    let oy = 249.7;
    let intrinsics = Intrinsics::new(fx, fy, ox, oy);
    let camera = Camera{intrinsics};

    for i in 0..number_of_frames {

        println!("starting solve");
        let now = Instant::now();
        for _ in 0..runs {
            let max_depth = max_depths[i];
            let reference_frame = &reference_frames[i];
            let target_frame = &target_frames[i];

            let solver_parameters = SolverParameters {
                lie_prior: Vector6::<Float>::zeros(),
                SE3_prior: Matrix4::<Float>::identity(),
                max_its: 1000,
                eps: 0.0000005,
                alpha_step: 1.0,
                max_depth,
                var_eps: 0.0001,
                var_min: 1000.0,
                max_its_var: 100,
                image_range_offset: 0,
                layer_index: 0,
                tau: 0.000001
            };

            let (_SE3, _lie)
                = solve(&reference_frame.image_pyramid[0],
                        &target_frame.image_pyramid[0],
                        &reference_frame.depth,
                        &target_frame.depth,
                        camera,
                        solver_parameters,
                        runtime_options);
        }
        let solver_duration = now.elapsed().as_millis();
        let average_duration = solver_duration as Float /runs as Float;
        println!("Average Solver duration: {} ms/run for {} runs",average_duration, runs);
        //println!("{}",SE3);
        //println!("{}",lie)


    }


}
