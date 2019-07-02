extern crate nalgebra as na;
extern crate image;
extern crate visual_odometry;

use std::time::Instant;
use std::path::PathBuf;
use na::{Matrix4,Vector6};
use visual_odometry::frame::load_frames;
use visual_odometry::{solve, Float, SolverOptions, SolverParameters};
use visual_odometry::camera::intrinsics::Intrinsics;
use visual_odometry::camera::Camera;
use visual_odometry::io::{*};
use visual_odometry::image::types::ImageFilter;


#[allow(non_snake_case)]
fn main() {

    let data_set = "rgbd_dataset_freiburg2_desk";
    let mut root = PathBuf::from("/Volumes/Sandisk/Diplomarbeit_Resources/VO_Bench");
    root.push(data_set);

    //let reference_start_file_name = "1311868174.699578";
    //let target_start_file_name = "1311868174.731625";
    let reference_start_file_name = "1311868164.363181";
    let target_start_file_name = "1311868164.399026";
    let intensity_folder = "rgb";
    let depth_folder = "depth";
    let extension = "png";
    let frame_count = 150;
    let step_count = 1;
    let debug = false;
    let run_vo = true;
    let max_diff_milliseconds = 0.03;
    let taus = [0.001,0.000001,0.000000001];
    // alphas range form level 0 -> higher
    //let alphas = [1.2,0.42, 0.25];
    let alphas = [1.0];
    let pyramid_levels = 1;
    //TODO: @Investigate -> sigma value
    let sigma: f32 = 3.0;
    //et eps = [0.07,0.05, 0.01];
    //let eps = [0.001]; //weighting
    let eps = [0.00001]; //lm
    let image_range_offsets = [10, 5, 2];
    let max_its = [40,60,100];

    let runtime_options = SolverOptions{
        lm: true,
        weighting: false,
        print_runtime_info: true
    };


    let project_root = std::env::current_dir().unwrap_or_else(|_| panic!("No current dir"));
    let out_dir = "output";
    let file_name = format!("{}_{}",data_set,reference_start_file_name);
    let mut lie_results_path = project_root;
    lie_results_path.push(out_dir);
    lie_results_path.push(file_name);

    let lie_results_file_path = lie_results_path.as_os_str().to_str().expect("failed to unwrap");

    let intensity_folder_path = generate_folder_path(root.clone(),intensity_folder);
    let depth_folder_path = generate_folder_path(root.clone(),depth_folder);

    let (reference_intensity_files, reference_depth_files)
        = generate_runtime_intensity_depth_lists(&intensity_folder_path,&depth_folder_path,reference_start_file_name,extension,step_count,frame_count, max_diff_milliseconds);
    let (target_intensity_files, target_depth_files)
        = generate_runtime_intensity_depth_lists(&intensity_folder_path,&depth_folder_path,target_start_file_name,extension,step_count,frame_count, max_diff_milliseconds);

    if debug {
        println!("{:?}",reference_intensity_files);
        println!("{:?}",reference_depth_files);
        println!("{:?}",target_intensity_files);
        println!("{:?}",target_depth_files);
    }

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
                      true,
                      pyramid_levels,
                      sigma);

    let number_of_frames = reference_frames.len();

    let fx = 520.9;
    let fy = 521.0;
    let ox = 321.5;
    let oy = 249.7;
    let intrinsics = Intrinsics::new(fx, fy, ox, oy);
    let camera = Camera{intrinsics};

    if run_vo {
        println!("starting solve");
        let mut SE3_buffer: Vec<Matrix4<Float>> = Vec::with_capacity(number_of_frames);
        let mut lie_buffer: Vec<Vector6<Float>> = Vec::with_capacity(number_of_frames);

        for i in 0..number_of_frames {
            let max_depth = max_depths[i];
            let reference_frame = &reference_frames[i];
            let target_frame = &target_frames[i];
            let idx_init = pyramid_levels as usize-1;
            let mut solver_parameters = SolverParameters {
                lie_prior: Vector6::<Float>::zeros(),
                SE3_prior: Matrix4::<Float>::identity(),
                max_its: max_its[idx_init],
                eps: eps[idx_init],
                alpha_step: alphas[idx_init] as Float,
                max_depth,
                var_eps: 0.0001,
                var_min: 1000.0,
                max_its_var: 100,
                image_range_offset: image_range_offsets[idx_init],
                layer_index: pyramid_levels-1,
                tau: taus[idx_init]
            };
            let now = Instant::now();
            for layer in (0..pyramid_levels).rev() {
                let reference_layer = &reference_frame.image_pyramid[layer as usize];
                let target_layer = &target_frame.image_pyramid[layer as usize];
                let reference_depth = &reference_frame.depth;
                let target_depth = &target_frame.depth;


                let (SE3, lie)
                    = solve(reference_layer,
                            target_layer,
                            reference_depth,
                            target_depth,
                            camera,
                            solver_parameters,
                            runtime_options);
                if layer > 0 {
                    //let diff = pyramid_levels-layer;
                    //let tau_new = tau_orig*(10.0 as Float).powi(diff as i32);
                    let next_idx = layer - 1;
                    let image_offset_new = image_range_offsets[next_idx as usize];
                    let alpha_new = alphas[next_idx as usize] as Float;
                    let eps_new = eps[next_idx as usize];
                    let tau_new = taus[next_idx as usize];
                    let max_its_new = max_its[next_idx as usize];
                    solver_parameters = SolverParameters {
                        lie_prior: lie,
                        SE3_prior: SE3,
                        max_its: max_its_new,
                        eps: eps_new,
                        alpha_step:  alpha_new,
                        max_depth,
                        var_eps: 0.0001,
                        var_min: 1000.0,
                        max_its_var: 100,
                        image_range_offset: image_offset_new,
                        layer_index: next_idx,
                        tau: tau_new
                    };
                }
                if layer == 0 {
                    SE3_buffer.push(SE3);
                    lie_buffer.push(lie);
                }
            }
            let solver_duration = now.elapsed().as_millis();

            println!("{}) Solver duration: {} ms",i, solver_duration as Float);
        }

        write_lie_vectors_to_file(lie_results_file_path,
                                  lie_buffer,
                                  &alphas,
                                  &eps,
                                  &image_range_offsets,
                                  &max_its,
                                  pyramid_levels,
                                  sigma,
                                  runtime_options.lm);
    }


}
