extern crate nalgebra as na;
extern crate image;
extern crate visual_odometry;

use std::time::Instant;
use std::path::PathBuf;
use na::{Matrix4,Vector6};
use visual_odometry::frame::load_frames;
use visual_odometry::{solve, Float, SolverOptions};
use visual_odometry::camera::intrinsics::Intrinsics;
use visual_odometry::camera::Camera;
use visual_odometry::io::{*};
use visual_odometry::image::types::ImageFilter;


#[allow(non_snake_case)]
fn main() {

    let data_set = "rgbd_dataset_freiburg2_desk";
    let mut root = PathBuf::from("/Volumes/Sandisk/Diplomarbeit_Resources/VO_Bench");
    root.push(data_set);

    let reference_start_file_name = "1311868174.699578";
    let target_start_file_name = "1311868174.731625";
    let intensity_folder = "rgb";
    let depth_folder = "depth";
    let extension = "png";
    let frame_count = 150;
    let step_count = 1;
    let debug = false;
    let run_vo = true;
    let max_diff_milliseconds = 0.03;

    let runtime_options = SolverOptions{
        lm: true,
        weighting: true,
        print_runtime_info: true
    };


    let project_root = std::env::current_dir().unwrap_or_else(|_| panic!("No current dir"));
    let out_dir = "output";
    let file_name = format!("{}_{}.txt",data_set,reference_start_file_name);
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
                      ImageFilter::SobelY);

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
            let now = Instant::now();
            let (SE3, lie)
                = solve(&reference_frame,
                        &target_frame,
                        camera,
                        1000,
                        0.0000005,
                        1.0,
                        max_depth,
                        0.0001,
                        1000.0,
                        100,
                        0,
                        runtime_options);
            let solver_duration = now.elapsed().as_millis();
            SE3_buffer.push(SE3);
            lie_buffer.push(lie);
            println!("{}) Solver duration: {} ms",i, solver_duration as Float);
        }
        write_lie_vectors_to_file(lie_results_file_path,lie_buffer);
    }


}
