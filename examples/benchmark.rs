extern crate image;
extern crate visual_odometry;


use std::time::Instant;
use visual_odometry::frame::load_frames;
use visual_odometry::{solve, Float};
use visual_odometry::camera::intrinsics::Intrinsics;
use visual_odometry::camera::Camera;
use visual_odometry::io::{generate_folder_path,generate_runtime_intensity_depth_lists, generate_runtime_paths};

#[allow(non_snake_case)]
fn main() {

    let runs = 20;

    let reference_start_file_name = "1311868174.699578";
    let target_start_file_name = "1311868174.731625";
    let intensity_folder = "images/color";
    let depth_folder = "images/depth";
    let extension = "png";
    let frame_count = 1;
    let step_count = 1;

    let intensity_folder_path = generate_folder_path(intensity_folder);
    let depth_folder_path = generate_folder_path(depth_folder);

    let (reference_intensity_files, reference_depth_files)
        = generate_runtime_intensity_depth_lists(&intensity_folder_path,&depth_folder_path,reference_start_file_name,extension,step_count,frame_count);
    let (target_intensity_files, target_depth_files)
        = generate_runtime_intensity_depth_lists(&intensity_folder_path,&depth_folder_path,target_start_file_name,extension,step_count,frame_count);

    let (reference_intensity_paths,reference_depth_paths,target_intensity_paths,target_depth_paths)
        = generate_runtime_paths(intensity_folder_path, depth_folder_path,reference_intensity_files, reference_depth_files, target_intensity_files,target_depth_files);

    let depth_factor = 5000.0;

    let (reference_frames, target_frames, max_depths)
        = load_frames(&reference_intensity_paths,
                      &reference_depth_paths,
                      &target_intensity_paths,
                      &target_depth_paths,
                      depth_factor);

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

            let (_SE3, _lie)
                = solve(&reference_frame,
                        &target_frame,
                        camera,
                        1000,
                        0.00000000001,
                        1.0,
                        max_depth,
                        0.0001,
                        100000.0,
                        100,
                        0,
                        false);
        }
        let solver_duration = now.elapsed().as_millis();
        let average_duration = solver_duration as Float /runs as Float;
        println!("Average Solver duration: {} ms/run for {} runs",average_duration, runs);
        //println!("{}",SE3);
        //println!("{}",lie)


    }


}
