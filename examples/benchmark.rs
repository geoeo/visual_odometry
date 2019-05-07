extern crate image;
extern crate visual_odometry;


use std::time::Instant;
use visual_odometry::frame::load_frames;
use visual_odometry::{solve, MatrixData};
use visual_odometry::camera::intrinsics::Intrinsics;
use visual_odometry::camera::Camera;

#[allow(non_snake_case)]
fn main() {
    let image_name_1 = "1311868174.699578";
    let depth_name_1 = "1311868174.687374";
    let image_name_2 = "1311868174.731625";
    let depth_name_2 = "1311868174.719933";
    let image_format = "png";

    let image_1_path = format!("images/{}.{}", image_name_1, image_format);
    let depth_1_path = format!("images/{}.{}", depth_name_1, image_format);
    let image_2_path = format!("images/{}.{}", image_name_2, image_format);
    let depth_2_path = format!("images/{}.{}", depth_name_2, image_format);

    let reference_image_paths = vec!(image_1_path);
    let reference_depth_paths = vec!(depth_1_path);
    let target_image_paths = vec!(image_2_path);
    let target_depth_paths = vec!(depth_2_path);

    let depth_factor = 5000.0;

    let (reference_frames, target_frames,max_depths)
        = load_frames(&reference_image_paths,
                      &reference_depth_paths,
                      &target_image_paths,
                      &target_depth_paths,
                      depth_factor);

    let number_of_frames = reference_frames.len();

    let fx = 520.9;
    let fy = 521.0;
    let ox = 321.5;
    let oy = 249.7;
    let intrinsics = Intrinsics::new(fx,fy,ox,oy);
    let camera = Camera{intrinsics};

    let runs = 100;
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
        let average_duration = solver_duration as MatrixData/runs as MatrixData;
        println!("Average Solver duration: {} ms/run for {} runs",average_duration, runs);
        //println!("{}",SE3);
        //println!("{}",lie)


    }







}
