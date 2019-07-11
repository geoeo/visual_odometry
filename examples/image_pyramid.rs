extern crate nalgebra as na;
extern crate visual_odometry;

use visual_odometry::frame::load_frames;
use visual_odometry::io::{generate_folder_path,generate_runtime_intensity_depth_lists, generate_runtime_paths};
use visual_odometry::image::types::ImageFilter;

#[allow(non_snake_case)]
fn main() {
    let root = std::env::current_dir().unwrap_or_else(|_| panic!("No current dir"));
    let reference_start_file_name = "1311868174.699578";
    let target_start_file_name = "1311868174.731625";
    let intensity_folder = "images/color";
    let depth_folder = "images/depth";
    let extension = "png";

    let layer_0_im_out_path = format!("output/{}_{}.{}","layer_0","intensity",extension);
    let layer_1_im_out_path = format!("output/{}_{}.{}","layer_1","intensity",extension);
    let layer_2_im_out_path = format!("output/{}_{}.{}","layer_2","intensity",extension);

    let layer_0_gx_out_path = format!("output/{}_{}.{}","layer_0","gx",extension);
    let layer_1_gx_out_path = format!("output/{}_{}.{}","layer_1","gx",extension);
    let layer_2_gx_out_path = format!("output/{}_{}.{}","layer_2","gx",extension);

    let frame_count = 1;
    let step_count = 1;
    let max_diff_milliseconds = 0.03;
    let pyramid_levels = 3;
    let sigma: f32 = 2.0;

    let intensity_folder_path = generate_folder_path(root.clone(),intensity_folder);
    let depth_folder_path = generate_folder_path(root.clone(),depth_folder);

    let (reference_intensity_files, reference_depth_files)
        = generate_runtime_intensity_depth_lists(&intensity_folder_path,&depth_folder_path,reference_start_file_name,extension,step_count,frame_count,max_diff_milliseconds);
    let (target_intensity_files, target_depth_files)
        = generate_runtime_intensity_depth_lists(&intensity_folder_path,&depth_folder_path,target_start_file_name,extension,step_count,frame_count,max_diff_milliseconds);

    let (reference_intensity_paths,reference_depth_paths,target_intensity_paths,target_depth_paths)
        = generate_runtime_paths(intensity_folder_path, depth_folder_path,reference_intensity_files, reference_depth_files, target_intensity_files,target_depth_files);

    let depth_factor = 5000.0;

    let (reference_frames, _target_frames, _max_depths)
        = load_frames(&reference_intensity_paths,
                      &reference_depth_paths,
                      &target_intensity_paths,
                      &target_depth_paths,
                      depth_factor,
                      -1.0,
                      ImageFilter::SobelX,
                      ImageFilter::SobelY,
                      true,
                      pyramid_levels,
                      sigma);

    let frame = &reference_frames[0];

    let layer_0 = &frame.image_pyramid[0];
    let layer_1 = &frame.image_pyramid[1];
    let layer_2 = &frame.image_pyramid[2];

    let layer_0_im = layer_0.intensity.to_image();
    let layer_1_im = layer_1.intensity.to_image();
    let layer_2_im = layer_2.intensity.to_image();

    let layer_0_gx = layer_0.gradient_x.to_image();
    let layer_1_gx = layer_1.gradient_x.to_image();
    let layer_2_gx = layer_2.gradient_x.to_image();

    layer_0_im.save(layer_0_im_out_path).unwrap();
    layer_1_im.save(layer_1_im_out_path).unwrap();
    layer_2_im.save(layer_2_im_out_path).unwrap();

    layer_0_gx.save(layer_0_gx_out_path).unwrap();
    layer_1_gx.save(layer_1_gx_out_path).unwrap();
    layer_2_gx.save(layer_2_gx_out_path).unwrap();

}