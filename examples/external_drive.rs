use std::path::PathBuf;
use visual_odometry::io::{generate_folder_path, generate_runtime_intensity_depth_lists};

fn main() {
    let root = PathBuf::from("/Volumes/Sandisk/Diplomarbeit_Resources/VO_Bench/rgbd_dataset_freiburg2_desk");
    let image_folder_path = generate_folder_path(root.clone(),"rgb");
    let depth_folder_path = generate_folder_path(root.clone(),"depth");

    let (intensity_files, depth_files) = generate_runtime_intensity_depth_lists(image_folder_path,depth_folder_path,"1311868164.363181","png",1,10);

    println!("{:?}",intensity_files);
    println!("{:?}",depth_files);
}