extern crate nalgebra as na;

use na::Vector6;
use visual_odometry::io::{*};
use visual_odometry::Float;

#[test]
fn generate_image_lists() {
    let root = std::env::current_dir().unwrap_or_else(|_| panic!("No current dir"));
    let image_folder_path = generate_folder_path(root.clone(),"images/color");
    let depth_folder_path = generate_folder_path(root.clone(),"images/depth");

    let intensity_files = get_file_list_in_dir(&image_folder_path).unwrap_or_else(|_|panic!("reading files failed"));
    let depth_files = get_file_list_in_dir(&depth_folder_path).unwrap_or_else(|_|panic!("reading files failed"));

    let image_str = intensity_files[0].clone();
    let float = file_name_to_float(&image_str);
    let float_str = float.to_string();

    assert_eq!(float,1311868174.699578);
    assert_eq!(float_str,"1311868174.699578");
    assert_eq!(intensity_files.len(),3);
    assert_eq!(depth_files.len(),3);
}

#[test]
fn associate_file_names(){
    let root = std::env::current_dir().unwrap_or_else(|_| panic!("No current dir"));
    let extension = "png";

    let depth_folder_path = generate_folder_path(root,"images/depth");

    let depth_image_1 = associate_file_name(&depth_folder_path,1311868174.699578,0.1);
    let depth_image_2 = associate_file_name(&depth_folder_path,1311868174.731625,0.1);
    let depth_image_3 = associate_file_name(&depth_folder_path,1311868174.767680,0.1);

    assert_eq!(depth_image_1, format!("{}.{}","1311868174.687374", extension));
    assert_eq!(depth_image_2, format!("{}.{}","1311868174.719933", extension));
    assert_eq!(depth_image_3, format!("{}.{}","1311868174.751101", extension));
}

#[test]
fn generate_correct_reference_target_lists(){
    let root = std::env::current_dir().unwrap_or_else(|_| panic!("No current dir"));
    let frame_count = 3;
    let step_count = 1;
    let start_name = "1311868174.699578";
    let start_name_2 = "1311868174.731625";
    let extension = "png";
    let start_file = format!("{}.{}",start_name,extension);
    let start_file_2 = format!("{}.{}",start_name_2,extension);
    let max_diff_milliseconds = 0.05;

    let image_folder_path = generate_folder_path(root.clone(),"images/color");
    let depth_folder_path = generate_folder_path(root.clone(),"images/depth");

    let intenstiy = get_file_list_in_dir(&image_folder_path).unwrap_or_else(|_|panic!("reading files failed"));
    let (start_idx,_) = intenstiy.iter().enumerate().find(|(_,x)| **x==start_file).unwrap_or_else(||panic!("reading files failed"));
    let (start_idx_2,_) = intenstiy.iter().enumerate().find(|(_,x)| **x==start_file_2).unwrap_or_else(||panic!("reading files failed"));

    let (runtime_intensity_files, runtime_depth_files)
        = generate_runtime_intensity_depth_lists(image_folder_path, depth_folder_path, start_name, extension, step_count, frame_count, max_diff_milliseconds);

    assert_eq!(runtime_intensity_files,vec!("1311868174.699578.png","1311868174.731625.png","1311868174.767680.png"));
    assert_eq!(runtime_depth_files,vec!("1311868174.687374.png","1311868174.719933.png","1311868174.751101.png"));
    assert_eq!(start_idx,0);
    assert_eq!(start_idx_2,1);
}

#[should_panic]
#[test]
fn ts_difference_too_large() {
    let root = std::env::current_dir().unwrap_or_else(|_| panic!("No current dir"));

    let depth_folder_path = generate_folder_path(root,"images/depth");

    let _depth_image_1 = associate_file_name(&depth_folder_path,1311868174.699578,0.000001);
}

#[test]
fn write_to_file(){
    let root = std::env::current_dir().unwrap_or_else(|_| panic!("No current dir"));
    let dir = "output";
    let file_name = "test_out.txt";

    let mut image_folder_path = root;
    image_folder_path.push(dir);
    image_folder_path.push(file_name);

    let path_as_string = image_folder_path.as_os_str().to_str().expect("failed to unwrap");
    assert_eq!(path_as_string,"/Users/marchaubenstock/Workspace/Rust/visual_odometry/output/test_out.txt");

    let test_vec: Vec<Vector6<Float>> = vec![Vector6::new(1.0,2.0,3.0,4.0,5.0,6.0),Vector6::new(11.0,22.0,33.0,44.0,55.0,66.0)];

    write_lie_vectors_to_file(path_as_string,test_vec, &[1.0], &[1.0], &[0], &[1], 1, 1.0, false );

}