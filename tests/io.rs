use visual_odometry::io::{get_file_list_in_dir, file_name_to_float, associate_file_name, generate_folder_path, generate_runtime_intensity_depth_lists};

#[test]
fn generate_image_lists() {
    let image_folder_path = generate_folder_path("images/color");
    let depth_folder_path = generate_folder_path("images/depth");

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
    let extension = "png";

    let depth_folder_path = generate_folder_path("images/depth");

    let depth_image_1 = associate_file_name(&depth_folder_path,"1311868174.699578");
    let depth_image_2 = associate_file_name(&depth_folder_path,"1311868174.731625");
    let depth_image_3 = associate_file_name(&depth_folder_path,"1311868174.767680");

    assert_eq!(depth_image_1,format!("{}.{}","1311868174.687374",extension));
    assert_eq!(depth_image_2,format!("{}.{}","1311868174.719933",extension));
    assert_eq!(depth_image_3,format!("{}.{}","1311868174.751101",extension));

}

#[test]
fn generate_correct_reference_target_lists(){

    let frame_count = 3;
    let start_name = "1311868174.699578";
    let start_name_2 = "1311868174.731625";
    let extension = "png";
    let start_file = format!("{}.{}",start_name,extension);
    let start_file_2 = format!("{}.{}",start_name_2,extension);

    let image_folder_path = generate_folder_path("images/color");
    let depth_folder_path = generate_folder_path("images/depth");

    let intenstiy = get_file_list_in_dir(&image_folder_path).unwrap_or_else(|_|panic!("reading files failed"));
    let (start_idx,_) = intenstiy.iter().enumerate().find(|(_,x)| **x==start_file).unwrap_or_else(||panic!("reading files failed"));
    let (start_idx_2,_) = intenstiy.iter().enumerate().find(|(_,x)| **x==start_file_2).unwrap_or_else(||panic!("reading files failed"));

    let (runtime_intensity_files, runtime_depth_files) = generate_runtime_intensity_depth_lists(image_folder_path, depth_folder_path, start_name, extension, frame_count);

    assert_eq!(runtime_intensity_files,vec!("1311868174.699578.png","1311868174.731625.png","1311868174.767680.png"));
    assert_eq!(runtime_depth_files,vec!("1311868174.687374.png","1311868174.719933.png","1311868174.751101.png"));
    assert_eq!(start_idx,0);
    assert_eq!(start_idx_2,1);
}