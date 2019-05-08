use visual_odometry::io::{get_file_list_in_dir,file_name_to_float,associate_file_name,generate_folder_path};

#[test]
fn generate_image_lists() {
    let image_folder_path = generate_folder_path("images/color");
    let depth_folder_path = generate_folder_path("images/depth");

    let color_files = get_file_list_in_dir(&image_folder_path).unwrap_or_else(|_|panic!("reading files failed"));
    let depth_files = get_file_list_in_dir(&depth_folder_path).unwrap_or_else(|_|panic!("reading files failed"));

    let image_str = color_files[0].clone();
    let float = file_name_to_float(&image_str);
    let float_str = float.to_string();

    assert_eq!(float,1311868174.699578);
    assert_eq!(float_str,"1311868174.699578");
    assert_eq!(color_files.len(),3);
    assert_eq!(depth_files.len(),3);

}

#[test]
fn associate_file_names(){
    let extension = "png";

    let image_folder_path = generate_folder_path("images/color");
    let depth_folder_path = generate_folder_path("images/depth");

    let depth_image_1 = associate_file_name(&depth_folder_path,"1311868174.699578").unwrap_or_else(|_|panic!("file assocation failed"));
    let depth_image_2 = associate_file_name(&depth_folder_path,"1311868174.731625").unwrap_or_else(|_|panic!("file assocation failed"));
    let depth_image_3 = associate_file_name(&depth_folder_path,"1311868174.767680").unwrap_or_else(|_|panic!("file assocation failed"));

    assert_eq!(depth_image_1,format!("{}.{}","1311868174.687374",extension));
    assert_eq!(depth_image_2,format!("{}.{}","1311868174.719933",extension));
    assert_eq!(depth_image_3,format!("{}.{}","1311868174.751101",extension));

}

#[test]
fn generate_correct_reference_target_lists(){

    let frame_count = 3;
    let start_idx = "1311868174.699578";
    let extension = "png";

    let image_folder_path = generate_folder_path("images/color");
    let depth_folder_path = generate_folder_path("images/depth");






    assert!(false);
}