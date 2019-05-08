use visual_odometry::io::{get_file_list_in_dir,file_name_to_float};

#[test]
fn generate_image_lists() {

    let current_dir = std::env::current_dir().unwrap_or_else(|_| panic!("No current dir"));
    let image_folder = "images/color";
    let depth_folder = "images/depth";
    let extension = "png";

    let mut image_folder_path = current_dir.clone();
    let mut depth_folder_path = current_dir.clone();

    image_folder_path.push(image_folder);
    depth_folder_path.push(depth_folder);


    let color_files = get_file_list_in_dir(image_folder_path).unwrap_or_else(|_|panic!("reading files failed"));
    let depth_files = get_file_list_in_dir(depth_folder_path).unwrap_or_else(|_|panic!("reading files failed"));

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
    let start_idx = "-1";
    let count = 1;

    //TODO

}