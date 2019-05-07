use visual_odometry::io::get_file_list_in_dir;

#[test]
fn generate_image_lists() {

    let current_dir = std::env::current_dir().unwrap_or_else(|_| panic!("No current dir"));
    let image_folder = "images/color";
    let depth_folder = "images/depth";
    let extension = "png";
    let start_idx = "-1";
    let count = 1;

    let mut image_folder_path = current_dir.clone();
    let mut depth_folder_path = current_dir.clone();

    image_folder_path.push(image_folder);
    depth_folder_path.push(depth_folder);



    let color_files = get_file_list_in_dir(image_folder_path).unwrap_or_else(|_|panic!("reading files failed"));
    let depth_files = get_file_list_in_dir(depth_folder_path).unwrap_or_else(|_|panic!("reading files failed"));

    assert_eq!(color_files.len(),3);
    assert_eq!(depth_files.len(),3)


}