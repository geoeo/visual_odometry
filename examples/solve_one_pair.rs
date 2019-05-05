extern crate image;
extern crate cv;
extern crate visual_odometry;


use std::path::Path;
use cv::Mat;
use visual_odometry::image::Image;
use visual_odometry::image::filters::types::ImageFilter;
use visual_odometry::{Frame, solve};
use visual_odometry::camera::intrinsics::Intrinsics;
use visual_odometry::camera::Camera;

#[allow(non_snake_case)]
fn main() {
    let image_name_1 = "1311868174.699578";
    let depth_name_1 = "1311868174.687374";
    let image_name_2 = "1311868174.731625";
    let depth_name_2 = "1311868174.719933";
    let image_format = "png";

    let image_1_path = format!("images/{}.{}",image_name_1, image_format);
    let depth_1_path = format!("images/{}.{}",depth_name_1, image_format);
    let image_2_path = format!("images/{}.{}",image_name_2, image_format);
    let depth_2_path = format!("images/{}.{}",depth_name_2, image_format);


    //TODO: simplify this process
    //TODO: @Investigate -> Seems to be a problem when loaded in depth images. Depth images are 16 bit but loaded as 8bit
    //TODO: ----
    let image_1_im_rs = image::open(&Path::new(&image_1_path)).unwrap().to_luma();
    let depth_1_im_cv
        = Mat::from_path(depth_1_path,
                         cv::imgcodecs::ImageReadMode::AnyDepth)
        .unwrap_or_else(|_| panic!("Could not read image"));
    let image_2_im_rs = image::open(&Path::new(&image_2_path)).unwrap().to_luma();
    let depth_2_im_cv
        = Mat::from_path(depth_2_path,
                         cv::imgcodecs::ImageReadMode::AnyDepth)
        .unwrap_or_else(|_| panic!("Could not read image"));


    let intensity_1 = Image::from_image(image_1_im_rs.clone(), ImageFilter::None, true);
    let mut depth_1 = Image::from_cv_mat(depth_1_im_cv, ImageFilter::None, false);
    let gx = Image::from_image(image_1_im_rs.clone(), ImageFilter::SobelX, false);
    let gy = Image::from_image(image_1_im_rs.clone(), ImageFilter::SobelY, false);

    let intensity_2 = Image::from_image(image_2_im_rs, ImageFilter::None, true);
    let mut depth_2 = Image::from_cv_mat(depth_2_im_cv, ImageFilter::None, false);

    depth_1.buffer /= 5000.0;
    depth_2.buffer /= 5000.0;

    let max_depth = depth_1.buffer.amax();
    //TODO: ----

    let reference_frame = Frame{intensity:intensity_1, depth: depth_1, gradient_x: Some(gx), gradient_y: Some(gy)};
    let target_frame = Frame{intensity:intensity_2, depth: depth_2, gradient_x: None, gradient_y: None};

    let fx = 520.9;
    let fy = 521.0;
    let ox = 321.5;
    let oy = 249.7;
    let intrinsics = Intrinsics::new(fx,fy,ox,oy);
    let camera = Camera{intrinsics};

    println!("starting solve");
    let (SE3, _lie) = solve(reference_frame,
                            target_frame,
                            camera,
                            1000,
                            0.0000001,
                            1.0,
                            max_depth,
                            0.00001,
                            100000.0,
                            100,
                            0,
                            true);
    println!("{}",SE3);



}
