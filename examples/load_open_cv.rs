extern crate cv;

use cv::Mat;

use visual_odometry::image::Image;
use visual_odometry::image::types::{ImageFilter,ImageEncoding};

fn main() {

    let depth_name = "1311868174.687374"; // max value should be 31280.0
    let intensity_name = "1311868174.699578"; // max value should be 254.0/1.86717
    let image_format = "png";
    let depth_path = format!("images/depth/{}.{}",depth_name,image_format);
    let intensity_path = format!("images/color/{}.{}",intensity_name,image_format);
    let cv_image_16
        = Mat::from_path(depth_path.clone(),
                         cv::imgcodecs::ImageReadMode::AnyDepth)
        .unwrap_or_else(|_| panic!("Could not read image"));
    let cv_image_8
        = Mat::from_path(intensity_path.clone(),
                         cv::imgcodecs::ImageReadMode::Grayscale)
        .unwrap_or_else(|_| panic!("Could not read image"));


    let image_16 = Image::from_cv_mat(cv_image_16,ImageFilter::None,false, ImageEncoding::U16);
    let image_8 = Image::from_cv_mat(cv_image_8,ImageFilter::None,true, ImageEncoding::U8);

    println!("{:?}",image_16.buffer.max());
    println!("{:?}",image_8.buffer.max());


}