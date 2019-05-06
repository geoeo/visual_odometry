extern crate cv;

use cv::Mat;

use visual_odometry::image::Image;
use visual_odometry::image::types::{ImageFilter,ImageEncoding};

fn main() {

    let image_name = "1311868174.687374"; // max value should be 31280.0
    let image_format = "png";
    let image_path = format!("images/{}.{}",image_name,image_format);
    let converted_file_out_path = format!("output/{}_converted.{}",image_name,image_format);
    let cv_image_16
        = Mat::from_path(image_path.clone(),
                         cv::imgcodecs::ImageReadMode::AnyDepth)
        .unwrap_or_else(|_| panic!("Could not read image"));



    let image_16 = Image::from_cv_mat(cv_image_16,ImageFilter::None,false);

    println!("{:?}",image_16.buffer.max());
    let new_image = image_16.to_image(ImageEncoding::U16);

    new_image.save(converted_file_out_path).unwrap();



}