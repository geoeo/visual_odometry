extern crate cv;

use cv::Mat;

use visual_odometry::image::Image;
use visual_odometry::image::filters::types::ImageFilter;

fn main() {

    let image_name = "1311868174.687374"; // max value should be 31280.0
    let image_format = "png";
    let image_path = format!("images/{}.{}",image_name,image_format);
    let cv_image_16
        = Mat::from_path(image_path.clone(),
                         cv::imgcodecs::ImageReadMode::AnyDepth)
        .unwrap_or_else(|_| panic!("Could not read image"));



    let image_16 = Image::from_cv_mat(cv_image_16,ImageFilter::None,false);

    println!("{:?}",image_16.buffer.max());






}