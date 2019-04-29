extern crate image;
extern crate visual_odometry;

use std::path::Path;

use visual_odometry::image_proc::frame::GrayImageFrame;
use visual_odometry::image_proc::filters::types::{ImageFilter, GradientDirection};

fn main() {

    let image_path = "images/blocks.jpg";
    let gray_image_path = "output/blocks_gray_scale.png";
    let converted_file_out_path = "output/blocks_converted.png";

    let image = image::open(&Path::new(&image_path)).unwrap().to_luma();
    image.save(gray_image_path).unwrap();

    let frame = GrayImageFrame::from_image(image,ImageFilter::None,GradientDirection::None);
    let new_image= frame.to_image();

    new_image.save(converted_file_out_path).unwrap();


}
