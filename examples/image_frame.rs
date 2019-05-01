extern crate image;
extern crate visual_odometry;

use std::path::Path;

use visual_odometry::image_proc::image::Image;
use visual_odometry::image_proc::filters::types::ImageFilter;

fn main() {

    let image_path = "images/blocks.jpg";
    let gray_image_path = "output/blocks_gray_scale.png";
    let converted_file_out_path = "output/blocks_converted.png";

    let image = image::open(&Path::new(&image_path)).unwrap().to_luma();
    image.save(gray_image_path).unwrap();

    let frame = Image::from_image(image, ImageFilter::None, false);
    let new_image= frame.to_image();

    new_image.save(converted_file_out_path).unwrap();

}
