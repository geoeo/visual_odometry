extern crate image;
extern crate visual_odometry;

use std::path::Path;

use visual_odometry::image::Image;
use visual_odometry::image::types::ImageFilter;

fn main() {
    let image_name = "depth";
    let image_format = "png";
    let image_path = format!("images/{}.{}",image_name, image_format);
    let gray_image_path = format!("output/{}_gray_scale.{}",image_name,image_format);
    let converted_file_out_path = format!("output/{}_converted.{}",image_name,image_format);

    let image = image::open(&Path::new(&image_path)).unwrap().to_luma();
    image.save(gray_image_path).unwrap();

    let frame = Image::from_image(image, ImageFilter::None, false);
    let new_image = frame.to_image();

    new_image.save(converted_file_out_path).unwrap();
}
