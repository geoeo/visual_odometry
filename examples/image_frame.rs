extern crate image;
extern crate visual_odometry;

use std::path::Path;

use visual_odometry::image::Image;
use visual_odometry::image::types::{ImageFilter,ImageEncoding};

fn main() {
    let image_name = "blocks";
    let image_format = "jpg";
    let image_path = format!("images/{}.{}",image_name, image_format);
    let gray_image_path = format!("output/{}_gray_scale.{}",image_name,image_format);
    let converted_file_out_path = format!("output/{}_converted.{}",image_name,image_format);

    let image = image::open(&Path::new(&image_path)).unwrap().to_luma();
    image.save(gray_image_path).unwrap();

    let frame = Image::from_image(image, ImageFilter::None, false);
    let new_image = frame.to_image(ImageEncoding::U8);

    new_image.save(converted_file_out_path).unwrap();
}
