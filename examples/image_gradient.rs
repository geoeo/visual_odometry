extern crate image;
extern crate visual_odometry;

use std::path::Path;

use image::imageops::filter3x3;
use visual_odometry::image::select_filter;
use visual_odometry::image::types::ImageFilter;

fn main() {

    let image_name = "ferris";
    let image_format = "png";
    let image_filter = ImageFilter::ScharrX;

    let image_path = format!("images/{}.{}",image_name, image_format);
    let file_out_path = format!("output/{}_{:?}.{}",image_name,image_filter,image_format);

    let image = image::open(&Path::new(&image_path)).unwrap().grayscale().to_luma();
    let kernel = select_filter(image_filter).unwrap();

    let gradient_image = filter3x3(&image,&kernel);

    gradient_image.save(file_out_path).unwrap();

}