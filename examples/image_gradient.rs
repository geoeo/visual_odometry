extern crate image;
extern crate visual_odometry;

use std::path::Path;

use image::imageops::filter3x3;
use visual_odometry::image_proc::select_filter;
use visual_odometry::image_proc::filters::types::ImageFilter;

fn main() {

    let image_path = "images/ferris.png";
    let file_out_path = "output/gradient_test.png";

    let image = image::open(&Path::new(&image_path)).unwrap().grayscale().to_luma();
    let kernel = select_filter(ImageFilter::ScharrX);

    let gradient_image = filter3x3(&image,&kernel);

    gradient_image.save(file_out_path).unwrap();

}