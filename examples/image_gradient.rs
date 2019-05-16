extern crate image as image_rs;
extern crate visual_odometry;

use std::path::Path;

use image_rs::imageops::filter3x3;
use visual_odometry::image::types::{ImageFilter, ImageEncoding};
use visual_odometry::image::Image;
use visual_odometry::image::filters::HORIZONTAL_SCHARR;

fn main() {

    let image_name = "ferris";
    let image_name_mat = "ferris_mat";
    let image_format = "png";
    let image_filter = ImageFilter::ScharrX;

    let image_path = format!("images/{}.{}",image_name, image_format);
    let file_out_path = format!("output/{}_{:?}.{}",image_name,image_filter,image_format);
    let file_mat_out_path = format!("output/{}_{:?}.{}",image_name_mat,image_filter,image_format);

    let image = image::open(&Path::new(&image_path)).unwrap().grayscale().to_luma();
    let kernel = HORIZONTAL_SCHARR;

    let gradient_image = filter3x3(&image,&kernel);
    let gradient_image_conv = Image::from_image(&gradient_image,ImageFilter::None, false);

    let max = gradient_image_conv.buffer.max();
    let min = gradient_image_conv.buffer.min();

    println!("min: {}, max: {}", min, max);

    let image_mat = Image::from_image(&image,ImageFilter::None, false);

    let mat_gradient_image = Image::from_matrix(&image_mat.buffer, ImageFilter::SobelX, false, ImageEncoding::F64);

    let my_max = mat_gradient_image.buffer.max();
    let my_min = mat_gradient_image.buffer.min();

    println!("my min: {}, my max: {}", my_min, my_max);

    let mat_gradient_image_rs = mat_gradient_image.to_image();

    //let my_max_2 = mat_gradient_image.buffer.max();
    //let my_min_2 = mat_gradient_image.buffer.min();

    //println!("my min 2: {}, my max 2: {}", my_min_2, my_max_2);

    gradient_image.save(file_out_path).unwrap();
    mat_gradient_image_rs.save(file_mat_out_path).unwrap();

}