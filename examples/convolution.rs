extern crate nalgebra as na;
extern crate image as image_rs;
extern crate visual_odometry;

use std::path::Path;

use na::{Vector1,Vector6,DMatrix, Matrix3, Matrix1};
use na::linalg;
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


    let vec = Vector6::new(1.0,2.0,3.0,4.0,5.0,6.0);
    let filter = Vector1::new(1.0);
    vec.convolve_full(filter);

    let mat = Matrix3::new(1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0);
    let filter_mat = Matrix1::new(1.0);
    mat.convolve_full(filter_mat);

    //let d_mat = DMatrix::from_diagonal_element(3,3,1.0);
    //let filter_d_mat = DMatrix::from_diagonal_element(1,1,1.0);
    //d_mat.convolve_full(filter_d_mat);

}