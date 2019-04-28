extern crate image;
extern crate visual_odometry;

use std::path::Path;

use image::imageops::filter3x3;
use visual_odometry::image_proc::select_filter;
use visual_odometry::image_proc::filters::types::{ImageFilter, GradientDirection};
use visual_odometry::image_proc::frame::GrayImageFrame;

#[test]
fn valid() {

    let image_path = "images/ferris.png";

    let image = image::open(&Path::new(&image_path)).unwrap().to_luma();
    let kernel = select_filter(ImageFilter::Scharr,GradientDirection::X);

    let gradient_image = filter3x3(&image,&kernel);

    let _frame = GrayImageFrame::new(image, ImageFilter::None, GradientDirection::None);
    let _gradient_frame = GrayImageFrame::new(gradient_image,ImageFilter::Scharr,GradientDirection::X);
    assert!(true)
}

#[test]
#[should_panic]
fn filter_for_no_gradient() {

    let image_path = "images/ferris.png";

    let image = image::open(&Path::new(&image_path)).unwrap().to_luma();
    let kernel = select_filter(ImageFilter::None,GradientDirection::None);
    let gradient_image = filter3x3(&image,&kernel);

    let _frame = GrayImageFrame::new(image, ImageFilter::None, GradientDirection::None);
    let _gradient_frame = GrayImageFrame::new(gradient_image,ImageFilter::Scharr,GradientDirection::X);
}




