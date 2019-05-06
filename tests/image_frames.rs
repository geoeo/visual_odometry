extern crate image;
extern crate visual_odometry;

use std::path::Path;

use image::imageops::filter3x3;
use image::flat::NormalForm;
use visual_odometry::image::select_filter;
use visual_odometry::image::types::{ImageFilter, ImageEncoding};
use visual_odometry::image::Image;

#[test]
fn image_is_row_major() {

    let image_path = "images/ferris.png";

    let image = image::open(&Path::new(&image_path)).unwrap().to_luma();
    let kernel = select_filter(ImageFilter::ScharrX).unwrap();
    let gradient_image = filter3x3(&image,&kernel);

    assert!(image.sample_layout().is_normal(NormalForm::RowMajorPacked));
    assert_eq!(image.sample_layout().is_normal(NormalForm::ColumnMajorPacked),false);

    assert!(gradient_image.sample_layout().is_normal(NormalForm::RowMajorPacked));
    assert_eq!(gradient_image.sample_layout().is_normal(NormalForm::ColumnMajorPacked),false);
}

#[test]
fn filter_for_no_gradient() {
    let kernel = select_filter(ImageFilter::None);
    assert_eq!(kernel, None);
}

#[test]
fn check_encodings() {
    let image_path = "images/ferris.png";

    let image = image::open(&Path::new(&image_path)).unwrap().to_luma();
    let frame = Image::from_image(image, ImageFilter::None, false);

    let width = 2;
    let height= 2;
    let vec_16 : Vec<u16> = vec!(1 as u16,2 as u16,3 as u16,4 as u16);
    let frame_16 = Image::from_vec_16(height, width, &vec_16,false);

    assert_eq!(frame.original_encoding, ImageEncoding::U8);
    assert_eq!(frame_16.original_encoding, ImageEncoding::U16);

}




