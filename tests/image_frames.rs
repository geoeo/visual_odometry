extern crate image;
extern crate visual_odometry;

use std::path::Path;

use image::imageops::filter3x3;
use image::flat::NormalForm;
use visual_odometry::image::select_filter;
use visual_odometry::image::filters::types::ImageFilter;

#[test]
fn image_is_row_major() {

    let image_path = "images/ferris.png";

    let image = image::open(&Path::new(&image_path)).unwrap().to_luma();
    let kernel = select_filter(ImageFilter::ScharrX);
    let gradient_image = filter3x3(&image,&kernel);

    assert!(image.sample_layout().is_normal(NormalForm::RowMajorPacked));
    assert_eq!(image.sample_layout().is_normal(NormalForm::ColumnMajorPacked),false);

    assert!(gradient_image.sample_layout().is_normal(NormalForm::RowMajorPacked));
    assert_eq!(gradient_image.sample_layout().is_normal(NormalForm::ColumnMajorPacked),false);
}

#[test]
#[should_panic]
fn filter_for_no_gradient() {
    let _kernel = select_filter(ImageFilter::None);
}




