pub mod filters;

use filters::types::{ImageFilter, GradientDirection};


pub fn select_filter(filter_type: ImageFilter, gradient_direction: GradientDirection) -> [f32;9]  {
    return match (filter_type, gradient_direction) {
        (ImageFilter::Sobel, GradientDirection::X) => filters::HORIZONTAL_SOBEL,
        (ImageFilter:: Sobel, GradientDirection::Y) => filters::VERTICAL_SOBEL,
        (ImageFilter:: Scharr, GradientDirection::X) => filters::HORIZONTAL_SCHARR,
        (ImageFilter:: Scharr, GradientDirection::Y) => filters::VERTICAL_SCHARR
        //(_,_) => panic!("Invalid (filter, direction) combination!: ({:?}, {:?})", filter_type, gradient_direction)
    };
}