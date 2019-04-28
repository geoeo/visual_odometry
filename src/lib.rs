//pub mod image_proc;
//
//use image_proc::types::*;
//use image_proc::filters::*;
//
//pub fn select_filter(filter_type: ImageFilter, gradient_direction: GradientDirection) -> [f32;9]  {
//    return match (filter_type, gradient_direction) {
//        (ImageFilter::Sobel, GradientDirection::X) => HORIZONTAL_SOBEL,
//        (ImageFilter:: Sobel, GradientDirection::Y) => VERTICAL_SOBEL,
//        (_,_) => panic!("Invalid (filter, direction) combination!: ({:?}, {:?})", filter_type, gradient_direction)
//    };
//}

pub mod image_proc;
