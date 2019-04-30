pub mod filters;
pub mod image;

pub use filters::types::ImageFilter;

pub fn select_filter(filter_type: ImageFilter) -> [f32;9]  {
    return match filter_type {
        ImageFilter::SobelX => filters::HORIZONTAL_SOBEL,
        ImageFilter::SobelY => filters::VERTICAL_SOBEL,
        ImageFilter::ScharrX => filters::HORIZONTAL_SCHARR,
        ImageFilter::ScharrY => filters::VERTICAL_SCHARR,
        ImageFilter::None => panic!("Invalid filter!: {:?}", filter_type)
    };
}
