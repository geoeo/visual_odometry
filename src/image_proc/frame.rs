extern crate image;

use image::GrayImage;
use super::filters::types::{ImageFilter, GradientDirection};

pub struct GrayImageFrame {
    buffer: GrayImage,
    filter: ImageFilter,
    direction: GradientDirection
}

impl GrayImageFrame {
    pub fn new(buffer: GrayImage, filter: ImageFilter, direction: GradientDirection) -> GrayImageFrame {
        GrayImageFrame {buffer, filter, direction}
    }
}

