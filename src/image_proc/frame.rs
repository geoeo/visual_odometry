extern crate image;

use image::GrayImage;

use super::filters::types::{ImageFilter,GradientDirection};

pub struct GrayFrame {
    buffer: GrayImage
}

impl GrayFrame {
    pub fn new(buffer: GrayImage) -> GrayFrame {
        GrayFrame{buffer}
    }
}

pub struct GradientFrame {
    buffer: GrayImage,
    filter: ImageFilter,
    direction: GradientDirection
}

impl GradientFrame {
    pub fn new(buffer: GrayImage, filter: ImageFilter, direction: GradientDirection) -> GradientFrame {
        GradientFrame {buffer, filter, direction}
    }
}

