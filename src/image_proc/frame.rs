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

    pub fn get_buffer(&self) -> &GrayImage {
        return &self.buffer;
    }

    pub fn get_filter(&self) -> ImageFilter {
        return self.filter;
    }

    pub fn get_direction(&self) -> GradientDirection {
        return self.direction;
    }
}

