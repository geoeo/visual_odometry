extern crate image as image_rs;

use image_rs::imageops::{blur, resize, FilterType};
use image_rs::GrayImage;
use crate::{Image, Float, Integer};
use crate::image::types::ImageFilter;

pub struct Layer {
    pub intensity: Image,
    pub gradient_x: Image,
    pub gradient_y: Image,
    pub layer_index: usize
}

impl Layer {


    pub fn from_image(image_prev: &GrayImage,
                      previous_layer: usize,
                      sigma: f32,
                      filter_x: ImageFilter,
                      filter_y: ImageFilter) -> Layer {
        assert!(previous_layer > 0);
        let width_prev = image_prev.width();
        let height_prev = image_prev.height();
        let width_new = width_prev / 2;
        let height_new = height_prev / 2;

        let image_low_pass = blur(image_prev, sigma);
        let image_new = resize(&image_low_pass,width_new, height_new, FilterType::Triangle);
        let intensity = Image::from_image(&image_new, ImageFilter::None, true);
        let gx = Image::from_image(&image_new, filter_x, true);
        let gy = Image::from_image(&image_new, filter_y, true);

        Layer{intensity, gradient_x: gx, gradient_y: gy, layer_index: previous_layer + 1}
    }

    // (2^L){x,y} + 0.5(2^L - 1) -> probably floor the result
    pub fn generate_depth_coordiantes(&self, x: usize, y: usize) -> (Integer, Integer) {
        let common = 0.5*((2.0 as Float).powi(self.layer_index as i32) - 1.0);
        let x_high = (2.0 as Float).powi(self.layer_index as i32)*(x as Float) + common;
        let y_high = (2.0 as Float).powi(self.layer_index as i32)*(y as Float) + common;
        (x_high.floor() as Integer,y_high.floor() as Integer)
    }
}