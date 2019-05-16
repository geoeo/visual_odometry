extern crate image as image_rs;

use image_rs::imageops::{blur, resize, FilterType};
use image_rs::GrayImage;
use crate::{Image, Float};
use crate::image::types::ImageFilter;

pub struct Layer {
    pub intensity: Image,
    pub gradient_x: Image,
    pub gradient_y: Image,
    pub layer_index: u32
}

impl Layer {

    pub fn from_image(image_orig: &GrayImage,
                      layer: u32,
                      sigma: f32,
                      filter_x: ImageFilter,
                      filter_y: ImageFilter) -> Layer {
        let (intensity, gx, gy) =
            match layer {
            0 => {
                let intensity = Image::from_image(image_orig, ImageFilter::None, true);
                let gx = Image::from_image(image_orig, filter_x, true);
                let gy = Image::from_image(image_orig, filter_y, true);
                (intensity, gx, gy)
            }
            x => {
                //TODO: Write unit test for this
                let width_prev = image_orig.width();
                let height_prev = image_orig.height();
                let width_new = (width_prev as Float) / (2.0 as Float).powi(x as i32);
                let height_new = (height_prev as Float)/ (2.0 as Float).powi(x as i32);
                let image_low_pass = blur(image_orig, sigma);
                let image_resize = resize(&image_low_pass,width_new as u32, height_new as u32, FilterType::Triangle) as GrayImage;
                let intensity = Image::from_image(&image_resize, ImageFilter::None, true);
                let gx = Image::from_image(&image_resize, filter_x, true);
                let gy = Image::from_image(&image_resize, filter_y, true);
                (intensity, gx, gy)
            }
        };

        Layer{intensity, gradient_x: gx, gradient_y: gy, layer_index: layer}
    }

    //TODO: Write Unit Test!
    // (2^L){x,y} + 0.5(2^L - 1) -> probably floor the result
    pub fn generate_depth_coordiantes(layer_index: u32, x: usize, y: usize) -> (usize, usize) {
        let (x_high, y_high) =
            if layer_index == 0 {
            (x, y)
        } else {
            let common = 0.5*((2.0 as Float).powi(layer_index as i32) - 1.0);
            let x_new = (2.0 as Float).powi(layer_index as i32)*(x as Float) + common;
            let y_new = (2.0 as Float).powi(layer_index as i32)*(y as Float) + common;
            (x_new.floor() as usize, y_new.floor() as usize)
        };
        (x_high, y_high)
    }
}