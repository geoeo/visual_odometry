extern crate image as image_rs;

use image_rs::imageops::{blur, resize, FilterType};
use image_rs::GrayImage;
use crate::{Image, Float};
use crate::image::types::ImageFilter;
use self::image_rs::ImageBuffer;
use self::image_rs::Luma;

pub struct Layer {
    pub intensity: Image,
    pub gradient_x: Image,
    pub gradient_y: Image,
    pub layer_index: u32
}

impl Layer {

    pub fn from_image(image_prev: GrayImage,
                      layer: u32,
                      sigma: f32,
                      filter_x: ImageFilter,
                      filter_y: ImageFilter) -> (ImageBuffer<Luma<u8>, Vec<u8>>,Layer) {
        let (image_current, intensity, gx, gy) =
            match layer {
            0 => {
                let intensity = Image::from_image(&image_prev, ImageFilter::None, true);
                let gx = Image::from_image(&image_prev, filter_x, true);
                let gy = Image::from_image(&image_prev, filter_y, true);
                (ImageBuffer::<Luma<u8>, Vec<u8>>::from(image_prev), intensity, gx, gy)
            }
            _ => {
                //TODO: Write unit test for this
                let width_new = image_prev.width()/2;
                let height_new = image_prev.height()/2;
                let image_low_pass = blur(&image_prev, sigma);
                let image_resize = resize(&image_low_pass,width_new, height_new, FilterType::Triangle) as GrayImage;
                let intensity = Image::from_image(&image_resize, ImageFilter::None, true);
                let gx = Image::from_image(&image_resize, filter_x, true);
                let gy = Image::from_image(&image_resize, filter_y, true);
                (image_resize, intensity, gx, gy)
            }
        };

        (image_current, Layer{intensity, gradient_x: gx, gradient_y: gy, layer_index: layer})
    }

    // (2^L){x,y} + 0.5(2^L - 1) -> probably floor the result
    pub fn generate_depth_coordiantes(layer_index: u32, x: usize, y: usize) -> (usize, usize) {
        let (x_high, y_high) =
            if layer_index == 0 {
            (x, y)
        } else {
            //let common = 0.5*((2.0 as Float).powi(layer_index as i32) - 1.0);
            let common = 0.0;
            let x_new = (2.0 as Float).powi(layer_index as i32)*(x as Float) + common;
            let y_new = (2.0 as Float).powi(layer_index as i32)*(y as Float) + common;
            (x_new.floor() as usize, y_new.floor() as usize)
        };
        (x_high, y_high)
    }
}