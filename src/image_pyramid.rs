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

//TODO:
impl Layer {

    pub fn blur_downsample(image_prev: &GrayImage,layer: u32, sigma: f32) -> GrayImage {
        let image_current =
            match layer {
                0 => panic!("Layer 0 image should be added explicitly"),
                _ => {
                    let width_new = image_prev.width()/2;
                    let height_new = image_prev.height()/2;
                    let image_low_pass = blur(image_prev, sigma);
                    //TODO: experiment with filters

                    resize(&image_low_pass,width_new, height_new, FilterType::Triangle) as GrayImage
                }
            };
        image_current
    }

    pub fn from_image(image: &GrayImage,
                      layer: u32,
                      standardize: bool,
                      filter_x: ImageFilter,
                      filter_y: ImageFilter) -> Layer {

        let intensity = Image::from_image(image, ImageFilter::None, standardize);
        let gx = Image::from_image(image, filter_x, standardize);
        let gy = Image::from_image(image, filter_y, standardize);

        Layer{intensity, gradient_x: gx, gradient_y: gy, layer_index: layer}
    }

    pub fn from_images(image: &Image,
                      gx: &Image,
                      gy: &Image,
                      layer: u32) -> Layer {

        Layer{intensity: image.clone(), gradient_x: gx.clone(), gradient_y: gy.clone(), layer_index: layer}
    }

    // (2^L){x,y} + 0.5(2^L - 1) -> probably floor the result
    pub fn generate_depth_coordiantes(layer_index: u32, x: usize, y: usize) -> (usize, usize) {
        let (x_high, y_high) =
            if layer_index == 0 {
            (x, y)
        } else {
            let common = 0.5*((2.0 as Float).powi(layer_index as i32) - 1.0);
            //let common = 0.0;
            let x_new = (2.0 as Float).powi(layer_index as i32)*(x as Float) + common;
            let y_new = (2.0 as Float).powi(layer_index as i32)*(y as Float) + common;
            (x_new.floor() as usize, y_new.floor() as usize)
        };
        (x_high, y_high)
    }
}