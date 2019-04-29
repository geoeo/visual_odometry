extern crate image;
extern crate nalgebra as na;

use image::GrayImage;
use image::flat::NormalForm;
use na::{Dynamic, DMatrix, VecStorage};
use super::filters::types::{ImageFilter, GradientDirection};
use self::image::{DynamicImage};

type ImageMat = f32;

pub struct GrayImageFrame {
    buffer: DMatrix<ImageMat>,
    filter: ImageFilter,
    direction: GradientDirection
}

impl GrayImageFrame {
    pub fn new(buffer: DMatrix<ImageMat>, filter: ImageFilter, direction: GradientDirection) -> GrayImageFrame {
        GrayImageFrame {buffer, filter, direction}
    }

    pub fn from_image(image : GrayImage,filter: ImageFilter, direction: GradientDirection) -> GrayImageFrame {
        let buffer = image_to_matrix(&image);
        GrayImageFrame{buffer, filter, direction}
    }

    pub fn get_buffer(&self) -> &DMatrix<ImageMat> {
        return &self.buffer;
    }

    pub fn get_filter(&self) -> ImageFilter
    {
        return self.filter;
    }

    pub fn get_direction(&self) -> GradientDirection {
        return self.direction;
    }

    pub fn to_image(&self) -> GrayImage {
        return matrix_to_image(&self.buffer);
    }
}

// https://en.wikipedia.org/wiki/Normalization_(image_processing)
fn normalize_to_gray(value : ImageMat, min : ImageMat, max :ImageMat) -> u8 {
    let range = 255 as ImageMat;
    return ((value - min) * (range / (max - min)) + min) as u8;
}

pub fn image_to_matrix(gray_image : &GrayImage) -> DMatrix<ImageMat> {
    debug_assert!(gray_image.sample_layout().is_normal(NormalForm::RowMajorPacked));

    let (width,height) = gray_image.dimensions();
    let size = (width*height) as usize;
    let mut vec_column_major : Vec<ImageMat> = Vec::with_capacity(size);
    for x in 0..width {
        for y in 0..height {
            let pixel = gray_image.get_pixel(x,y);
            let pixel_value = pixel.data[0];
            vec_column_major.push(pixel_value as ImageMat);
        }
    }
    let nrows = Dynamic::new(height as usize);
    let ncols = Dynamic::new(width as usize);
    let vec_storage = VecStorage::new(nrows,ncols,vec_column_major);
    return DMatrix::from_data(vec_storage);
}

pub fn matrix_to_image(matrix: &DMatrix<ImageMat>) -> GrayImage {
    let (rows,cols) = matrix.shape();
    let min = matrix.min();
    // if max is in u8 range, use the full range
    let max = if matrix.max() < (256 as ImageMat) {(255 as ImageMat)} else {matrix.max()};
    let mut gray_image = DynamicImage::new_luma8(cols as u32,rows as u32).to_luma();
    for c in 0..cols {
        for r in 0..rows {
            let val = *matrix.index((r,c));
            let pixel = gray_image.get_pixel_mut(c as u32,r as u32);
            pixel.data[0] = normalize_to_gray(val, min, max);
        }
    }
    return gray_image;
}




