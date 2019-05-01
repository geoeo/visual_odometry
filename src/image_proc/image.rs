extern crate image;
extern crate nalgebra as na;

use image::GrayImage;
use image::flat::NormalForm;
use na::{Dynamic, DMatrix, VecStorage};
use super::filters::types::ImageFilter;
use self::image::DynamicImage;
use crate::numerics::MatrixData;
use crate::numerics::matrix_ops::z_standardise;

pub struct Image {
    buffer: DMatrix<MatrixData>,
    filter: ImageFilter,
    is_standardized : bool
}

impl Image {
    pub fn new(buffer: DMatrix<MatrixData>, filter: ImageFilter, is_standardized : bool) -> Image {
        Image { buffer, filter, is_standardized}
    }

    pub fn from_image(image: GrayImage, filter: ImageFilter, z_standardize : bool) -> Image {
        let mut buffer = image_to_matrix(image);
        if z_standardize {
            z_standardise(&mut buffer);
        }
        Image{buffer : buffer, filter, is_standardized : z_standardize}
    }

    pub fn get_buffer(&self) -> &DMatrix<MatrixData> {
        return &self.buffer;
    }

    pub fn get_filter(&self) -> ImageFilter
    {
        return self.filter;
    }

    pub fn is_standardized(&self) -> bool {return self.is_standardized;}

    pub fn to_image(&self) -> GrayImage {
        return matrix_to_image(&self.buffer);
    }
}

// https://en.wikipedia.org/wiki/Normalization_(image_processing)
fn normalize_to_gray(value: MatrixData, min: MatrixData, max: MatrixData) -> u8 {
    let range = 255 as MatrixData; // 255 - 0
    return ((value - min) * (range / (max - min)) + min) as u8;
}

pub fn image_to_matrix(gray_image: GrayImage) -> DMatrix<MatrixData> {
    debug_assert!(gray_image.sample_layout().is_normal(NormalForm::RowMajorPacked));

    let (width, height) = gray_image.dimensions();
    let size = (width * height) as usize;
    let mut vec_column_major: Vec<MatrixData> = Vec::with_capacity(size);
    for x in 0..width {
        for y in 0..height {
            let pixel = gray_image.get_pixel(x, y);
            let pixel_value = pixel.data[0];
            vec_column_major.push(pixel_value as MatrixData);
        }
    }
    let nrows = Dynamic::new(height as usize);
    let ncols = Dynamic::new(width as usize);
    let vec_storage = VecStorage::new(nrows, ncols, vec_column_major);
    return DMatrix::from_data(vec_storage);
}

pub fn matrix_to_image(matrix: &DMatrix<MatrixData>) -> GrayImage {
    let (rows, cols) = matrix.shape();
    let min = matrix.min();
    // if max is in u8 range, use the full range
    let mut max = matrix.max();
    if max < (256 as MatrixData) {
        max = 255 as MatrixData;
    }
    let mut gray_image = DynamicImage::new_luma8(cols as u32, rows as u32).to_luma();
    for c in 0..cols {
        for r in 0..rows {
            let val = *matrix.index((r, c));
            let pixel = gray_image.get_pixel_mut(c as u32, r as u32);
            pixel.data[0] = normalize_to_gray(val, min, max);
        }
    }
    return gray_image;
}




