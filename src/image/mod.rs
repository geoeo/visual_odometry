extern crate image as image_rs;
extern crate nalgebra as na;

pub mod filters;
pub mod types;

use image_rs::{GrayImage, DynamicImage};
use image_rs::flat::NormalForm;
use na::{Matrix3,DMatrix};
use crate::Float;
use crate::numerics::{z_standardize, row_major_index,filter3x3};
use self::types::{ImageFilter, ImageEncoding};

#[derive(Debug,Clone)]
pub struct Image {
    pub buffer: DMatrix<Float>,
    pub filter: ImageFilter,
    pub is_standardized : bool,
    pub original_encoding: ImageEncoding}

impl Image {
    pub fn from_matrix(matrix: &DMatrix<Float>, filter: ImageFilter, standardize : bool, original_encoding: ImageEncoding) -> Image {
        let mut buffer = matrix.clone();
        let filter_type = select_filter(filter);

        if standardize {
            z_standardize(&mut buffer);
        }

        let filtered_buffer =
            match filter_type {
                None => buffer,
                Some(filter) => filter3x3(&filter,&buffer)
            };

        Image{ buffer: filtered_buffer, filter, is_standardized : standardize, original_encoding }
    }

    pub fn from_image(image: &GrayImage, filter: ImageFilter, standardize : bool) -> Image {
        let mut buffer = image_to_matrix(image);
        let filter_type = select_filter(filter);
        if standardize {
            z_standardize(&mut buffer);
        }

        let filtered_buffer =
            match filter_type {
                None => buffer,
                Some(filter) => filter3x3(&filter,&buffer)
            };


        let encoding =
            match standardize {
                true => ImageEncoding::F64,
                false => ImageEncoding::U8
            };

        Image{ buffer: filtered_buffer, filter, is_standardized : standardize, original_encoding: encoding }
    }

    pub fn from_vec_16(height: usize, width: usize, vec_16: &Vec<u16>, standardize : bool) -> Image {
        let mut buffer = vec_16_to_matrix(height, width, vec_16);
        if standardize {
            z_standardize(&mut buffer);
        }
        Image{
            buffer,
            filter: ImageFilter::None,
            is_standardized : standardize,
            original_encoding: ImageEncoding::U16
        }
    }

    pub fn to_image(&self) -> GrayImage {
        return matrix_to_image(&self.buffer, self.original_encoding);
    }
}

fn image_to_matrix(gray_image: &GrayImage) -> DMatrix<Float> {
    debug_assert!(gray_image.sample_layout().is_normal(NormalForm::RowMajorPacked));

    let (width, height) = gray_image.dimensions();
    let size = (width * height) as usize;
    let mut vec_column_major: Vec<Float> = Vec::with_capacity(size);
    for x in 0..width {
        for y in 0..height {
            let pixel = gray_image.get_pixel(x, y);
            let pixel_value = pixel.data[0];
            vec_column_major.push(pixel_value as Float);
        }
    }
    DMatrix::<Float>::from_vec(height as usize, width as usize, vec_column_major)
}

// byte size hardcoded for now
fn vec_16_to_matrix(height: usize, width: usize, vec_16: &Vec<u16>) -> DMatrix<Float> {
    let size = width*height;
    let mut vec_column_major: Vec<Float> = Vec::with_capacity(size);
    for x in 0..width {
        for y in 0..height {
            let idx = row_major_index(y,x,width);
            let pixel_value = vec_16[idx];
            vec_column_major.push(pixel_value as Float);
        }
    }
    DMatrix::<Float>::from_vec(height, width, vec_column_major)
}

fn matrix_to_image(matrix: &DMatrix<Float>, encoding: ImageEncoding) -> GrayImage {
    let (rows, cols) = matrix.shape();

    let mut gray_image = DynamicImage::new_luma8(cols as u32, rows as u32).to_luma();
    let max = matrix.max();
    let min = matrix.min();
    for c in 0..cols {
        for r in 0..rows {
            let val = *matrix.index((r, c));
            let mut pixel = gray_image.get_pixel_mut(c as u32, r as u32);
            pixel.data[0] = encoding.normalize_to_gray(max,min,val);
        }
    }
    gray_image
}

pub fn select_filter(filter_type: ImageFilter) -> Option<Matrix3<Float>>  {
    return match filter_type {
        ImageFilter::SobelX => Some(filters::horizontal_sobel()),
        ImageFilter::SobelY => Some(filters::vertical_sobel()),
        ImageFilter::ScharrX => Some(filters::horizontal_scharr()),
        ImageFilter::ScharrY => Some(filters::vertical_scharr()),
        ImageFilter::None => None
    };
}
