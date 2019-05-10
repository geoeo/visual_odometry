extern crate image;
extern crate nalgebra as na;

pub mod filters;
pub mod types;

use image::{GrayImage,DynamicImage};
use image::flat::NormalForm;
use image::imageops::filter3x3;
use na::DMatrix;
use crate::Float;
use crate::numerics::{z_standardize, row_major_index};
use self::types::{ImageFilter,ImageEncoding};
use cv::mat::Mat;

pub struct Image {
    pub buffer: DMatrix<Float>,
    pub filter: ImageFilter,
    pub is_standardized : bool,
    pub original_encoding: ImageEncoding}

impl Image {
    pub fn new(buffer: DMatrix<Float>, filter: ImageFilter, is_standardized : bool, original_encoding: ImageEncoding) -> Image {
        Image { buffer, filter, is_standardized,original_encoding}
    }

    pub fn from_image(image: &GrayImage, filter: ImageFilter, standardize : bool) -> Image {
        let filter_type = select_filter(filter);
        let mut buffer =
            match filter_type {
                None => image_to_matrix(image),
                Some(filter) => image_to_matrix(&filter3x3(image,&filter))
        };
        //TODO @Investigate -> maybe standardarize before filter
        if standardize {
            z_standardize(&mut buffer);
        }

        Image{ buffer, filter, is_standardized : standardize, original_encoding: ImageEncoding::U8 }
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

    pub fn from_cv_mat(image: Mat, filter: ImageFilter, standardize : bool, original_encoding: ImageEncoding) -> Image {
        let mut buffer = cv_mat_to_matrix(image,original_encoding);
        if standardize {
            z_standardize(&mut buffer);
        }
        Image{ buffer, filter, is_standardized : standardize, original_encoding}
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
    for c in 0..cols {
        for r in 0..rows {
            let val = *matrix.index((r, c));
            let mut pixel = gray_image.get_pixel_mut(c as u32, r as u32);
            pixel.data[0] = encoding.normalize_to_gray(val);
        }
    }
    gray_image
}


pub fn cv_mat_to_matrix(cv_mat: Mat, original_encoding: ImageEncoding) -> DMatrix<Float> {

    let height = cv_mat.rows;
    let width = cv_mat.cols;
    let size = (height*width) as usize;
    let mut vec_column_major: Vec<Float> = Vec::with_capacity(size);
    for x in 0..width {
        for y in 0..height {
            let pixel_value =
                match original_encoding {
                ImageEncoding::U8 => cv_mat.at2::<u8>(y,x) as Float,
                ImageEncoding::U16 =>cv_mat.at2::<u16>(y,x) as Float,
                ImageEncoding::S16 =>cv_mat.at2::<i16>(y,x) as Float,
                ImageEncoding::F64 =>cv_mat.at2::<f64>(y,x) as Float
            };

            vec_column_major.push(pixel_value);
        }
    }
    DMatrix::<Float>::from_vec(height as usize,width as usize ,vec_column_major)
}


pub fn select_filter(filter_type: ImageFilter) -> Option<[f32;9]>  {
    return match filter_type {
        ImageFilter::SobelX => Some(filters::HORIZONTAL_SOBEL),
        ImageFilter::SobelY => Some(filters::VERTICAL_SOBEL),
        ImageFilter::ScharrX => Some(filters::HORIZONTAL_SCHARR),
        ImageFilter::ScharrY => Some(filters::VERTICAL_SCHARR),
        ImageFilter::None => None
    };
}
