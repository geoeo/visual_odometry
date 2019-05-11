extern crate cv;

use std::path::PathBuf;
use crate::{Image, Float};
use crate::image::types::{ImageFilter, ImageEncoding};
use crate::io::read_png_16bits_row_major;
use cv::Mat;
use crate::numerics::z_standardize_cv_mat;
use self::cv::highgui::Show;

pub struct Frame {
    pub intensity : Image,
    pub depth : Image,
    pub gradient_x : Image,
    pub gradient_y : Image
}

pub fn load_frames(reference_image_paths: &Vec<PathBuf>,
                   reference_depth_paths: &Vec<PathBuf>,
                   target_image_paths: &Vec<PathBuf>,
                   target_depth_paths: &Vec<PathBuf>,
                   depth_factor: Float)
    -> (Vec<Frame>, Vec<Frame>, Vec<Float>) {
    assert_eq!(reference_image_paths.len(),target_image_paths.len());
    assert_eq!(reference_depth_paths.len(),target_depth_paths.len());
    assert_eq!(reference_image_paths.len(),reference_depth_paths.len());
    assert_eq!(reference_image_paths.len(),target_depth_paths.len());
    let samples = reference_image_paths.len();

    let mut reference_frames: Vec<Frame> = Vec::with_capacity(samples);
    let mut target_frames: Vec<Frame> = Vec::with_capacity(samples);
    let mut max_depths: Vec<Float> = Vec::with_capacity(samples);

    for i in 0..samples {
        let reference_image_path = &reference_image_paths[i];
        let reference_depth_path = &reference_depth_paths[i];
        let target_image_path = &target_image_paths[i];
        let target_depth_path = &target_depth_paths[i];



        let image_ref = Mat::from_path(reference_image_path.clone().as_os_str().to_str().expect("Failed"),
                                       cv::imgcodecs::ImageReadMode::Grayscale)
            .unwrap_or_else(|_| panic!("Could not read image"));
        let depth_ref = Mat::from_path(reference_depth_path.clone().as_os_str().to_str().expect("Failed"),
                                       cv::imgcodecs::ImageReadMode::AnyDepth)
            .unwrap_or_else(|_| panic!("Could not read image"));

        let image_ref_norm = z_standardize_cv_mat(&image_ref,ImageEncoding::U8);
        let image_ref_norm_1 = z_standardize_cv_mat(&image_ref,ImageEncoding::U8);
        let image_ref_norm_2 = z_standardize_cv_mat(&image_ref,ImageEncoding::U8);

        let target_string = target_image_path.as_os_str().to_str().expect("Failed");
        let image_target = Mat::from_path(target_string,
                                          cv::imgcodecs::ImageReadMode::Grayscale)
            .unwrap_or_else(|_| panic!("Could not read image"));
        let depth_target = Mat::from_path(target_depth_path.clone().as_os_str().to_str().expect("Failed"),
                                          cv::imgcodecs::ImageReadMode::AnyDepth)
            .unwrap_or_else(|_| panic!("Could not read image"));
        let image_target_norm = z_standardize_cv_mat(&image_target,ImageEncoding::U8);
        let image_target_norm_1 = z_standardize_cv_mat(&image_target,ImageEncoding::U8);
        let image_target_norm_2 = z_standardize_cv_mat(&image_target,ImageEncoding::U8);

        let intensity_1 = Image::from_cv_mat(&image_ref_norm, ImageFilter::None, false, ImageEncoding::F64);
        let mut depth_1 = Image::from_cv_mat(&depth_ref, ImageFilter::None, false, ImageEncoding::U16);

        let intensity_2 = Image::from_cv_mat(&image_target_norm, ImageFilter::None, false, ImageEncoding::F64);
        let mut depth_2 = Image::from_cv_mat(&depth_target, ImageFilter::None, false, ImageEncoding::U16);

        // Sobel is mutating!
        // Also!! sobel filter seems to invalidate other matrices created with z_standardize_cv_mat(&).
        // As such we need to copy these values into an image BEFORE calling sobel on subsequent matrices
        //Cv16SC1 = 3, Cv64FC1 = 6
        let ddepth = 6;
        let g_x_ref = image_ref_norm_1.sobel(ddepth,1,0,1,1.0,0.0,cv::BorderType::Default);
        let gx = Image::from_cv_mat(&g_x_ref, ImageFilter::None, false, ImageEncoding::F64);

        let g_y_ref = image_ref_norm_2.sobel(ddepth,0,1,1,1.0,0.0,cv::BorderType::Default);
        let gy = Image::from_cv_mat(&g_y_ref, ImageFilter::None, false, ImageEncoding::F64);

        let g_x_target = image_target_norm_1.sobel(ddepth,1,0,1,1.0,0.0,cv::BorderType::Default);
        let gx_2 = Image::from_cv_mat(&g_x_target, ImageFilter::None, false, ImageEncoding::F64);
        let g_y_target = image_target_norm_2.sobel(ddepth,0,1,1,1.0,0.0,cv::BorderType::Default);
        let gy_2 = Image::from_cv_mat(&g_y_target, ImageFilter::None, false, ImageEncoding::F64);


        println!("max: {}",gx.buffer.max());
        println!("min: {}",gy.buffer.min());
        println!("intenstiy max: {}",intensity_1.buffer.max());
        println!("intenisty min: {}",intensity_1.buffer.min());

        println!("max: {}",gx_2.buffer.max());
        println!("min: {}",gy_2.buffer.min());
        println!("intenstiy max: {}",intensity_2.buffer.max());
        println!("intenisty min: {}",intensity_2.buffer.min());

        let t1 = *intensity_1.buffer.index((1,1));
        let t2 = *intensity_2.buffer.index((1,1)); // TODO: Investigate -> This should not be negative

        depth_1.buffer /= depth_factor;
        depth_2.buffer /= depth_factor;
        let max_depth = depth_1.buffer.amax();

        println!("max depth: {}",max_depth);

        let reference_frame = Frame{intensity:intensity_1, depth: depth_1, gradient_x: gx, gradient_y: gy};
        let target_frame = Frame{intensity:intensity_2, depth: depth_2, gradient_x: gx_2, gradient_y: gy_2};

        reference_frames.push(reference_frame);
        target_frames.push(target_frame);
        max_depths.push(max_depth);

    }

    (reference_frames, target_frames, max_depths)
}