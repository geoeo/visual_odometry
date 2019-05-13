use std::path::PathBuf;
use crate::{Image, Float};
use crate::image::types::ImageFilter;
use crate::io::read_png_16bits_row_major;

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
                   depth_factor: Float,
                   filter_x: ImageFilter,
                   filter_y: ImageFilter)
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

        let image_ref = image::open(reference_image_path).unwrap().to_luma();
        let (width,height,depth_ref)
            = read_png_16bits_row_major(reference_depth_path)
            .unwrap_or_else(|_| panic!("Could not read image"));

        let image_2 = image::open(target_image_path).unwrap().to_luma();
        let (_,_,target_depth)
            = read_png_16bits_row_major(target_depth_path)
            .unwrap_or_else(|_| panic!("Could not read image"));

        let intensity_1 = Image::from_image(&image_ref, ImageFilter::None, true);
        let mut depth_1 = Image::from_vec_16(height,width,&depth_ref, false);
        let gx = Image::from_image(&image_ref, filter_x, true);
        let gy = Image::from_image(&image_ref, filter_y, true);

        let intensity_2 = Image::from_image(&image_2, ImageFilter::None, true);
        let mut depth_2 = Image::from_vec_16(height,width,&target_depth, false);
        let gx_2 = Image::from_image(&image_2, filter_x, true);
        let gy_2 = Image::from_image(&image_2, filter_y, true);

        depth_1.buffer /= depth_factor;
        depth_2.buffer /= depth_factor;
        let max_depth = depth_1.buffer.amax();

        let reference_frame = Frame{intensity:intensity_1, depth: depth_1, gradient_x: gx, gradient_y: gy};
        let target_frame = Frame{intensity:intensity_2, depth: depth_2, gradient_x: gx_2, gradient_y: gy_2};

        reference_frames.push(reference_frame);
        target_frames.push(target_frame);
        max_depths.push(max_depth);

    }

    (reference_frames, target_frames, max_depths)
}