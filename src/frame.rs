use std::path::PathBuf;
use crate::{Image, Float};
use crate::image::types::ImageFilter;
use crate::io::read_png_16bits_row_major;

//TODO: Encapsulate intensity and gradients by making a vector containing both.
// This will enable an image pyramid approach
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

        //TODO: This will get refactored into a layer generation
        // --
        let image_ref = image::open(reference_image_path).unwrap().to_luma();
        let image_target = image::open(target_image_path).unwrap().to_luma();

        let intensity_ref = Image::from_image(&image_ref, ImageFilter::None, true);
        let gx_ref = Image::from_image(&image_ref, filter_x, true);
        let gy_ref = Image::from_image(&image_ref, filter_y, true);

        let intensity_target = Image::from_image(&image_target, ImageFilter::None, true);
        let gx_target = Image::from_image(&image_target, filter_x, true);
        let gy_target = Image::from_image(&image_target, filter_y, true);
        // --

        let (width,height,depth_ref)
            = read_png_16bits_row_major(reference_depth_path)
            .unwrap_or_else(|_| panic!("Could not read image"));
        let (_,_,target_depth)
            = read_png_16bits_row_major(target_depth_path)
            .unwrap_or_else(|_| panic!("Could not read image"));

        let mut depth_1 = Image::from_vec_16(height,width,&depth_ref, false);
        let mut depth_2 = Image::from_vec_16(height,width,&target_depth, false);
        depth_1.buffer /= depth_factor;
        depth_2.buffer /= depth_factor;
        let max_depth = depth_1.buffer.amax();

        let reference_frame = Frame{intensity:intensity_ref, depth: depth_1, gradient_x: gx_ref, gradient_y: gy_ref};
        let target_frame = Frame{intensity:intensity_target, depth: depth_2, gradient_x: gx_target, gradient_y: gy_target};

        reference_frames.push(reference_frame);
        target_frames.push(target_frame);
        max_depths.push(max_depth);

    }

    (reference_frames, target_frames, max_depths)
}