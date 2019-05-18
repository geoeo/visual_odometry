use std::path::PathBuf;
use crate::{Image, Float};
use crate::image::types::ImageFilter;
use crate::io::read_png_16bits_row_major;
use crate::image_pyramid::Layer;
use image::GrayImage;

//TODO: Encapsulate  (downsampled) intensity and gradients by making a Layer containing both.
// This will enable an image pyramid approach
//pub struct Frame {
//    pub intensity : Image,
//    pub depth : Image,
//    pub gradient_x : Image,
//    pub gradient_y : Image
//}

pub struct Frame {
    pub image_pyramid: Vec<Layer>,
    pub depth : Image
}

pub fn load_frames(reference_image_paths: &Vec<PathBuf>,
                   reference_depth_paths: &Vec<PathBuf>,
                   target_image_paths: &Vec<PathBuf>,
                   target_depth_paths: &Vec<PathBuf>,
                   depth_factor: Float,
                   filter_x: ImageFilter,
                   filter_y: ImageFilter,
                   standardize: bool,
                   pyramid_levels: u32,
                   sigma: f32)
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
        let image_target = image::open(target_image_path).unwrap().to_luma();

        let mut image_pyramid_ref: Vec<GrayImage> = Vec::with_capacity(pyramid_levels as usize);
        let mut image_pyramid_target: Vec<GrayImage> = Vec::with_capacity(pyramid_levels as usize);

        image_pyramid_ref.push(image_ref);
        image_pyramid_target.push(image_target);

        for layer_id in 1..pyramid_levels {
            let idx = layer_id as usize-1;
            let sampled_image_ref = Layer::blur_downsample(&image_pyramid_ref[idx],layer_id,sigma);
            let sampled_image_target = Layer::blur_downsample(&image_pyramid_target[idx],layer_id,sigma);
            image_pyramid_ref.push(sampled_image_ref);
            image_pyramid_target.push(sampled_image_target);
        }

        let layer_pyramid_ref = image_pyramid_ref
            .iter().enumerate()
            .map(|(i, x)| Layer::from_image(x, i as u32, standardize, filter_x, filter_y)).collect();

        let layer_pyramid_target = image_pyramid_target
            .iter().enumerate()
            .map(|(i, x)| Layer::from_image(x, i as u32, standardize, filter_x, filter_y)).collect();

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

        let reference_frame = Frame{image_pyramid: layer_pyramid_ref, depth: depth_1};
        let target_frame = Frame{image_pyramid: layer_pyramid_target, depth: depth_2};

        reference_frames.push(reference_frame);
        target_frames.push(target_frame);
        max_depths.push(max_depth);

    }

    (reference_frames, target_frames, max_depths)
}