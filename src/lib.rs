use crate::image::Image;

pub mod image;
pub mod numerics;
pub mod camera;
pub mod jacobian;

pub type MatrixData = f64;

pub struct Frame {
    intensity : Image,
    depth : Image,
    gradient_x : Option<Image>,
    gradient_y : Option<Image>
}