use crate::image::Image;

pub mod image;
pub mod numerics;
pub mod camera;
pub mod jacobians;
pub mod gauss_newton_routines;

pub type MatrixData = f64; //Rename this to Float?

pub struct Frame {
    intensity : Image,
    depth : Image,
    gradient_x : Option<Image>,
    gradient_y : Option<Image>
}