extern crate nalgebra as na;

use na::{Matrix, Matrix4, U3, U4, Dynamic, Vector6, VecStorage, DVector};
use crate::image::Image;

pub mod image;
pub mod numerics;
pub mod camera;
pub mod jacobians;
pub mod gauss_newton_routines;

pub type MatrixData = f64; //Rename this to Float?
// Image coordinates with a "depth" fixed at 1.0 i.e. (u, v, 1.0)
pub type NormalizedImageCoordinates = Matrix<MatrixData, U3, Dynamic, VecStorage<MatrixData, U3, Dynamic>>;
// Homogeneous 3D coordinates i.e. (X, Y, Z, 1.0)
pub type HomogeneousBackProjections = Matrix<MatrixData, U4, Dynamic, VecStorage<MatrixData, U4, Dynamic>>;

pub struct Frame {
    intensity : Image,
    depth : Image,
    gradient_x : Option<Image>,
    gradient_y : Option<Image>
}

#[allow(non_snake_case)]
pub fn solve(reference: Frame,
             target: Frame,
             max_its: usize,
             eps: MatrixData,
             alpha_step: MatrixData,
             max_depth: MatrixData,
             var_eps: MatrixData,
             var_min: MatrixData,
             image_range_offset: usize)
    -> (Matrix4<MatrixData>, Vector6<MatrixData>) {
    let mut lie = Vector6::<MatrixData>::zeros();
    let mut SE3 = Matrix4::<MatrixData>::zeros();

    let image_width = reference.intensity.buffer.ncols();
    let image_height = reference.intensity.buffer.nrows();
    let N = image_width*image_height;

    let init_vec_zeros = vec![0.0 as MatrixData;N];
    let init_vec_ones = vec![1.0 as MatrixData;N];
    let mut residuals = DVector::<MatrixData>::from_vec(init_vec_zeros);
    let mut weights = DVector::<MatrixData>::from_vec(init_vec_ones);



    (SE3, lie)
}