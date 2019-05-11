extern crate  nalgebra as na;

use na::Matrix3;
use crate::Float;

pub fn horizontal_sobel() -> Matrix3<Float> {
    Matrix3::<Float>::new(
        -1.0, 0.0, 1.0,
        -2.0, 0.0, 2.0,
        -1.0, 0.0, 1.0)
}

pub fn horizontal_sobel_vec() -> Vec<Float> {
    vec!(-1.0, -2.0, -1.0, 0.0 ,0.0 ,0.0 ,1.0, 2.0, 1.0)
}


pub fn vertical_sobel() -> Matrix3<Float> {
    Matrix3::<Float>::new(
        -1.0, -2.0, -1.0,
        0.0, 0.0, 0.0,
        1.0, 2.0, 1.0
    )
}

pub fn vertical_sobel_vec() -> Vec<Float> {
    vec!(-1.0, 0.0, 1.0, -2.0 ,0.0 ,2.0 ,-1.0, 0.0, 1.0)
}

pub fn horizontal_scharr() -> Matrix3<Float> {
    Matrix3::<Float>::new(
        -3.0, 0.0, 3.0,
        -10.0, 0.0, 10.0,
        -3.0, 0.0, 3.0)
}


pub fn vertical_scharr() -> Matrix3<Float> {
    Matrix3::<Float>::new(
        -3.0, -10.0, -3.0,
        0.0, 0.0, 0.0,
        3.0, 10.0, 3.0
    )
}


pub static HORIZONTAL_SOBEL: [f32; 9] = [
    -1.0, 0.0, 1.0,
    -2.0, 0.0, 2.0,
    -1.0, 0.0, 1.0];

pub static VERTICAL_SOBEL: [f32; 9] = [
    -1.0, -2.0, -1.0,
    0.0,  0.0,  0.0,
    1.0,  2.0,  1.0];



pub static HORIZONTAL_SCHARR: [f32; 9] = [
    -3.0, 0.0, 3.0,
    -10.0, 0.0, 10.0,
    -3.0, 0.0, 3.0];

pub static VERTICAL_SCHARR: [f32; 9] = [
    -3.0, -10.0, -3.0,
    0.0,  0.0,  0.0,
    3.0, 10.0, 3.0];


