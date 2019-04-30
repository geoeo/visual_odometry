extern crate nalgebra as na;

use na::Matrix3x4;
use super::MatrixData;

pub fn generator_x() -> Matrix3x4<MatrixData> {
    return Matrix3x4::new(0.0, 0.0, 0.0, 1.0,
                          0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0);
}

pub fn generator_x_neg() -> Matrix3x4<MatrixData> {
    return Matrix3x4::new(0.0, 0.0, 0.0, -1.0,
                          0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0);
}

pub fn generator_y() -> Matrix3x4<MatrixData> {
    return Matrix3x4::new(0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 1.0,
                          0.0, 0.0, 0.0, 0.0);
}

pub fn generator_y_neg() -> Matrix3x4<MatrixData> {
    return Matrix3x4::new(0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, -1.0,
                          0.0, 0.0, 0.0, 0.0);
}

pub fn generator_z() -> Matrix3x4<MatrixData> {
    return Matrix3x4::new(0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 1.0);
}

pub fn generator_z_neg() -> Matrix3x4<MatrixData> {
    return Matrix3x4::new(0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, -1.0);
}

// rotation around X
pub fn generator_roll() -> Matrix3x4<MatrixData> {
    return Matrix3x4::new(0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, -1.0, 0.0,
                          0.0, 1.0, 0.0, 0.0);
}

pub fn generator_roll_neg() -> Matrix3x4<MatrixData> {
    return Matrix3x4::new(0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 1.0, 0.0,
                          0.0, -1.0, 0.0, 0.0);
}

// rotation around Y
pub fn generator_pitch() -> Matrix3x4<MatrixData> {
    return Matrix3x4::new(0.0, 0.0, 1.0, 0.0,
                          0.0, 0.0, 0.0, 0.0,
                          -1.0, 0.0, 0.0, 0.0);
}

pub fn generator_pitch_neg() -> Matrix3x4<MatrixData> {
    return Matrix3x4::new(0.0, 0.0, -1.0, 0.0,
                          0.0, 0.0, 0.0, 0.0,
                          1.0, 0.0, 0.0, 0.0);
}

// rotation around Z
pub fn generator_yaw() -> Matrix3x4<MatrixData> {
    return Matrix3x4::new(0.0, -1.0, 0.0, 0.0,
                          1.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0);
}

pub fn generator_yaw_neg() -> Matrix3x4<MatrixData> {
    return Matrix3x4::new(0.0, 1.0, 0.0, 0.0,
                          -1.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0);
}
