extern crate nalgebra as na;

use na::DMatrix;
use super::MatrixData;

pub fn column_major_index(c : usize, r : usize, height: usize) -> usize {
    return c*height + r;
}

pub fn row_major_index(c : usize, r : usize, cols: usize) -> usize {
    return r*cols + c;
}

pub fn z_standardise(matrix : &mut DMatrix<MatrixData>) -> &mut DMatrix<MatrixData> {
    let mean = matrix.mean();
    let std_dev = matrix.variance().sqrt();
    let matrix_itr = matrix.iter_mut();
    for elem in matrix_itr {
        *elem = (*elem - mean)/std_dev;
    }
    return matrix;
}