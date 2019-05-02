extern crate nalgebra as na;

use na::{DMatrix, Matrix3, Vector3, Matrix4, U3, U1};
use crate::MatrixData;

pub mod lie;

pub fn column_major_index(c : usize, r : usize, height: usize) -> usize {
    return c*height + r;
}

pub fn row_major_index(c : usize, r : usize, cols: usize) -> usize {
    return r*cols + c;
}

pub fn z_standardize(matrix : &mut DMatrix<MatrixData>) -> () {
    let mean = matrix.mean();
    let std_dev = matrix.variance().sqrt();
    let matrix_itr = matrix.iter_mut();
    for elem in matrix_itr {
        *elem = (*elem - mean)/std_dev;
    }
}

pub fn skew_symmetric(a : MatrixData, b : MatrixData, c : MatrixData) -> Matrix3<MatrixData> {
    return Matrix3::<MatrixData>::new(0.0, -c, b,
                                      c, 0.0, -a,
                                      -b, a, 0.0);
}

#[allow(non_snake_case)]
pub fn isometry_from_parts(SO3: Matrix3<MatrixData>, t: Vector3<MatrixData>) -> Matrix4<MatrixData> {
    return Matrix4::<MatrixData>::new(*SO3.index((0, 0)), *SO3.index((0, 1)), *SO3.index((0, 2)), *t.index(0),
                                      *SO3.index((1, 0)), *SO3.index((1, 1)), *SO3.index((1, 2)), *t.index(1),
                                      *SO3.index((2, 0)), *SO3.index((2, 1)), *SO3.index((2, 2)), *t.index(2),
                                      0.0, 0.0, 0.0, 1.0);


}

#[allow(non_snake_case)]
pub fn parts_from_isometry(SE3 : Matrix4<MatrixData>) -> (Matrix3<MatrixData>, Vector3<MatrixData>) {
    let SO3 = SE3.fixed_slice::<U3,U3>(0,0).clone_owned();
    let t = SE3.fixed_slice::<U3,U1>(0,3).clone_owned();
    return (SO3,t);

}
