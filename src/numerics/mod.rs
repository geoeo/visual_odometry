extern crate nalgebra as na;

use na::{DMatrix, Matrix3, Vector3, Matrix4, U3, U1};
use crate::Float;

pub mod lie;
pub mod weighting;

pub fn column_major_index(r : usize, c : usize, rows: usize) -> usize {
    c*rows + r
}

pub fn row_major_index(r : usize, c : usize, cols: usize) -> usize {
    return r*cols + c;
}

//TODO @Investigate this.
pub fn z_standardize(matrix : &mut DMatrix<Float>) -> () {
    let mean = matrix.mean();
    let std_dev = matrix.variance().sqrt();
    let matrix_itr = matrix.iter_mut();
    for elem in matrix_itr {
        *elem = (*elem - mean)/std_dev;
    }
}

pub fn skew_symmetric(a : Float, b : Float, c : Float) -> Matrix3<Float> {
    Matrix3::<Float>::new(0.0, -c, b,
                          c, 0.0, -a,
                          -b, a, 0.0)
}

#[allow(non_snake_case)]
pub fn isometry_from_parts(SO3: Matrix3<Float>, t: Vector3<Float>) -> Matrix4<Float> {
    Matrix4::<Float>::new(*SO3.index((0, 0)), *SO3.index((0, 1)), *SO3.index((0, 2)), *t.index(0),
                          *SO3.index((1, 0)), *SO3.index((1, 1)), *SO3.index((1, 2)), *t.index(1),
                          *SO3.index((2, 0)), *SO3.index((2, 1)), *SO3.index((2, 2)), *t.index(2),
                          0.0, 0.0, 0.0, 1.0)


}

#[allow(non_snake_case)]
pub fn parts_from_isometry(SE3 : Matrix4<Float>) -> (Matrix3<Float>, Vector3<Float>) {
    let SO3 = SE3.fixed_slice::<U3,U3>(0,0).clone_owned();
    let t = SE3.fixed_slice::<U3,U1>(0,3).clone_owned();
    (SO3,t)

}
