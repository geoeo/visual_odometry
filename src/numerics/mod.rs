extern crate nalgebra as na;

use na::{DMatrix, Matrix3, Vector3, Matrix4, U3, U1};
use crate::{Float,Unsigned};

pub mod lie;
pub mod weighting;

pub fn column_major_index(r : usize, c : usize, rows: usize) -> usize {
    c*rows + r
}

pub fn row_major_index(r : usize, c : usize, cols: usize) -> usize {
    return r*cols + c;
}

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

// https://en.wikipedia.org/wiki/Kernel_(image_processing)
// Actually the kernel has to be flipped accross x and y.
// But filters are symmetric!
pub fn filter3x3(kernel: &Matrix3<Float>, matrix: DMatrix<Float>) -> DMatrix<Float> {
    let width = matrix.ncols();
    let height = matrix.nrows();
    let size = width*height;
    let kernel_size = 3;
    let mut vec_column_major: Vec<Float> = Vec::with_capacity(size);
    let width_i32 = width as Unsigned;
    let height_i32 = height as Unsigned;
    for x in 0..width_i32 {
        for y in 0..height_i32 {
            let mut value = 0.0;
            let mut valid_count = 0.0;
            for i in (x-kernel_size)..(x+kernel_size) {
                for j in (y-kernel_size)..(y+kernel_size) {
                    let convolved_value =
                        match is_within_kernel_bounds(y,x,kernel_size,width_i32,height_i32) {
                            true => {
                                //Cant be negative, safe to cast
                                let kernel_value = *kernel.index((j as  usize,i as usize));
                                let pixel_value = *matrix.index((j as usize,i as usize));
                                valid_count+=1.0;
                                kernel_value*pixel_value
                            },
                            false => 0.0 //TODO: implement different bound behaviours
                        };
                    value += convolved_value;
                }
            }
            let avg_value = value/valid_count;
            vec_column_major.push(avg_value);
        }
    }

    DMatrix::<Float>::from_vec(height,width, vec_column_major)
}

fn is_within_kernel_bounds(j: Unsigned, i: Unsigned, kernel_size: Unsigned ,width: Unsigned ,height: Unsigned) -> bool {

    let is_j_in_range = j >= 0 && j < height;
    let is_i_in_range = i >= 0 && i < width;
    return is_j_in_range && is_i_in_range;

}
