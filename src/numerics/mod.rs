extern crate nalgebra as na;

use na::{DMatrix, Matrix3, Vector3, Vector6, Matrix4, U3, U1};
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

#[allow(non_snake_case)]
pub fn lm_gamma(F_prev: Float,
                F: Float,
                lie_new: Vector6<Float>,
                g: Vector6<Float>,
                mu: Float) -> Float {
    let lie_new_trans = lie_new.transpose();
    let numerator = F_prev - F;
    let denom_inner = mu*(lie_new - g);
    let single = lie_new_trans*denom_inner;
    let single_val = single.index(0);
    numerator/(mu*single_val)
}

//@GPU
// https://en.wikipedia.org/wiki/Kernel_(image_processing)
// Actually the kernel has to be flipped accross x and y? But kernels seem to work as is
pub fn filter3x3(kernel: &Matrix3<Float>, matrix: &DMatrix<Float>) -> DMatrix<Float> {
    let width = matrix.ncols();
    let height = matrix.nrows();
    let size = width*height;
    let kernel_size = 3;
    let kernel_min = (kernel_size-1)/2;
    let mut vec_column_major: Vec<Float> = Vec::with_capacity(size);
    let width_i32 = width as Unsigned;
    let height_i32 = height as Unsigned;
    for x in 0..width_i32 {
        'image: for y in 0..height_i32 {
            let mut value = 0.0;
            let mut out_of_range_flag = false;
            for i in 0..kernel_size {
                for j in 0..kernel_size {
                    let i_matrix = x + i - kernel_min;
                    let j_matrix = y + j - kernel_min;
                    let i_kernel = i;
                    let j_kernel = j;
                    let convolved_value =
                        match is_within_kernel_bounds(j_matrix,i_matrix,width_i32,height_i32){
                            true => {
                                //Cant be negative, safe to cast
                                let kernel_value = *kernel.index((j_kernel as  usize,i_kernel as usize));
                                let pixel_value = *matrix.index((j_matrix as usize,i_matrix as usize));
                                kernel_value*pixel_value
                            },
                            false => {
                                //TODO: implement different bound behaviours
                                out_of_range_flag = true;
                                0.0
                            }
                        };
                    value += convolved_value;
                }
            }
            if !out_of_range_flag{
                vec_column_major.push(value);
            } else {
                vec_column_major.push(0.0);
            }

        }
    }

    DMatrix::<Float>::from_vec(height,width, vec_column_major)
}

fn is_within_kernel_bounds(j: Unsigned, i: Unsigned ,width: Unsigned ,height: Unsigned) -> bool {

    let is_j_in_range = j >= 0 && j < height;
    let is_i_in_range = i >= 0 && i < width;
    return is_j_in_range && is_i_in_range;

}
