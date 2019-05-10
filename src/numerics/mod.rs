extern crate nalgebra as na;
extern crate cv;
extern crate byteorder;

use na::{DMatrix, Matrix3, Vector3, Matrix4, U3, U1};
use crate::Float;
use cv::Mat;
use crate::image::types::ImageEncoding;
use crate::image::cv_mat_to_matrix;
use byteorder::{LittleEndian, WriteBytesExt};
use std::mem;

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

pub fn z_standardize_cv_mat(cv_mat : &mut Mat, encoding: ImageEncoding) -> () {
    let height = cv_mat.rows;
    let width = cv_mat.cols;
    let n = (height*width) as Float;
    let mut mean : Float = 0.0;
    let mut std_dev = 0.0;
    for x in 0..width {
        for y in 0..height {
            let pixel_value =
                match encoding {
                    ImageEncoding::U8 => panic!("Z-standardization only supported for floating point types"),
                    ImageEncoding::U16 =>panic!("Z-standardization only supported for floating point types"),
                    ImageEncoding::S16 =>panic!("Z-standardization only supported for floating point types"),
                    ImageEncoding::F64 =>cv_mat.at2::<f64>(y,x) as Float
                };

            mean+=pixel_value;
        }
    }

    mean /= n;

    for x in 0..width {
        for y in 0..height {
            let pixel_value =
                match encoding {
                    ImageEncoding::U8 => panic!("Z-standardization only supported for floating point types"),
                    ImageEncoding::U16 =>panic!("Z-standardization only supported for floating point types"),
                    ImageEncoding::S16 =>panic!("Z-standardization only supported for floating point types"),
                    ImageEncoding::F64 => cv_mat.at2::<f64>(y,x) as Float
                };

            let t = (pixel_value-mean);
            std_dev+=t*t;
        }
    }

    std_dev/=n;
    std_dev = std_dev.sqrt();

    for x in 0..width {
        for y in 0..height {
            let pixel_value =
                match encoding {
                    ImageEncoding::U8 => panic!("Z-standardization only supported for floating point types"),
                    ImageEncoding::U16 =>panic!("Z-standardization only supported for floating point types"),
                    ImageEncoding::S16 =>panic!("Z-standardization only supported for floating point types"),
                    ImageEncoding::F64 =>cv_mat.at2::<f64>(y,x) as Float
                };

            let v = (pixel_value-mean)/std_dev;
            match encoding {
                ImageEncoding::U8 => panic!("Z-standardization only supported for floating point types"),
                ImageEncoding::U16 =>panic!("Z-standardization only supported for floating point types"),
                ImageEncoding::S16 =>panic!("Z-standardization only supported for floating point types"),
                ImageEncoding::F64 => {
//                    let data = cv_mat.data();
//                    let pos = y as usize * cv_mat.step1(0) * cv_mat.elem_size1() + x as usize * cv_mat.step1(1) * cv_mat.elem_size1();
//                    let mut byte = &mut data[pos];
//                    let mut bs = [0u8; mem::size_of::<f64>()];
//                    bs.as_mut()
//                        .write_i64::<LittleEndian>(i)
//                        .expect("Unable to write");
//                    *byte = bs;
                    panic!("Z-standardization not yet implemented for floating point types")
                }
            };
        }
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
