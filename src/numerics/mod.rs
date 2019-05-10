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
use self::byteorder::BigEndian;

pub mod lie;
pub mod weighting;

pub fn column_major_index(r : usize, c : usize, rows: usize) -> usize {
    c*rows + r
}

pub fn row_major_index(r : usize, c : usize, cols: usize) -> usize {
    return r*cols + c;
}

const f64_size: usize = mem::size_of::<f64>();

//TODO @Investigate this.
pub fn z_standardize(matrix : &mut DMatrix<Float>) -> () {
    let mean = matrix.mean();
    let std_dev = matrix.variance().sqrt();
    let matrix_itr = matrix.iter_mut();
    for elem in matrix_itr {
        *elem = (*elem - mean)/std_dev;
    }
}

pub fn z_standardize_cv_mat(cv_mat : &Mat, encoding: ImageEncoding) -> Mat {
    let height = cv_mat.rows;
    let width = cv_mat.cols;
    let n = (height*width) as Float;
    let mut mean : Float = 0.0;
    let mut std_dev = 0.0;
    for x in 0..width {
        for y in 0..height {
            let pixel_value =
                match encoding {
                    ImageEncoding::U8 => cv_mat.at2::<u8>(y,x) as Float,
                    ImageEncoding::U16 =>cv_mat.at2::<u16>(y,x) as Float,
                    ImageEncoding::S16 =>cv_mat.at2::<i16>(y,x) as Float,
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
                    ImageEncoding::U8 => cv_mat.at2::<u8>(y,x) as Float,
                    ImageEncoding::U16 =>cv_mat.at2::<u16>(y,x) as Float,
                    ImageEncoding::S16 =>cv_mat.at2::<i16>(y,x) as Float,
                    ImageEncoding::F64 => cv_mat.at2::<f64>(y,x) as Float
                };

            let t = (pixel_value-mean);
            std_dev+=t*t;
        }
    }

    std_dev/=n;
    std_dev = std_dev.sqrt();
    let mut value_buffer: Vec<u8> = Vec::new();

    for y in 0..height {
        for x in 0..width {
            let pixel_value =
                match encoding {
                    ImageEncoding::U8 => cv_mat.at2::<u8>(y,x) as Float,
                    ImageEncoding::U16 =>cv_mat.at2::<u16>(y,x) as Float,
                    ImageEncoding::S16 =>cv_mat.at2::<i16>(y,x) as Float,
                    ImageEncoding::F64 =>cv_mat.at2::<f64>(y,x) as Float
                };

            let v = (pixel_value-mean)/std_dev;

            let mut bs = [0u8; f64_size];
            bs.as_mut()
                .write_f64::<LittleEndian>(v)
                .expect("Unable to write");
            for elem in &bs {
                value_buffer.push(*elem);
            }
        }
    }

    Mat::from_buffer(height,width,cv::CvType::Cv64FC1,&value_buffer)

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
