extern crate nalgebra as na;

use na::{Matrix3x4,Vector6, Matrix3, Vector3};
use super::MatrixData;
use super::matrix_ops::{skew_symmetric};

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

pub fn exp(v_lie : Vector6<MatrixData>) -> (Matrix3<MatrixData>, Vector3<MatrixData>) {
    let u = Vector3::from(v_lie.fixed_rows::<na::U3>(0));
    let w = Vector3::from(v_lie.fixed_rows::<na::U3>(3));

    let w_t = w.transpose();
    let w_x = skew_symmetric(*w.index(0), *w.index(1), *w.index(2));
    let w_x_squared = w_x*w_x;
    let theta_squared = *(w_t*w).index(0);
    let theta = theta_squared.sqrt();

    //TODO: use Taylor Expansion when theta is small
    let a =
        if theta != 0.0 {
            theta.sin() / theta
        } else { 0.0 };

    let b = if theta_squared != 0.0 {
        (1.0 - theta.cos()) / theta_squared
    } else { 0.0 };

    let c = if theta_squared != 0.0 {
        (1.0 - a) / theta_squared
    } else { 0.0 };

    //TODO: when calls in const become available refactor identity()
    let so3_new = Matrix3::<MatrixData>::identity() + a*w_x + b*w_x_squared;
    let v : Matrix3<MatrixData> = Matrix3::<MatrixData>::identity() + b*w_x + c*w_x_squared;

    let t = v*u;

    return (so3_new, t)
}

pub fn ln(so3 : Matrix3<MatrixData>, t : Vector3<MatrixData>) -> Vector6<MatrixData> {

    let trace = so3.trace();
    let theta = ((trace - 1.0)/2.0).cos();
    let theta_squared = theta*theta;

    let so3_t = so3.transpose();

    //TODO: use Taylor Expansion when theta is small
    let ln_r =
        if theta != 0.0 {
            (theta / (2.0 * theta.sin())) * (so3 - so3_t)
        } else {
            Matrix3::<MatrixData>::identity()
        };

    let w_3 = *ln_r.index((2,1));
    let w_4 = *ln_r.index((0,2));
    let w_5 = *ln_r.index((1,0));

    let w_x = skew_symmetric(w_3, w_4, w_5);
    let w_x_squared = w_x*w_x;

    let a = if theta != 0.0 {
        theta.sin() / theta
    } else { 0.0 };

    let b = if theta_squared != 0.0 {
        (1.0 - theta.cos()) / theta_squared
    } else { 0.0 };

    let coeff = if theta_squared != 0.0  {
        (1.0/theta_squared)*(1.0 - (a/(2.0*b)))
    } else { 0.0 };

    let v_inv = Matrix3::<MatrixData>::identity() + 0.5*w_x + coeff*w_x_squared;

    let u = v_inv*t;

    let w_0 = *u.index(0);
    let w_1 = *u.index(1);
    let w_2 = *u.index(2);

    let w = Vector6::<MatrixData>::new(w_0,w_1,w_2,w_3,w_4,w_5);

    return w;
}