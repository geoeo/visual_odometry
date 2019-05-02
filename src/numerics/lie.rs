extern crate nalgebra as na;

use na::{Matrix3x4,Vector6, Matrix3, Vector3};
use crate::MatrixData;
use super::{skew_symmetric};

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

#[allow(non_snake_case)]
pub fn exp(v_lie : Vector6<MatrixData>) -> (Matrix3<MatrixData>, Vector3<MatrixData>) {
    let u = Vector3::from(v_lie.fixed_rows::<na::U3>(0));
    let w = Vector3::from(v_lie.fixed_rows::<na::U3>(3));

    let w_t = w.transpose();
    let W_x = skew_symmetric(*w.index(0), *w.index(1), *w.index(2));
    let W_x_squared = W_x*W_x;
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
    let SO3_new = Matrix3::<MatrixData>::identity() + a*W_x + b*W_x_squared;
    let V : Matrix3<MatrixData> = Matrix3::<MatrixData>::identity() + b*W_x + c*W_x_squared;

    let t = V*u;

    return (SO3_new, t)
}

#[allow(non_snake_case)]
pub fn ln(SO3 : Matrix3<MatrixData>, t : Vector3<MatrixData>) -> Vector6<MatrixData> {

    let trace = SO3.trace();
    let theta = ((trace - 1.0)/2.0).acos();
    let theta_squared = theta*theta;

    let SO3_t = SO3.transpose();

    //TODO: use Taylor Expansion when theta is small
    let ln_r =
        if theta != 0.0 {
            (theta / (2.0 * theta.sin())) * (SO3 - SO3_t)
        } else {
            Matrix3::<MatrixData>::identity()
        };

    let w_3 = *ln_r.index((2,1));
    let w_4 = *ln_r.index((0,2));
    let w_5 = *ln_r.index((1,0));

    let W_x = skew_symmetric(w_3, w_4, w_5);
    let W_x_squared = W_x*W_x;

    let a = if theta != 0.0 {
        theta.sin() / theta
    } else { 0.0 };

    let b = if theta_squared != 0.0 {
        (1.0 - theta.cos()) / theta_squared
    } else { 0.0 };

    let coeff = if theta_squared != 0.0  {
        (1.0/theta_squared)*(1.0 - (a/(2.0*b)))
    } else { 0.0 };

    let V_inv = Matrix3::<MatrixData>::identity() - 0.5*W_x + coeff*W_x_squared;

    let u = V_inv*t;

    let w_0 = *u.index(0);
    let w_1 = *u.index(1);
    let w_2 = *u.index(2);

    let w = Vector6::<MatrixData>::new(w_0, w_1,w_2, w_3, w_4, w_5);

    return w;
}