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

pub fn exp(v : Vector6<MatrixData>) -> (Matrix3<MatrixData>, Vector3<MatrixData>) {
    let u = Vector3::from(v.fixed_rows::<na::U3>(0));
    let w = Vector3::from(v.fixed_rows::<na::U3>(3));

    let w_t = w.transpose();
    let w_x = skew_symmetric(w);
    let w_x_squared = w_x*w_x;
    let theta_squared_mat = w_t*w;
    let theta_squared = *theta_squared_mat.index(0);
    let theta = theta_squared.sqrt();

    let mut a : MatrixData = 0.0;
    let mut b : MatrixData = 0.0;
    let mut c : MatrixData = 0.0;

    //TODO: use Taylor Expansion for Sin when theta_sqred is small
    if theta != 0.0 {
        a = theta.sin() / theta;
    }

    //TODO: use Taylor Expansion for Cos when theta_sqred is small
    if theta_squared != 0.0 {
        b = (1.0 - theta.cos()) / theta_squared;
        c = (1.0 - a) / theta_squared;
    }

    //TODO: when calls in const become available refactor identity()
    let rot_new_mat = Matrix3::<MatrixData>::identity() + a*w_x + b*w_x_squared;
    let v_mat = Matrix3::<MatrixData>::identity() + b*w_x + c*w_x_squared;

    let t = v_mat*u;

    return (rot_new_mat, t)
}

/*
def ln(R, t,twist_size):
w = np.zeros((twist_size,1),dtype=matrix_data_type)

trace = np.trace(R)
theta = math.acos((trace-1.0)/2.0)
theta_sqred = math.pow(theta,2.0)

R_transpose = np.transpose(R)
ln_R = I_3

//TODO: use Taylor Expansion when theta is small
if not theta == 0:
ln_R = (theta/(2*math.sin(theta)))*(R-R_transpose)

w[3] = ln_R[2,1]
w[4] = ln_R[0,2]
w[5] = ln_R[1,0]

w_x = Utils.skew_symmetric(w[3], w[4], w[5])
w_x_squared = np.matmul(w_x, w_x)

A = 0
B = 0
coeff = 0

//TODO: use Taylor Expansion when theta_sqred is small
if not theta == 0:
A = math.sin(theta) / theta

if not theta_sqred == 0:
B = (1 - math.cos(theta)) / theta_sqred

if not (theta == 0 or theta_sqred == 0):
coeff = (1.0/(theta_sqred))*(1.0 - (A/(2.0*B)))

V_inv = I_3 + np.multiply(0.5,w_x) + np.multiply(coeff,w_x_squared)

u = np.matmul(V_inv,t)

w[0] = u[0]
w[1] = u[1]
w[2] = u[2]

return w
*/

pub fn ln(rot_mat : Matrix3<MatrixData>, t : Vector3<MatrixData>) -> Vector6<MatrixData> {
    return Vector6::<MatrixData>::zeros();
}