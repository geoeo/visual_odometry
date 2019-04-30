extern crate nalgebra as na;

use na::{Matrix3x4,Matrix6x1, Matrix3, Matrix4x1};
use super::MatrixData;

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
/*
def exp(w, twist_size):
w_angle = w[3:twist_size]
w_angle_transpose = np.transpose(w_angle)
w_x = Utils.skew_symmetric(w[3], w[4], w[5])
w_x_squared = np.matmul(w_x, w_x)

# closed form solution for exponential map
theta_sqred = np.matmul(w_angle_transpose, w_angle)[0][0]
theta = math.sqrt(theta_sqred)

A = 0
B = 0
C = 0

//TODO: use Taylor Expansion when theta_sqred is small
if not theta == 0:
A = math.sin(theta) / theta

if not theta_sqred == 0:
B = (1 - math.cos(theta)) / theta_sqred
C = (1 - A) / theta_sqred

u = np.array([w[0], w[1], w[2]]).reshape((3, 1))

R_new = I_3 + np.multiply(A, w_x) + np.multiply(B, w_x_squared)
V = I_3 + np.multiply(B, w_x) + np.multiply(C, w_x_squared)

t_new = np.matmul(V, u)

return R_new, t_new
*/

//TODO:
pub fn exp(w : Matrix6x1<MatrixData>) -> (Matrix3<MatrixData>, Matrix4x1<MatrixData>) {
    return (Matrix3::zeros(), Matrix4x1::zeros())
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
