extern crate nalgebra as na;

use na::{Matrix,Matrix1x2,Matrix2x3,Dynamic,DMatrix, Matrix3x4,Matrix3x6,U3,U6,DVector, VecStorage};
use crate::{Float, HomogeneousBackProjections};
use crate::camera::intrinsics::Intrinsics;

pub fn image_jacobian(gradient_x: &DMatrix<Float>, gradient_y: &DMatrix<Float>, px: usize, py: usize) -> Matrix1x2<Float> {
    let index = (py, px);
    let gx = *gradient_x.index(index);
    let gy = *gradient_y.index(index);
    return Matrix1x2::<Float>::new(gx, gy);
}


#[allow(non_snake_case)]
pub fn perspective_jacobians(camera_intrinsics: &Intrinsics, world_points: &HomogeneousBackProjections)
                             -> Vec<Matrix2x3<Float>> {
    let N = world_points.ncols();
    let fx = camera_intrinsics.fx();
    let fy = camera_intrinsics.fy();
    let mut persp_jacobians: Vec<Matrix2x3<Float>> = Vec::with_capacity(N);
    for P in world_points.column_iter() {
        let X = *P.index((0, 0));
        let Y = *P.index((1, 0));
        let Z = *P.index((2, 0));
        let Z_sqrd = Z * Z;

        let (v00, v11, v02, v12) =
            if Z != 0.0 && Z_sqrd.is_finite() {
                (fx / Z, fy / Z, (-fx * X) / Z_sqrd, (-fy * Y) / Z_sqrd)
            } else {
                (0.0, 0.0, 0.0, 0.0)
            };

        let persp_jacobian = Matrix2x3::<Float>::new(v00, 0.0, v02,
                                                     0.0, v11, v12);
        persp_jacobians.push(persp_jacobian);
    }
    persp_jacobians
}

#[allow(non_snake_case)]
pub fn lie_jacobians(generator_x: Matrix3x4<Float>,
                     generator_y: Matrix3x4<Float>,
                     generator_z: Matrix3x4<Float>,
                     generator_roll: Matrix3x4<Float>,
                     generator_pitch: Matrix3x4<Float>,
                     generator_yaw: Matrix3x4<Float>,
                     Y_est: &HomogeneousBackProjections)
                     -> Vec<Matrix3x6<Float>> {
    let N = Y_est.ncols();

    let G_1_y = generator_x*Y_est;
    let G_1_y_stacked = stack_columns(G_1_y);

    let G_2_y = generator_y*Y_est;
    let G_2_y_stacked = stack_columns(G_2_y);

    let G_3_y = generator_z*Y_est;
    let G_3_y_stacked = stack_columns(G_3_y);

    let G_4_y = generator_roll*Y_est;
    let G_4_y_stacked = stack_columns(G_4_y);

    let G_5_y = generator_pitch*Y_est;
    let G_5_y_stacked = stack_columns(G_5_y);

    let G_6_y = generator_yaw*Y_est;
    let G_6_y_stacked = stack_columns(G_6_y);

    let G = Matrix::<Float, Dynamic, U6, VecStorage<Float, Dynamic, U6>>::from_columns(&[G_1_y_stacked,
        G_2_y_stacked,
        G_3_y_stacked,
        G_4_y_stacked,
        G_5_y_stacked,
        G_6_y_stacked]);
    let rows = G.nrows();
    let mut G_vec: Vec<Matrix3x6<Float>> = Vec::with_capacity(N);
    for r_start in (0..rows).step_by(3) {
        let G_sub = G.fixed_rows::<U3>(r_start).clone_owned();
        G_vec.push(G_sub);
    }

    G_vec
}

fn stack_columns(m: Matrix<Float, U3, Dynamic, VecStorage<Float, U3, Dynamic>>)
                 -> DVector<Float> {
    let dim = m.nrows()*m.ncols();
    let mut stacked_vec: Vec<Float>  = Vec::with_capacity(dim);
    for val in m.iter() {
        stacked_vec.push(*val);
    }
    DVector::from_vec(stacked_vec)
}