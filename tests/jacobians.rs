extern crate approx;
extern crate nalgebra as na;
extern crate visual_odometry;

use approx::assert_relative_eq;
use na::{Matrix2x3,DMatrix, Matrix3x4,Matrix3x6};
use visual_odometry::MatrixData;
use visual_odometry::camera::intrinsics::Intrinsics;
use visual_odometry::numerics::lie::*;
use visual_odometry::jacobians::{perspective_jacobian,lie_jacobian};

#[allow(non_snake_case)]
#[test]
fn perspective_jacobian_test(){
    let K = Matrix3x4::<MatrixData>::new(2.0, 0.0, 5.0,0.0,
                                         0.0,1.0,0.5,0.0,
                                         0.0,0.0,1.0,0.0);
    let K_inv = Intrinsics::invert(K);
    let camera_intrinsics = Intrinsics {K,K_inv};

    let jacobian_gt = Matrix2x3::<MatrixData>::new(1.0, 0.0, -0.5,
                                                   0.0, 0.5, -0.25);
    let points = DMatrix::<MatrixData>::from_vec(3,1, vec![1.0,1.0,2.0]);

    let persp_jacobians = perspective_jacobian(&camera_intrinsics,&points);

    let persp_jacobian = persp_jacobians[0];

    assert_eq!(persp_jacobian,jacobian_gt);
}

#[allow(non_snake_case)]
#[test]
fn lie_jacobian_test() {
    let mut P = vec![1.0,2.0,1.0,1.0];
    let mut P2 = vec![2.0,1.0,2.0,1.0];
    P.append(&mut P2);

    assert_eq!(P.len(),8);

    let points = DMatrix::<MatrixData>::from_vec(4,2,P);

    let lie_jacobians = lie_jacobian(generator_x(),
                                     generator_y(),
                                     generator_z(),
                                     generator_roll(),
                                     generator_pitch(),
                                     generator_yaw(),
                                     &points);

    assert_eq!(lie_jacobians.len(),2);

    let j1 = lie_jacobians[0];
    let j2 = lie_jacobians[1];

    let j1_gt = Matrix3x6::<MatrixData>::new(1.0,0.0,0.0,0.0,1.0,-2.0,
                                             0.0,1.0,0.0,-1.0,0.0,1.0,
                                             0.0,0.0,1.0,2.0,-1.0,0.0);

    let j2_gt = Matrix3x6::<MatrixData>::new(1.0,0.0,0.0,0.0,2.0,-1.0,
                                             0.0,1.0,0.0,-2.0,0.0,2.0,
                                             0.0,0.0,1.0,1.0,-2.0,0.0);

    assert_eq!(j1,j1_gt);
    assert_eq!(j2,j2_gt);

}
