extern crate approx;
extern crate nalgebra as na;
extern crate visual_odometry;

use na::{Matrix2x3, Matrix3x4,Matrix3x6};
use visual_odometry::{Float, HomogeneousBackProjections};
use visual_odometry::camera::intrinsics::Intrinsics;
use visual_odometry::numerics::lie::*;
use visual_odometry::jacobians::{perspective_jacobians, lie_jacobians};

#[test]
#[allow(non_snake_case)]
fn perspective_jacobian_test(){
    let K = Matrix3x4::<Float>::new(2.0, 0.0, 5.0, 0.0,
                                    0.0, 1.0, 0.5, 0.0,
                                    0.0, 0.0, 1.0, 0.0);
    let K_inv = Intrinsics::invert(K);
    let camera_intrinsics = Intrinsics {K,K_inv};

    let jacobian_gt = Matrix2x3::<Float>::new(1.0, 0.0, -0.5,
                                              0.0, 0.5, -0.25);
    let points = HomogeneousBackProjections::from_vec(vec![1.0,1.0,2.0,1.0]);

    let persp_jacobians = perspective_jacobians(&camera_intrinsics, &points);

    let persp_jacobian = persp_jacobians[0];

    assert_eq!(persp_jacobian,jacobian_gt);
}

#[test]
#[allow(non_snake_case)]
fn lie_jacobian_test() {
    let mut P = vec![1.0,2.0,1.0,1.0];
    let mut P2 = vec![2.0,1.0,2.0,1.0];
    P.append(&mut P2);

    assert_eq!(P.len(),8);

    let points = HomogeneousBackProjections::from_vec(P);

    let lie_jacobians = lie_jacobians(generator_x(),
                                      generator_y(),
                                      generator_z(),
                                      generator_roll(),
                                      generator_pitch(),
                                      generator_yaw(),
                                      &points);

    assert_eq!(lie_jacobians.len(),2);

    let j1 = lie_jacobians[0];
    let j2 = lie_jacobians[1];

    let j1_gt = Matrix3x6::<Float>::new(1.0, 0.0, 0.0, 0.0, 1.0, -2.0,
                                        0.0, 1.0, 0.0, -1.0, 0.0, 1.0,
                                        0.0, 0.0, 1.0, 2.0, -1.0, 0.0);

    let j2_gt = Matrix3x6::<Float>::new(1.0, 0.0, 0.0, 0.0, 2.0, -1.0,
                                        0.0, 1.0, 0.0, -2.0, 0.0, 2.0,
                                        0.0, 0.0, 1.0, 1.0, -2.0, 0.0);

    assert_eq!(j1,j1_gt);
    assert_eq!(j2,j2_gt);

}
