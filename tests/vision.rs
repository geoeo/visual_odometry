extern crate approx;
extern crate nalgebra as na;
extern crate visual_odometry;

use approx::assert_relative_eq;
use na::{Matrix2x3, DMatrix, Matrix3x4, Matrix3x6, DVector};
use visual_odometry::MatrixData;
use visual_odometry::camera::intrinsics::Intrinsics;
use visual_odometry::camera::Camera;
use visual_odometry::gauss_newton_routines::back_project;

#[test]
#[allow(non_snake_case)]
fn back_project_then_project(){
    let K = Matrix3x4::<MatrixData>::new(2.0, 0.0, 5.0,0.0,
                                         0.0,1.0,0.5,0.0,
                                         0.0,0.0,1.0,0.0);
    let K_inv = Intrinsics::invert(K);
    let mut P = vec![1.0,2.0,1.0,1.0];
    let mut P2 = vec![2.0,1.0,2.0,1.0];
    P.append(&mut P2);

    let camera = Camera { intrinsics: Intrinsics {K,K_inv}};
    let points = DMatrix::<MatrixData>::from_vec(4,2,P);

    let depth_reference = DMatrix::<MatrixData>::from_vec(2,1,vec![1.0, 2.0]);
    let depth_target = DMatrix::<MatrixData>::from_vec(2,1,vec![1.0, 1.0]); // doesnt matter
    let (back_projections,
        valid_ref,
        valid_target)
        = back_project(camera,
                       &depth_reference,
                       &depth_target,
                       1,
                       2
                       , 100.0);

    let projections = camera.apply_perspective_projection(&back_projections);
    let u_1 = *projections.index((0,0));
    let v_1 = *projections.index((1,0));

    let u_2 = *projections.index((0,1));
    let v_2 = *projections.index((1,1));

    assert_eq!(u_1,0.0);
    assert_eq!(v_1,0.0);

    assert_eq!(u_2,0.0);
    assert_eq!(v_2,1.0);



}