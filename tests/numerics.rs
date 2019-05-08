extern crate approx;
extern crate nalgebra as na;
extern crate visual_odometry;

use approx::assert_relative_eq;
use na::{Vector6, DMatrix, Vector3, Matrix3,Matrix4};
use visual_odometry::Float;
use visual_odometry::numerics::{z_standardize, parts_from_isometry, isometry_from_parts};
use visual_odometry::numerics::lie::{exp,ln};


#[test]
fn z_standardization() {
    let mean = 2.5 as Float;
    let variance : Float = 5.0/4.0;
    let std_dev = variance.sqrt() as Float;

    let vec: Vec<Float> = vec![1.0, 2.0, 3.0, 4.0];
    let gt_vec = vec.iter().map(|x| (x-mean)/std_dev).collect();
    let gt = DMatrix::from_vec(2,2, gt_vec);

    let mut mat = DMatrix::from_vec(2,2, vec);
    z_standardize(&mut mat);

    assert_eq!(mat,gt);

}

#[test]
fn lie_exponentials() {

    let lie_1 = Vector6::<Float>::new(1.0, 1.0, 1.0, 1.0, 1.0, 1.0);
    let lie_2 = Vector6::<Float>::new(1.0, 0.5, 0.1, 1.0, 0.0, 1.0);
    let t_3 = Vector3::<Float>::new(2.0, 5.0, 1.0);

    let (so3_1, t_1) = exp(lie_1);
    let lie_new_1 = ln(so3_1, t_1);

    let (so3_2, t_2) = exp(lie_2);
    let lie_new_2 = ln(so3_2, t_2);

    let lie_new_3 = ln(Matrix3::<Float>::identity(), t_3);
    let (so3_new_3, t_new_3) = exp(lie_new_3);

    assert_relative_eq!(lie_new_1, lie_1, epsilon = 0.000000001);
    assert_relative_eq!(lie_new_2, lie_2, epsilon = 0.000000001);
    assert_relative_eq!(t_new_3, t_3, epsilon = 0.000000001);
    assert_relative_eq!(so3_new_3, Matrix3::<Float>::identity(), epsilon = 0.000000001);

}

#[test]
fn isometry() {
    let so3 = Matrix3::<Float>::new(0.5, 0.0, 0.5,
                                    0.4, 0.2, 0.4,
                                    0.1, 0.8, 0.1);
    let t = Vector3::<Float>::new(4.0, 100.0, -25.4);
    let se3 = Matrix4::<Float>::new(0.5, 0.0, 0.5, 4.0,
                                    0.4, 0.2, 0.4, 100.0,
                                    0.1, 0.8, 0.1, -25.4,
                                    0.0, 0.0, 0.0, 1.0);

    let (so3_new, t_new) = parts_from_isometry(se3);
    let se3_new = isometry_from_parts(so3,t);

    assert_eq!(so3_new, so3);
    assert_eq!(t_new, t);
    assert_eq!(se3_new, se3);
}