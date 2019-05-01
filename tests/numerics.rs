extern crate approx;
extern crate nalgebra as na;
extern crate visual_odometry;

use approx::assert_relative_eq;
use na::{Vector6, DMatrix, Vector3, Matrix3};
use visual_odometry::numerics::MatrixData;
use visual_odometry::numerics::matrix_ops::z_standardise;
use visual_odometry::numerics::lie::{exp,ln,};


#[test]
fn z_standardization() {
    let mean = 2.5 as MatrixData;
    let variance : MatrixData = 5.0/4.0;
    let std_dev = variance.sqrt() as MatrixData;

    let vec: Vec<MatrixData> = vec![1.0,2.0,3.0,4.0];
    let gt_vec = vec.iter().map(|x| (x-mean)/std_dev).collect();
    let gt = DMatrix::from_vec(2,2, gt_vec);

    let mut mat = DMatrix::from_vec(2,2, vec);
    let res = z_standardise(&mut mat);

    assert_eq!(*res,gt);

}

#[test]
fn lie_exponentials() {

    let lie_1 = Vector6::<MatrixData>::new(1.0, 1.0, 1.0, 1.0, 1.0, 1.0);
    let lie_2 = Vector6::<MatrixData>::new(1.0, 0.5, 0.1, 1.0, 0.0, 1.0);
    let t_3 = Vector3::<MatrixData>::new(2.0, 5.0, 1.0);

    let (so3_1, t_1) = exp(lie_1);
    let lie_new_1 = ln(so3_1, t_1);

    let (so3_2, t_2) = exp(lie_2);
    let lie_new_2 = ln(so3_2, t_2);

    let lie_new_3 = ln(Matrix3::<MatrixData>::identity(), t_3);
    let (so3_new_3, t_new_3) = exp(lie_new_3);

    assert_relative_eq!(lie_new_1, lie_1, epsilon = 0.000000001);
    assert_relative_eq!(lie_new_2, lie_2, epsilon = 0.000000001);
    assert_relative_eq!(t_new_3, t_3, epsilon = 0.000000001);
    assert_relative_eq!(so3_new_3, Matrix3::<MatrixData>::identity(), epsilon = 0.000000001);

}