extern crate nalgebra as na;
extern crate visual_odometry;

use na::DMatrix;
use visual_odometry::numerics::MatrixData;
use visual_odometry::numerics::matrix_ops::z_standardise;

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