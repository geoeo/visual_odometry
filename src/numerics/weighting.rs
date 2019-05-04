extern crate nalgebra as na;

use crate::MatrixData;
use nalgebra::DVector;

pub fn t_dist_variance(residuals: &DVector<MatrixData>,
                       valid_measurements_reference: &DVector<bool>,
                       valid_measurements_target: &DVector<bool>,
                       number_of_valid_measurements: usize,
                       degrees_of_freedom: usize,
                       variance_prev: MatrixData,
                       variance_min: MatrixData,
                       eps: MatrixData,
                       max_it : usize) -> MatrixData {
    let mut variance = variance_min;
    let mut variance_prev = variance_min;
    for i in 0..max_it {
        let variance = t_dist_variance_step(residuals,
                                            valid_measurements_reference,
                                            valid_measurements_target,
                                            number_of_valid_measurements,
                                            degrees_of_freedom,
                                            variance_prev);
        if (variance_prev - variance).abs() < eps || variance == 0.0 || variance.is_infinite() || variance.is_nan() {
            break;
        }
        variance_prev = variance;
    };

    variance
}

#[allow(non_snake_case)]
fn t_dist_variance_step(residuals: &DVector<MatrixData>,
                        valid_measurements_reference: &DVector<bool>,
                        valid_measurements_target: &DVector<bool>,
                        number_of_valid_measurements: usize,
                        degrees_of_freedom: usize,
                        variance_prev: MatrixData) -> MatrixData {
    let numerator = degrees_of_freedom as MatrixData + 1.0;
    let mut variance = variance_prev;
    let N = residuals.ncols();
    for i in 0..N {
        if !valid_measurements_reference.index(i) || !valid_measurements_target.index(i) {
            continue;
        }
        let res = residuals.index(i);
        let res_sqrd = res * res;
        let denominator = (degrees_of_freedom as MatrixData) + (res_sqrd / variance_prev);
        variance += (numerator / denominator) * res_sqrd;
    }
    variance / (number_of_valid_measurements as MatrixData)
}

