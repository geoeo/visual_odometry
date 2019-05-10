extern crate nalgebra as na;

use crate::Float;

// @GPU
pub fn generate_weights(residuals: &Vec<Float>, weights: &mut Vec<Float>, variance: Float, degrees_of_freedom: usize)
                        -> () {
    let numerator = (degrees_of_freedom + 1) as Float;
    let number_of_samples = residuals.len();
    for i in 0..number_of_samples {
        let residual = residuals[i];
        let temp = residual / variance;
        let temp_sqrd = temp*temp;
        //let frac = temp_sqrd/variance;
        weights[i] = numerator / (degrees_of_freedom as Float + temp_sqrd);
    }
}

pub fn t_dist_variance(residuals: &Vec<Float>,
                       valid_measurements_reference: &Vec<bool>,
                       valid_measurements_target: &Vec<bool>,
                       number_of_valid_measurements: usize,
                       degrees_of_freedom: usize,
                       variance_min: Float,
                       eps: Float,
                       max_it: usize) -> Float {
    let mut variance = variance_min;
    let mut variance_prev = variance_min;
    for _ in 0..max_it {
        variance = t_dist_variance_step(residuals,
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
fn t_dist_variance_step(residuals: &Vec<Float>,
                        valid_measurements_reference: &Vec<bool>,
                        valid_measurements_target: &Vec<bool>,
                        number_of_valid_measurements: usize,
                        degrees_of_freedom: usize,
                        variance_prev: Float) -> Float {
    let numerator = degrees_of_freedom as Float + 1.0;
    let mut variance = variance_prev;
    let N = residuals.len();
    for i in 0..N {
        if !valid_measurements_reference[i] || !valid_measurements_target[i] {
            continue;
        }
        let res = residuals[i];
        let res_sqrd = res * res;
        let denominator = (degrees_of_freedom as Float) + (res_sqrd / variance_prev);
        variance += (numerator / denominator) * res_sqrd;
    }
    variance / (number_of_valid_measurements as Float)
}

