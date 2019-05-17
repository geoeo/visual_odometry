extern crate nalgebra as na;

use crate::Float;
use crate::numerics::column_major_index;

// @GPU
pub fn generate_weights(residuals: &Vec<Float>,
                        weights: &mut Vec<Float>,
                        variance: Float,
                        degrees_of_freedom: usize,
                        width: usize,
                        height: usize,
                        image_offset: usize)
                        -> () {
    let numerator = (degrees_of_freedom + 1) as Float;
    for x in image_offset..(width-image_offset) {
        for y in image_offset..(height-image_offset) {
            let idx = column_major_index(y,x,height);
            let residual = residuals[idx];
            let res_sqrd = residual * residual;
            weights[idx] = numerator / (degrees_of_freedom as Float + (res_sqrd / variance));
        }
    }
}

pub fn t_dist_variance(residuals: &Vec<Float>,
                       degrees_of_freedom: usize,
                       variance_min: Float,
                       eps: Float,
                       max_it: usize,
                       width: usize,
                       height: usize,
                       image_offset: usize) -> Float {
    let mut variance = variance_min;
    let mut variance_prev = variance_min;
    for _it in 0..max_it {
        variance = t_dist_variance_step(residuals,
                                        degrees_of_freedom,
                                        variance_prev,
                                        width,
                                        height,
                                        image_offset);
        if (variance_prev - variance).abs() < eps || variance == 0.0 || variance.is_infinite() || variance.is_nan() {
            break;
        }
        variance_prev = variance;
    };

    variance
}

//TODO: @Investigate -> Weighting with image offset
#[allow(non_snake_case)]
fn t_dist_variance_step(residuals: &Vec<Float>,
                        degrees_of_freedom: usize,
                        variance_prev: Float,
                        width: usize,
                        height: usize,
                        image_offset: usize) -> Float {
    let numerator = degrees_of_freedom as Float + 1.0;
    let mut variance = variance_prev;
    let N = residuals.len();
    for x in image_offset..(width-image_offset) {
        for y in image_offset..(height-image_offset){
            let idx = column_major_index(y,x,height);
            let res = residuals[idx];
            let res_sqrd = res * res;
            let denominator = (degrees_of_freedom as Float) + (res_sqrd / variance_prev);
            variance += (numerator / denominator) * res_sqrd;
        }
    }
    variance / (N as Float)
}

