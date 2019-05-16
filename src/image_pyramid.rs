use crate::{Image, Float, Integer};

pub struct Layer {
    pub intensity: Image,
    pub gradient_x: Image,
    pub gradient_y: Image,
    pub layer_index: usize
}

impl Layer {
    //TODO: use the formulae from the paper to generate x,y coodinates for the full res depth image
    // (2^L){x,y} + 0.5(2^L - 1) -> probably floor the result
    pub fn generate_depth_coordiantes(&self, x: usize, y: usize) -> (Integer, Integer) {
        (-1,-1)
    }
}