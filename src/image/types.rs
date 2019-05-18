use crate::Float;

#[repr(u16)]
#[derive(Debug,Copy,Clone)]
pub enum ImageFilter {
    None,
    SobelX,
    SobelY,
    ScharrX,
    ScharrY
}

#[repr(u8)]
#[derive(Debug,Copy,Clone,PartialEq)]
pub enum ImageEncoding {
    U8,
    U16,
    S16,
    F64
}

impl ImageEncoding {
    // https://en.wikipedia.org/wiki/Normalization_(image_processing)
    //TODO: Make using f64 better
    pub fn normalize_to_gray(&self, max: Float, min : Float, value: Float) -> u8 {
//        let max
//            = match self {
//            ImageEncoding::U8 => std::u8::MAX as Float,
//            ImageEncoding::U16 => std::u16::MAX as Float,
//            ImageEncoding::S16 => panic!("normalize_to_gray not implemented for S16"),
//            ImageEncoding::F64 => std::u8::MAX as Float,
//        } as Float;
//
//        let min
//            = match self {
//            ImageEncoding::U8 => std::u8::MIN as Float,
//            ImageEncoding::U16 => std::u16::MIN as Float,
//            ImageEncoding::S16 => panic!("normalize_to_gray not implemented for S16"),
//            ImageEncoding::F64 => std::u8::MIN as Float,
//        } as Float;

        let v =
            match value < 0.0 {
                true => 0.0,
                false => value
            };
        let range = 255 as Float; // 255 - 0
        ((v - min) * (range / (max - min))) as u8
    }
}

