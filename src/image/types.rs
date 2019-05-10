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
    pub fn normalize_to_gray(&self, value: Float) -> u8 {
        let max
            = match self {
            ImageEncoding::U8 => 255,
            ImageEncoding::U16 => 65535, // 256*256-1
            ImageEncoding::S16 => panic!("normalizing with S16 not implemented"),
            ImageEncoding::F64 => panic!("normalizing with F64 not implemented")
        } as Float;

        let range = 255 as Float; // 255 - 0
        (value * range / max) as u8
    }
}

