use crate::MatrixData;

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
    U16
}

impl ImageEncoding {
    // https://en.wikipedia.org/wiki/Normalization_(image_processing)
    pub fn normalize_to_gray(&self, value: MatrixData) -> u8 {
        let max
            = match self {
            ImageEncoding::U8 => 255,
            ImageEncoding::U16 => 65535 // 256*256-1
        } as MatrixData;

        let range = 255 as MatrixData; // 255 - 0
        (value * range / max) as u8
    }
}
