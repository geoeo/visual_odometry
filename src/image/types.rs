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
#[derive(Debug,Copy,Clone)]
pub enum ImageEncoding {
    U8,
    U16
}

