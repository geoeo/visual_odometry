#[repr(u8)]
#[derive(Debug,Copy,Clone)]
pub enum ImageFilter {
    None,
    Sobel,
    Scharr
}

#[repr(u8)]
#[derive(Debug,Copy,Clone)]
pub enum GradientDirection {
    None,
    X,
    Y
}