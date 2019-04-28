#[repr(u8)]
#[derive(Debug,Copy,Clone)]
pub enum ImageFilter {
    Sobel,
    Scharr
}

#[repr(u8)]
#[derive(Debug,Copy,Clone)]
pub enum GradientDirection {
    X,
    Y
}