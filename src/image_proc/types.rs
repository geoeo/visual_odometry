#[derive(Debug,Copy,Clone)]
pub enum ImageFilter {
    Sobel,
    Scharr
}

#[derive(Debug,Copy,Clone)]
pub enum GradientDirection {
    X,
    Y
}