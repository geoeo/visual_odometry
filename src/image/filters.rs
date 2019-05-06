pub static HORIZONTAL_SOBEL: [f32; 9] = [
    -1.0, 0.0, 1.0,
    -2.0, 0.0, 2.0,
    -1.0, 0.0, 1.0];

pub static VERTICAL_SOBEL: [f32; 9] = [
    -1.0, -2.0, -1.0,
    0.0,  0.0,  0.0,
    1.0,  2.0,  1.0];

pub static HORIZONTAL_SCHARR: [f32; 9] = [
    -3.0, 0.0, 3.0,
    -10.0, 0.0, 10.0,
    -3.0, 0.0, 3.0];

pub static VERTICAL_SCHARR: [f32; 9] = [
    -3.0, -10.0, -3.0,
    0.0,  0.0,  0.0,
    3.0,  10.0,  3.0];