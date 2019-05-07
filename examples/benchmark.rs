extern crate image;
extern crate visual_odometry;


use std::path::Path;
use std::time::Instant;
use visual_odometry::image::Image;
use visual_odometry::image::types::ImageFilter;
use visual_odometry::{Frame, solve, MatrixData};
use visual_odometry::camera::intrinsics::Intrinsics;
use visual_odometry::camera::Camera;
use visual_odometry::io::read_png_16bits_row_major;

#[allow(non_snake_case)]
fn main() {
    let image_name_1 = "1311868174.699578";
    let depth_name_1 = "1311868174.687374";
    let image_name_2 = "1311868174.731625";
    let depth_name_2 = "1311868174.719933";
    let image_format = "png";

    let image_1_path = format!("images/{}.{}", image_name_1, image_format);
    let depth_1_path = format!("images/{}.{}", depth_name_1, image_format);
    let image_2_path = format!("images/{}.{}", image_name_2, image_format);
    let depth_2_path = format!("images/{}.{}", depth_name_2, image_format);

    let depth_factor = 5000.0;

    //TODO: simplify this process -> Maybe put this into Frame
    //TODO: ----
    let image_1_im_rs = image::open(&Path::new(&image_1_path)).unwrap().to_luma();
    let (width,height,depth_1_im)
        = read_png_16bits_row_major(depth_1_path)
        .unwrap_or_else(|_| panic!("Could not read image"));
    let image_2_im_rs = image::open(&Path::new(&image_2_path)).unwrap().to_luma();
    let (_,_,depth_2_im)
        = read_png_16bits_row_major(depth_2_path)
        .unwrap_or_else(|_| panic!("Could not read image"));

    let intensity_1 = Image::from_image(image_1_im_rs.clone(), ImageFilter::None, true);
    let mut depth_1 = Image::from_vec_16(height,width,&depth_1_im, false);
    let gx = Image::from_image(image_1_im_rs.clone(), ImageFilter::SobelX, false);
    let gy = Image::from_image(image_1_im_rs.clone(), ImageFilter::SobelY, false);

    let intensity_2 = Image::from_image(image_2_im_rs.clone(), ImageFilter::None, true);
    let mut depth_2 = Image::from_vec_16(height,width,&depth_2_im, false);
    let gx_2 = Image::from_image(image_2_im_rs.clone(), ImageFilter::SobelX, false);
    let gy_2 = Image::from_image(image_2_im_rs.clone(), ImageFilter::SobelY, false);


    depth_1.buffer /= depth_factor;
    depth_2.buffer /= depth_factor;
    //TODO: ----

    let max_depth = depth_1.buffer.amax();

    let reference_frame = Frame{intensity:intensity_1, depth: depth_1, gradient_x: gx, gradient_y: gy};
    let target_frame = Frame{intensity:intensity_2, depth: depth_2, gradient_x: gx_2, gradient_y: gy_2};

    let fx = 520.9;
    let fy = 521.0;
    let ox = 321.5;
    let oy = 249.7;
    let intrinsics = Intrinsics::new(fx,fy,ox,oy);
    let camera = Camera{intrinsics};

    let runs = 100;
    println!("starting solve");
    let now = Instant::now();
    for _ in 0..runs {
        let (_SE3, _lie)
            = solve(&reference_frame,
                    &target_frame,
                    camera,
                    1000,
                    0.00000000001,
                    1.0,
                    max_depth,
                    0.0001,
                    100000.0,
                    100,
                    0,
                    false);
    }

    let solver_duration = now.elapsed().as_millis();
    let average_duration = solver_duration as MatrixData/runs as MatrixData;
    println!("Average Solver duration: {} ms/run for {} runs",average_duration, runs);
    //println!("{}",SE3);
    //println!("{}",lie)



}
