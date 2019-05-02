extern crate image;

use std::fs::File;
use std::path::Path;

use image::GenericImageView;

fn main() {

    let image_name = "depth";
    let image_format = "png";
    let image_path = format!("images/{}.{}",image_name,image_format);
    let file_out_path = format!("output/{}_load_save",image_name);

    let image = image::open(&Path::new(&image_path)).unwrap();

    // The dimensions method returns the images width and height
    println!("dimensions {:?}", image.dimensions());

    // The color method returns the image's ColorType
    println!("{:?}", image.color());

    let fout = &mut File::create(&Path::new(&format!("{}.png", file_out_path))).unwrap();

    // Write the contents of this image to the Writer in PNG format.
    image.write_to(fout, image::PNG).unwrap();
}