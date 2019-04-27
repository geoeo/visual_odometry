extern crate image;

use std::fs::File;
use std::path::Path;

use image::GenericImageView;

fn main() {

    let image_path = "images/ferris.png";
    let file_out_path = "output/test";

    let image = image::open(&Path::new(&image_path)).unwrap();

    // The dimensions method returns the images width and height
    println!("dimensions {:?}", image.dimensions());

    // The color method returns the image's ColorType
    println!("{:?}", image.color());

    let fout = &mut File::create(&Path::new(&format!("{}.png", file_out_path))).unwrap();

    // Write the contents of this image to the Writer in PNG format.
    image.write_to(fout, image::PNG).unwrap();
}