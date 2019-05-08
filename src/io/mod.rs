use byteorder::{BigEndian, ReadBytesExt};
use png::{self, HasParameters};
use std::{self, fs::File, io::Cursor,path::Path};
use std::io;
use std::fs::read_dir;


// https://github.com/mpizenberg/visual-odometry-rs/blob/master/src/misc/helper.rs#L13
/// Read a 16 bit gray png image from a file.
pub fn read_png_16bits_row_major<P: AsRef<Path>>(
    file_path: P,
) -> Result<(usize, usize, Box<Vec<u16>>), png::DecodingError> {
    // Load 16 bits PNG depth image.
    let img_file = File::open(file_path)?;
    let mut decoder = png::Decoder::new(img_file);
    // Use the IDENTITY transformation because by default
    // it will use STRIP_16 which only keep 8 bits.
    // See also SWAP_ENDIAN that might be useful
    //   (but seems not possible to use according to documentation).
    decoder.set(png::Transformations::IDENTITY);
    let (info, mut reader) = decoder.read_info()?;
    let mut buffer = vec![0; info.buffer_size()];
    reader.next_frame(&mut buffer)?;

    // Transform buffer into 16 bits slice.
    // Transform buffer into 16 bits slice.
    // if cfg!(target_endian = "big") ...
    let mut buffer_u16 = vec![0; (info.width * info.height) as usize];
    let mut buffer_cursor = Cursor::new(buffer);
    buffer_cursor.read_u16_into::<BigEndian>(&mut buffer_u16)?;

    // Return u16 buffer.
    Ok((info.width as usize, info.height as usize, Box::new(buffer_u16)))
}

pub fn get_file_list_in_dir<P: AsRef<Path>>(image_folder_path: P) -> io::Result<Vec<String>> {

    let mut file_vec: Vec<String> = Vec::new();
    for entry in read_dir(image_folder_path)? {
        let entry = entry?;
        let path = entry.path();
        if path.is_dir(){
            continue;
        } else {
            let name = entry.file_name().into_string().unwrap_or_else(|_| panic!("Error reading filename"));
            if is_valid_file(&name) {
                file_vec.push(name);
            }
        }
    }

    Ok(file_vec)

}

fn is_valid_file(file_name: &str) -> bool {
    !(file_name == ".DS_Store" || file_name == "._.DS_Store")
}

pub fn file_name_to_float(filename: &str) -> f64 {
    let splits = filename.split(".");
    let vec: Vec<&str> = splits.collect();
    let float_str = format!("{}.{}",vec[0],vec[1]);
    float_str.parse().unwrap_or_else(|_| panic!("unable to convert filename to float"))
}

pub fn associate_file_name<P: AsRef<Path>>(image_folder_path: P, file_name: &str)-> io::Result<String> {
    let file_name_list = get_file_list_in_dir(image_folder_path)?;
    let file_name_to_match_as_float = file_name_to_float(file_name);
    let time_stamp_differences: Vec<f64>
        = file_name_list.iter()
        .map(|x| file_name_to_float(x))
        .map(|x| (file_name_to_match_as_float-x).abs())
        .collect();
    let (closet_match_idx, _)
        = time_stamp_differences
        .iter()
        .enumerate()
        .fold((0, 100.0), |(ts_idx, acc), (i, x)| if *x < acc { (i, *x) } else { (ts_idx, acc) });

    Ok(file_name_list[closet_match_idx].clone())
}

