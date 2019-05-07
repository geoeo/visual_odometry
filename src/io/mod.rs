use byteorder::{BigEndian, ReadBytesExt};
use png::{self, HasParameters};
use std::{self, fs::File, io::Cursor,path::Path, path::PathBuf};


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

pub fn generate_image_depth_path_lists(image_folder_path: PathBuf, start_idx: String, extension: String, count: usize)
    -> () {

}

